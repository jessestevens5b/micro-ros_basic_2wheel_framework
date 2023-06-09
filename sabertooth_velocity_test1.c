//By Jesse Stevens
/*TODO: - add dynamic ability to change timing of feedback (by creating/destroying executors)
 *      - neaten up everything into functions and probably separate files
 *      - add rollover handling for encoder counts (not super urgent yet)
 *      - further test closed loop speed control
 *      - definitely understand if we really need tf to be coming from the vehicle, I think it should be
 *          "robot_localization" package in ros to fuse data from raw inputs
 *      - Add further abilities for LED feedback etc
 *      - Action buttons?
 *      - build out on circuit board for better encoder testing
 *
 *
 * Currently:
 *      - 500PPR encoders (omron)
 *      - pull up resistors (1k) to 3.3v (encoders pull down)
 *      - Raspberry Pi Pico
 *      - Sabertooth Motor driver (change to VESC in time)
 *
 */
//#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/init_options.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#include <geometry_msgs/msg/twist.h>
#include "nav_msgs/msg/odometry.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "math.h"
#include <geometry_msgs/msg/quaternion.h>
#include "rosidl_runtime_c/string_functions.h"
#include <string.h>
#include <std_msgs/msg/float64.h>

//for debugging topic feedback:
//#define DEBUG_ENABLED

const uint LED_PIN = 25;

//quick hack to convert MS to NS for timer: (needed?)
#define MS_TO_NS(ms) ((ms) * 1000000ULL)

// WHEELS AND ENCODERS:
const uint encoder_resolution = 500; //ticks per revolution of ground measuring wheel (if on wheel shaft: encoder)
volatile int32_t left_encoder_count = 0; //for storing current count (updated in ISR)
volatile int32_t right_encoder_count = 0; //for storing current count (updated in ISR)
const float wheel_radius = 0.020; //20mm radius (40mm diameter)
const float wheel_base_distance = 1.555; //1555mm spacing between wheels

//PID closed loop speed control:
float prev_left_error = 0;
float prev_right_error = 0;
int64_t last_pid_run = 0;
int64_t last_odom_calc = 0;

//motor cutoff (when lost contact from ROS):
static int64_t last_cmd_received_time = 0;

float target_left_wheel_velocity = 0;
float target_right_wheel_velocity = 0;

// Robot position and orientation (global)
double robot_x = 0.0;
double robot_y = 0.0;
double robot_theta = 0.0;

//encoder input pins:
const int LEncoderA = 6;
const int LEncoderB = 7;
const int REncoderA = 8;
const int REncoderB = 9;


//our actual pubs and subs:
rcl_publisher_t odom; //odometry publishing
rcl_subscription_t cmd_vel_sub; //to subscribe to velocity topic
rcl_subscription_t updated_odom_sub; //to subscribe for updated odometry from sensor fusion in ROS2

#ifdef DEBUG_ENABLED
//DEBUG:
rcl_publisher_t debug_publisher;
std_msgs__msg__Float64 debug_msg;
#endif

rclc_support_t support; //what is this? Still seems needed though

//odometry message types
nav_msgs__msg__Odometry odom_msg;
nav_msgs__msg__Odometry updated_odom;

//our cmd_vel message store:
geometry_msgs__msg__Twist cmd_vel;

void left_encoder_process() {
    static const int8_t left_encoder_direction = 1;
    volatile static uint8_t old_a_state = 0;
    uint8_t a_state = gpio_get(LEncoderA);

    if (a_state != old_a_state) {
        old_a_state = a_state;
        if (a_state == 1) {
            if (left_encoder_direction == 1) {
                if (gpio_get(LEncoderB) == 0) {
                    left_encoder_count++;
                } else {
                    left_encoder_count--;
                }
            } else {
                if (gpio_get(LEncoderB) == 1) {
                    left_encoder_count++;
                } else {
                    left_encoder_count--;
                }
            }
        } else {
            if (left_encoder_direction == 1) {
                if (gpio_get(LEncoderB) == 1) {
                    left_encoder_count++;
                } else {
                    left_encoder_count--;
                }
            } else {
                if (gpio_get(LEncoderB) == 0) {
                    left_encoder_count++;
                } else {
                    left_encoder_count--;
                }
            }
        }
    }
}

void right_encoder_process() {
    static const int8_t right_encoder_direction = 0;
    volatile static uint8_t old_a_state = 0;
    uint8_t a_state = gpio_get(REncoderA);

    if (a_state != old_a_state) {
        old_a_state = a_state;
        if (a_state == 1) {
            if (right_encoder_direction == 1) {
                if (gpio_get(REncoderB) == 0) {
                    right_encoder_count++;
                } else {
                    right_encoder_count--;
                }
            } else {
                if (gpio_get(REncoderB) == 1) {
                    right_encoder_count++;
                } else {
                    right_encoder_count--;
                }
            }
        } else {
            if (right_encoder_direction == 1) {
                if (gpio_get(REncoderB) == 1) {
                    right_encoder_count++;
                } else {
                    right_encoder_count--;
                }
            } else {
                if (gpio_get(REncoderB) == 0) {
                    right_encoder_count++;
                } else {
                    right_encoder_count--;
                }
            }
        }
    }
}

//for some reason we can only have one interrupt callback, so we have to process on interrupt which gpio triggered it:
void encoder_isr( uint gpio, uint32_t events){

    switch (gpio){
        case LEncoderA:
            left_encoder_process();
            return;
        case REncoderA:
            right_encoder_process();
            return;
    }
}

// Function to read encoder data and calculate wheel velocities
void read_encoders_and_compute_wheel_velocities(float *left_wheel_velocity, float *right_wheel_velocity) {
    // Store the previous encoder counts and current time to calculate the velocity
    static int32_t prev_left_encoder_count = 0;
    static int32_t prev_right_encoder_count = 0;
    static int64_t prev_time_us = 0;

    // Get the current encoder counts and time
    int32_t curr_left_encoder_count = left_encoder_count;
    int32_t curr_right_encoder_count = right_encoder_count;
    int64_t curr_time_us = time_us_64();

    // Calculate the elapsed time since the last call (in seconds)
    float elapsed_time = (curr_time_us - prev_time_us) / 1000000.0f;

    // Calculate the change in encoder counts for left and right wheels
    int32_t delta_left_encoder_count = curr_left_encoder_count - prev_left_encoder_count;
    int32_t delta_right_encoder_count = curr_right_encoder_count - prev_right_encoder_count;

    // Handle rollover for encoder counts
    if (delta_left_encoder_count > encoder_resolution / 2) {
        delta_left_encoder_count -= encoder_resolution;
    } else if (delta_left_encoder_count < -encoder_resolution / 2) {
        delta_left_encoder_count += encoder_resolution;
    }

    if (delta_right_encoder_count > encoder_resolution / 2) {
        delta_right_encoder_count -= encoder_resolution;
    } else if (delta_right_encoder_count < -encoder_resolution / 2) {
        delta_right_encoder_count += encoder_resolution;
    }

    // Calculate the wheel rotations (in revolutions) based on the change in encoder counts
    float left_wheel_rotations = (float) delta_left_encoder_count / encoder_resolution;
    float right_wheel_rotations = (float) delta_right_encoder_count / encoder_resolution;

    // Calculate the distance traveled by each wheel (in meters)
    float left_wheel_distance = left_wheel_rotations * 2.0 * M_PI * wheel_radius;
    float right_wheel_distance = right_wheel_rotations * 2.0 * M_PI * wheel_radius;

    // Calculate the wheel velocities (in meters per second) and store them globally:
    *left_wheel_velocity = left_wheel_distance / elapsed_time;
    *right_wheel_velocity = right_wheel_distance / elapsed_time;

    // Update the previous encoder counts and time
    prev_left_encoder_count = curr_left_encoder_count;
    prev_right_encoder_count = curr_right_encoder_count;
    prev_time_us = curr_time_us;
}

//quick convert so we can just throw ints into motors:
void sendMotor(int input){
    uart_putc_raw(uart1, (uint8_t)input);
}

//for use by safety cutoff to stop motors:
void stop_motors() {
    target_left_wheel_velocity = 0;
    target_right_wheel_velocity = 0;
    sendMotor( 64); // Stop left motor
    sendMotor( 192); // Stop right motor
}

//for closed loop velocity based motor speed control and safety cut-off:
void motor_speed_control(){
    //motor safety cutoff
    static int64_t safety_timeout_us = 500 * 1000; // 500ms in microseconds

    //motor driver send frequency limiting and storage of speed:
    static int64_t last_motor_send = 0;
    static int64_t motor_send_space = 150 * 1000; //100ms in microseconds

    // Check for inactivity and stop motors if necessary
    //if (time_us_64() - last_cmd_received_time > safety_timeout_us) {
    if(last_motor_send == 24){ //some bullshit to keep the if happy
        /*
        stop_motors();
        last_cmd_received_time = time_us_64(); // Update last_cmd_received_time to prevent continuous motor stop commands

        //test:
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
         */
    }
    else {
        //we need to schedule our motor sending so we don't overwhelm the controller:
        if ((time_us_64() - last_motor_send) > motor_send_space) {

            // Define PID controller gains
            const float Kp = 150.0;
            const float Ki = 1.0;
            const float Kd = 5.0;

            float left_wheel_velocity = 0.0f;
            float right_wheel_velocity = 0.0f;
            read_encoders_and_compute_wheel_velocities(&left_wheel_velocity, &right_wheel_velocity);

            //time for use in PID:
            float elapsed_time = (time_us_64() - last_pid_run) / 1000000.0f; //convert back to seconds from microseconds

            // Compute the error between the target velocity and the current velocity
            float left_error = target_left_wheel_velocity - left_wheel_velocity;
            float right_error = target_right_wheel_velocity - right_wheel_velocity;

            // Compute the error integral for each wheel
            static float left_error_integral = 0;
            static float right_error_integral = 0;
            float left_error_derivative = 0;
            float right_error_derivative = 0;

            if (fabs(left_error) < 0.001) {
                left_error_integral = 0; //if close to zero, make it zero
            } else {
                left_error_integral += left_error * elapsed_time;
            }

            if (fabs(right_error) < 0.001) {
                right_error_integral = 0; //if close to zero, make it zero
            } else {
                right_error_integral += right_error * elapsed_time;
            }

            // Compute the error derivative for each wheel
            left_error_derivative = (left_error - prev_left_error) / elapsed_time;
            right_error_derivative = (right_error - prev_right_error) / elapsed_time;

            // Compute the control signal
            float left_control_signal = 0.0f;
            float right_control_signal = 0.0f;

            //make sure we don't jitter when floating point problems make us not exactly zero:
            if (target_left_wheel_velocity != 0.0f){
                left_control_signal = Kp * left_error + Ki * left_error_integral + Kd * left_error_derivative;
            }

            if (target_right_wheel_velocity != 0.0f){
                right_control_signal = Kp * right_error + Ki * right_error_integral + Kd * right_error_derivative;
            }

            // Apply the control signal by sending it to the motor driver
            // The control signal should be converted to the appropriate format for the motor driver
            // Here, we assume that the control signal is a percentage of the maximum motor speed
            // and clamp it between -100% and 100%
            int16_t left_motor_speed = (int16_t) (64 + (0.64 * left_control_signal));
            int16_t right_motor_speed = (int16_t) (192 + (0.64 * right_control_signal));

            left_motor_speed = left_motor_speed > 127 ? 127 : left_motor_speed;
            left_motor_speed = left_motor_speed < -127 ? -127 : left_motor_speed;
            right_motor_speed = right_motor_speed > 255 ? 255 : right_motor_speed;
            right_motor_speed = right_motor_speed < 128 ? 128 : right_motor_speed;
            sendMotor(left_motor_speed);
            sendMotor(right_motor_speed);

            // Save the errors for the next iteration
            prev_left_error = left_error;
            prev_right_error = right_error;

            //save when we've run so we can know the elapsed time in future:
            last_pid_run = time_us_64();

            //so we don't overwhelm the controller with too much data
            last_motor_send = time_us_64();
        }
    }
}

//processes the twist commands target velocities (callback)
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist *) msgin;
    float linear_vel = msg->linear.x;
    float angular_vel = msg->angular.z;

    target_left_wheel_velocity = (linear_vel - angular_vel);
    target_right_wheel_velocity = (linear_vel + angular_vel);

    //for safety cutoff store the current time (uS since boot):
    last_cmd_received_time = time_us_64();

    //bit of feedback to show data is passing:
    //gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

void quaternion_to_euler(const geometry_msgs__msg__Quaternion *q, double *roll, double *pitch, double *yaw) {
    double sinr_cosp = 2 * (q->w * q->x + q->y * q->z);
    double cosr_cosp = 1 - 2 * (q->x * q->x + q->y * q->y);
    *roll = atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (q->w * q->y - q->z * q->x);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp);
    else
        *pitch = asin(sinp);

    double siny_cosp = 2 * (q->w * q->z + q->x * q->y);
    double cosy_cosp = 1 - 2 * (q->y * q->y + q->z * q->z);
    *yaw = atan2(siny_cosp, cosy_cosp);
}

void updated_odom_callback(const void *msgin) {
    const nav_msgs__msg__Odometry *msg = (const nav_msgs__msg__Odometry *)msgin;

    // Update the robot's position and orientation based on the received odometry data
    robot_x = msg->pose.pose.position.x;
    robot_y = msg->pose.pose.position.y;

    double roll, pitch, yaw;
    quaternion_to_euler(&msg->pose.pose.orientation, &roll, &pitch, &yaw);
    robot_theta = yaw;

    // We won't update velocities as this is calculated from encoders and really shouldn't be messed with
}

void quaternion_from_yaw(double yaw, geometry_msgs__msg__Quaternion *quaternion) {
    // Compute the quaternion values from the yaw (theta) angle
    double half_yaw = yaw * 0.5;
    double c_yaw = cos(half_yaw);
    double s_yaw = sin(half_yaw);

    // Assign the quaternion components
    quaternion->x = 0.0;
    quaternion->y = 0.0;
    quaternion->z = s_yaw;
    quaternion->w = c_yaw;
}

//processes the distance travelled into odometry feedback to ROS:
void odomtf_calc_send(const rclc_support_t *support){

    static double prev_left_wheel_distance;
    static double prev_right_wheel_distance;
    static int32_t prev_left_encoder_count = 0;
    static int32_t prev_right_encoder_count = 0;

    // Initialize variables for wheel velocities, robot velocities, and pose
    static float left_wheel_velocity, right_wheel_velocity;
    static float linear_velocity, angular_velocity;
    //static double x = 0, y = 0, theta = 0; // Position and orientation of the robot in the odometry frame

    // Get the current encoder counts and time
    int32_t curr_left_encoder_count = left_encoder_count;
    int32_t curr_right_encoder_count = right_encoder_count;
    int64_t curr_time_us = time_us_64();

    // Calculate the elapsed time since the last call (in seconds)
    //float elapsed_time = (curr_time_us - prev_time_us) / 1000000.0f;

    // Calculate the change in encoder counts for left and right wheels:
    int32_t delta_left_encoder_count = curr_left_encoder_count - prev_left_encoder_count;
    int32_t delta_right_encoder_count = curr_right_encoder_count - prev_right_encoder_count;

    // Handle rollover for encoder counts - NEED TO IMPLEMENT FOR THE 4million turn rollover on wheels

    // Calculate the wheel rotations (in revolutions) based on the change in encoder counts
    float left_wheel_rotations = (float) delta_left_encoder_count / encoder_resolution;
    float right_wheel_rotations = (float) delta_right_encoder_count / encoder_resolution;

    // Calculate the distance traveled by each wheel (in meters)
    float left_wheel_distance = left_wheel_rotations * 2.0 * M_PI * wheel_radius;
    float right_wheel_distance = right_wheel_rotations * 2.0 * M_PI * wheel_radius;

    // calculate how long it's been since the last run (in seconds):
    float delta_time = (time_us_64() - last_odom_calc) / 1000000; //delta is in seconds, convert micros to whole seconds


    double delta_left_wheel_distance = left_wheel_distance - prev_left_wheel_distance;
    double delta_right_wheel_distance = right_wheel_distance - prev_right_wheel_distance;

    double delta_distance = (delta_right_wheel_distance + delta_left_wheel_distance) / 2.0;
    double delta_theta = (delta_right_wheel_distance - delta_left_wheel_distance) / wheel_base_distance;

    robot_x += delta_distance * cos(robot_theta + delta_theta / 2.0);
    robot_y += delta_distance * sin(robot_theta + delta_theta / 2.0);
    robot_theta += delta_theta;

    //normalise the output (when wrapping past 360/0 degrees) to -pi to +pi, or 0 to 2pi:
    robot_theta = fmod(robot_theta + 2 * M_PI, 2 * M_PI);

    // Calculate linear and angular velocities
    linear_velocity = delta_distance / delta_time;
    angular_velocity = delta_theta / delta_time;

    // Update previous wheel distances and time
    prev_left_wheel_distance = left_wheel_distance;
    prev_right_wheel_distance = right_wheel_distance;

    // Populate the nav_msgs/Odometry message with the computed pose and velocities

    // Set the header of the Odometry message
    odom_msg.header.stamp.sec = (int32_t) (curr_time_us / 1000000);
    odom_msg.header.stamp.nanosec = (uint32_t) ((curr_time_us % 1000000) * 1000);
    odom_msg.header.frame_id.data = (char *) "odom";
    odom_msg.header.frame_id.size = strlen(odom_msg.header.frame_id.data);
    odom_msg.header.frame_id.capacity = strlen(odom_msg.header.frame_id.data) + 1;
    odom_msg.child_frame_id.data = (char *) "base_link";
    odom_msg.child_frame_id.size = strlen(odom_msg.child_frame_id.data);
    odom_msg.child_frame_id.capacity = strlen(odom_msg.child_frame_id.data) + 1;


    //nav_msgs__msg__Odometry odom_msg;
    odom_msg.pose.pose.position.x = robot_x;
    odom_msg.pose.pose.position.y = robot_y;
    odom_msg.pose.pose.position.z = 0.0;
    quaternion_from_yaw(robot_theta, &odom_msg.pose.pose.orientation);
    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;

    // Publish the odometry message
    rcl_publish(&odom, &odom_msg, NULL);

    //DEBUG DATA:
    #ifdef DEBUG_ENABLED
    debug_msg.data = delta_right_encoder_count;
    rcl_publish(&debug_publisher, &debug_msg, NULL);
    #endif
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    //timer callback bits (for publishing outwards to ROS2)
    //odometry feedback:;
    odomtf_calc_send(&support);
}

void flasher(){
    static int64_t lastflash = 0;
    if (time_us_64() - lastflash > (1000*1000)){
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        lastflash = time_us_64();
    }
}

int main() {
    //stdio_init_all(); //we don't want feedback right now via serial

    rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
    );

    // Initialize GPIO pins for encoders
    gpio_init(LEncoderA);
    gpio_init(LEncoderB);
    gpio_init(REncoderA);
    gpio_init(REncoderB);

    // Initialize GPIO pins for encoders
    gpio_set_dir(LEncoderA, GPIO_IN);
    gpio_set_dir(LEncoderB, GPIO_IN);
    gpio_set_dir(REncoderA, GPIO_IN);
    gpio_set_dir(REncoderB, GPIO_IN);

    // Set up interrupts for encoders
    gpio_set_irq_enabled_with_callback(LEncoderA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &encoder_isr);
    //gpio_set_irq_enabled_with_callback(REncoderA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &right_encoder_isr); //cannot declare more than one callback?
    gpio_set_irq_enabled(REncoderA, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true); //seems this works?

    //LED indicator on board:
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    //uart config for sabertooth motor controller:
    uart_init(uart1, 9600);
    gpio_set_function(4, GPIO_FUNC_UART);
    gpio_set_function(5, GPIO_FUNC_UART);
    uart_set_hw_flow(uart1, false, false); // Disable hardware flow control
    uart_set_format(uart1, 8, 1, UART_PARITY_NONE); // 8 data bits, 1 stop bit, no parity

    //little startup flash:
    gpio_put(LED_PIN, true);
    sleep_ms(500);
    gpio_put(LED_PIN, false);
    sendMotor(64);

    //try to ping/connect to agent:
    rcl_ret_t ret = rmw_uros_ping_agent(500, 200);

    //allocator:
    rcl_allocator_t allocator = rcl_get_default_allocator();

    //support object:
    rclc_support_init(&support, 0, NULL, &allocator); //not pushing args into it

    // create node
    rcl_node_t node = rcl_get_zero_initialized_node();
    //rcl_node_options_t node_ops = rcl_node_get_default_options();
    rclc_node_init_default(&node, "pico_micro_ros_sabertooth", "", &support); //namespace can be empty

    //create our subscription to /cmd_vel:
    rclc_subscription_init_best_effort(
            &cmd_vel_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
            "/cmd_vel");

    //create our subscription to /updated_odom:
    rclc_subscription_init_best_effort(
            &updated_odom_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/updated_odom"
            );

    //create our publisher to /odom
    rclc_publisher_init_default(
            &odom,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "/odom"
            );

    #ifdef DEBUG_ENABLED
    //DEBUG PUBLISHER:
    const rosidl_message_type_support_t * debug_pub_type_support = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64);
    rcl_publisher_options_t debug_pub_options = rcl_publisher_get_default_options();
    rcl_publisher_init(
            &debug_publisher,
            &node,
            debug_pub_type_support,
            "/debug",
            &debug_pub_options
            );
    #endif

    // create timer,
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = 50;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

    //create the executor that harvests the incoming messages and shoots them off to the callback:
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    //rclc_support_init(&support, 0, NULL, &allocator);
    rclc_executor_init(&executor, &support.context, 2, &allocator);

    //timer elements for executor to publish data:
    unsigned int rcl_wait_timeout = 200;   // in ms
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));
    rclc_executor_add_timer(&executor, &timer);

    //add the cmd_vel subscription to the executor:
    rclc_executor_add_subscription(
            &executor,
            &cmd_vel_sub,
            &cmd_vel,
            &cmd_vel_callback,
            ON_NEW_DATA
            );

    //add the updated_odom subscription to the executor:
    rclc_executor_add_subscription(
            &executor,
            &updated_odom_sub,
            &updated_odom,
            &updated_odom_callback,
            ON_NEW_DATA
            );

    //turn the indicator LED on so we know we're awake:
    gpio_put(LED_PIN, 1);

    while (true) {
        rclc_executor_spin_some(&executor, MS_TO_NS(100)); //working now?

        //closed loop speed control and safety cut-off:
        motor_speed_control();

        //the "everything is ok" flasher:
        flasher();

    }

    //if we break out, turn off the LED:
    gpio_put(LED_PIN, 0);

    rcl_subscription_fini(&cmd_vel_sub, &node);
    rcl_node_fini(&node);
    //rcl_shutdown(&context);
    return 0;
}
