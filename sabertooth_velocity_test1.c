

//#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/init_options.h>
//#include <rmw_uros/options.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

//jesse added includes:
#include <geometry_msgs/msg/twist.h>
#include "nav_msgs/msg/odometry.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "math.h"
#include <geometry_msgs/msg/quaternion.h>
#include "rosidl_runtime_c/string_functions.h"
#include <string.h>

const uint LED_PIN = 25;

//quick hack to convert MS to NS for timer: (needed?)
#define MS_TO_NS(ms) ((ms) * 1000000ULL)

// WHEELS AND ENCODERS:
const uint encoder_resolution = 4096; //ticks per revolution (check)
//int32_t last_left_encoder_count = 0; //for storing previous count for comparison (for calculated wheel rotation)
//int32_t last_right_encoder_count = 0; //for storing previous count for comparison (for calculated wheel rotation)
volatile int32_t left_encoder_count = 0; //for storing current count
volatile int32_t right_encoder_count = 0; //for storing current count
const float wheel_radius = 0.025; //25mm
const float wheel_base_distance = 0.35; //350mm spacing
//double last_left_wheel_distance = 0.0; //for distance travelled feedback to odom
//double last_right_wheel_distance = 0.0; //for distance travelled feedback to odom

//PID closed loop speed control:
float prev_left_error = 0;
float prev_right_error = 0;
int64_t last_pid_run = 0;
int64_t last_odom_calc = 0;
int64_t odom_read_space = 100 * 1000; //needed now?

//motor cutoff (when lost contact from ROS:
int64_t last_cmd_received_time = 0;

float left_wheel_velocity = 0;
float right_wheel_velocity = 0;
float target_left_wheel_velocity = 0;
float target_right_wheel_velocity = 0;


//our actual pubs and subs:
rcl_publisher_t odom; //not yet used until we start publishing
rcl_subscription_t cmd_vel_sub;
rclc_support_t support;

//odometry
nav_msgs__msg__Odometry odom_msg;

//our cmd_vel message store:
geometry_msgs__msg__Twist cmd_vel;


void left_encoder_isr() {
    static uint8_t old_a_state = 0;
    uint8_t a_state = gpio_get(6);
    uint8_t b_state = gpio_get(7);

    if (a_state != old_a_state) {
        old_a_state = a_state;
        if (a_state != b_state) {
            left_encoder_count++;
        } else {
            left_encoder_count--;
        }
    }
}

void right_encoder_isr() {
    static uint8_t old_a_state = 0;
    uint8_t a_state = gpio_get(8);
    uint8_t b_state = gpio_get(9);

    if (a_state != old_a_state) {
        old_a_state = a_state;
        if (a_state != b_state) {
            right_encoder_count++;
        } else {
            right_encoder_count--;
        }
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
    static int64_t motor_send_space = 100 * 1000;

    // Check for inactivity and stop motors if necessary
    if (time_us_64() - last_cmd_received_time > safety_timeout_us) {
        stop_motors();
        last_cmd_received_time = time_us_64(); // Update last_cmd_received_time to prevent continuous motor stop commands
    }
    else {
        //we need to schedule our motor sending so we don't overwhelm the controller:
        if ((time_us_64() - last_motor_send) > motor_send_space) {

            float elapsed_time = (time_us_64() - last_pid_run) / 1000; //in ms

            // Define PID controller gains
            const float Kp = 10.0;
            const float Ki = 0.0;
            const float Kd = 1.0;

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
            float left_control_signal = Kp * left_error + Ki * left_error_integral + Kd * left_error_derivative;
            float right_control_signal = Kp * right_error + Ki * right_error_integral + Kd * right_error_derivative;

            // Apply the control signal by sending it to the motor driver
            // The control signal should be converted to the appropriate format for the motor driver
            // Here, we assume that the control signal is a percentage of the maximum motor speed
            // and clamp it between -100% and 100%
            int16_t left_motor_speed = (int16_t) (64 + 0.64 * left_control_signal);
            int16_t right_motor_speed = (int16_t) (192 + 0.64 * right_control_signal);
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
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
}

//processes the distance travelled into odometry feedback to ROS:
void odom_calc_send(const rclc_support_t *support){

    static double prev_left_wheel_distance;
    static double prev_right_wheel_distance;
    static int32_t prev_left_encoder_count = 0;
    static int32_t prev_right_encoder_count = 0;

    // Initialize variables for wheel velocities, robot velocities, and pose
    static float left_wheel_velocity, right_wheel_velocity;
    static float linear_velocity, angular_velocity;
    static double x = 0, y = 0, theta = 0; // Position and orientation of the robot in the odometry frame

    // Get the current encoder counts and time
    int32_t curr_left_encoder_count = left_encoder_count;
    int32_t curr_right_encoder_count = right_encoder_count;
    int64_t curr_time_us = time_us_64();

    // Calculate the elapsed time since the last call (in seconds)
    //float elapsed_time = (curr_time_us - prev_time_us) / 1000000.0f;

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

    // calculate how long it's been since the last run (in seconds):
    float delta_time = (time_us_64() - last_odom_calc) * 1e-9;

    double delta_left_wheel_distance = left_wheel_distance - prev_left_wheel_distance;
    double delta_right_wheel_distance = right_wheel_distance - prev_right_wheel_distance;

    double delta_distance = (delta_right_wheel_distance + delta_left_wheel_distance) / 2.0;
    double delta_theta = (delta_right_wheel_distance - delta_left_wheel_distance) / wheel_base_distance;

    x += delta_distance * cos(theta + delta_theta / 2.0);
    y += delta_distance * sin(theta + delta_theta / 2.0);
    theta += delta_theta;

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
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = 0.0;
    //quaternion_from_yaw(theta, &odom_msg.pose.pose.orientation);
    odom_msg.twist.twist.linear.x = linear_velocity;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = angular_velocity;

    // Publish the odometry message
    rcl_publish(&odom, &odom_msg, NULL);
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time){
    //timer callback bits (for publishing outwards to ROS2)
    //odometry feedback:;
    odom_calc_send(&support);
}

void flasher(){
    static int64_t lastflash = 0;
    if (time_us_64() - lastflash > (1000*1000)){
        gpio_put(LED_PIN, !gpio_get(LED_PIN));
        lastflash = time_us_64();
    }
}

int main() {
    //stdio_init_all(); //we don't want feedback right now

    rmw_uros_set_custom_transport(
            true,
            NULL,
            pico_serial_transport_open,
            pico_serial_transport_close,
            pico_serial_transport_write,
            pico_serial_transport_read
    );

    // Initialize GPIO pins for encoders
    gpio_init(6);
    gpio_init(7);
    gpio_init(8);
    gpio_init(9);

    // Initialize GPIO pins for encoders
    gpio_set_dir(6, GPIO_IN);
    gpio_set_dir(7, GPIO_IN);
    gpio_set_dir(8, GPIO_IN);
    gpio_set_dir(9, GPIO_IN);

    // Set up interrupts for encoders
    gpio_set_irq_enabled_with_callback(6, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &left_encoder_isr);
    gpio_set_irq_enabled_with_callback(8, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &right_encoder_isr);


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
            "cmd_vel");

    //create our publisher to /odom
    rclc_publisher_init_default(
            &odom,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
            "odom"
            );

    // create timer,
    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    const unsigned int timer_timeout = 100;
    rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(timer_timeout), timer_callback);

    //create the executor that harvests the incoming messages and shoots them off to the callback:
    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    //rclc_support_init(&support, 0, NULL, &allocator);
    rclc_executor_init(&executor, &support.context, 2, &allocator);

    //timer elements for executor to publish data:
    unsigned int rcl_wait_timeout = 1000;   // in ms
    rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout));
    rclc_executor_add_timer(&executor, &timer);

    //add the subscription to the executor:
    rclc_executor_add_subscription(
            &executor,
            &cmd_vel_sub,
            &cmd_vel,
            &cmd_vel_callback,
            ON_NEW_DATA);

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
