# micro-ros_basic_2wheel_framework
Sketchpad for work on getting micro-ros to play ball using a Raspberry Pi Pico.


requires the full micro-ros sdk for raspberry pi pico to compile

Just chucking this quickly here as a backup after reaching a milestone of basic functionality

MESSY MESSY MESSY (fast flow messy code here on in)

(some reused code from the micro-ros for pico example)

TODO:
- re-enable the cutoff when /cmd_vel hasn't been published to in a short while (in place, but disabled for testing)
- add parameter changing of PID controls for closed loop speed control via ros2 param subscription/callback
- storage of PID params in flash with logic to only write after params have been changed (to not burn it out)
- add to params ability to change encoder direction, wheel diameter, wheel width, encoder points per rotation
- add ability for output rate to change to /odom (destroy executor, create executor) when not being commanded via /cmd_vel for a certain period of time (to save processing power/heat on computer)
