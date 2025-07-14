# msteer_duty.py

This script publishes a predefined sequence of motor duty cycle and servo steering commands to two ROS 2 topics:  
- `/commands/motor/duty_cycle`  
- `/commands/servo/position`  

Each pair of motor and servo values is held for 3 seconds before moving to the next in the sequence. The node operates at 100 Hz using a timer callback and shuts down automatically once the full sequence is published.  
It is useful for simple open-loop test runs with synchronized motor and steering inputs.
