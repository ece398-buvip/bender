#!/bin/bash

echo "About to run... GET TO DA ESTOP"

sleep 3

#ros2 topic pub -t 1 /bender/right_duty std_msgs/msg/UInt32 "data: 32"
ros2 topic pub -t 1 /bender/left_duty std_msgs/msg/UInt32 "data: 40"

sleep 5

#ros2 topic pub -t 1 /bender/right_duty std_msgs/msg/UInt32 "data: 0"
ros2 topic pub -t 1 /bender/left_duty std_msgs/msg/UInt32 "data: 0"
