# Project ACR

## Description

The ACR is a robot designed to detect and repair damage on wind turbine wings.
Using an arduino and the ROS middleware, the robot can currently:
- Drive around; controlled by keyboard input via a usb connection
- Stick to surfaces using an external vacuum pump

## Current Goals

- [ ] Replacing the Arduino by a Controllino
- [ ] On-board raspberry pi for wireless control
- [ ] Implementing SLAM for autonomous navigation [RUNNING] 

## SLAM for autonomous navigation
> This is the current plan of attack, changes will be made along the way. This is a WIP

#### Odometry
1. Use tf2 to locate ACR in wordspace
2. Implement odometry, and send metric commands

#### Mapping
3. Use lidar to map the environment

