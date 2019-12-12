# Project ACR

## Description

The ACR is a robot designed to detect and repair damage on wind turbine wings.
Using an arduino and the ROS middleware, the robot can currently:
- Drive around; controlled by keyboard input via a usb connection
- Stick to surfaces using an external vacuum pump

#### Notes
> The ACR was previously using two arduinos, one to control the direction and trigger the motors, the other to keep a constant speed for the robot when the arduino was under heavy load.

This has been changed to a single arduino to make sure we can tell the ACR how many steps to take, instead of telling it how long to enable the motors for.
The previous implementation of controlling the robot was made possible by a shield on top the robot, which is essentially a gate.

//TODO Write documentation on the shield

## Goals

- [ ] Replacing the Arduino by a Controllino
- [ ] On-board raspberry pi for wireless control
- [ ] Implementing SLAM for autonomous navigation [RUNNING] 

## SLAM for autonomous navigation
> This is the current plan of attack, changes will be made along the way.

#### Odometry
1. Modify ACR and arduino code so make the ACR step-controllable
2. Implement odometry; tell the robot how far to go, based on standard ROS message types
3. Use tf2 to locate ACR in wordspace

#### Mapping
3. Use lidar to map the environment

