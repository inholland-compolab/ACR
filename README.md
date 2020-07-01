# Project ACR

## Description

The ACR is a robot designed to detect and repair damage on wind turbine wings.
Using an arduino and the ROS middleware, the robot can currently:
- Drive around; controlled by keyboard input via a usb connection
- Stick to surfaces using an external vacuum pump

#### Hardware

- **Lidar:** Leddar Vu8
- **Controller:** Arduino ATMega2560 with custom shield (see docs folder for documentation on the shield)
- **Stepper Drivers:** TB6600 (4x)
- **Stepper Motors:** NEMA17 (4x)

#### Installation

For installing the required to software to run the ACR, follow these steps:

1. Follow the instructions on http://wiki.ros.org/melodic/Installation/Ubuntu to install ROS. Like they say: use Ubuntu, it's they easiest to setup, and since this project was developed on Ubuntu, the most recommended when using this project.

2. For installing everything needed concerning the Lidar, you'll need to head to the LeddarSDK github page https://github.com/leddartech/LeddarSDK. 

3. Upload code to the arduino. The source code is located in `rosssystem/src/acr_control/src`.

#### Running

For running the robot, follow these steps:

1. Open a terminal and enter `roscore`

2. Plug in the arduino. Open a terminal and enter `rosrun rosserial_python serial_node.py /dev/ttyACM0`. The last argument refers to the USB port the arduino is connected to. Run `ls /dev/tty*` in the terminal to see all available devices (it should be something like `ttyUSB0` or `ttyACM0`. If you see an error after running the `rosserial` command, it's probably because you got the wrong port)

3. Open a terminal and enter `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

4. Control the robot using the `i`, `j`, `k`, `l`, `m`, `,` and `.` keys. Keep the terminal with teleop_twist focused, as it will only listen to the keyboard when it is focused

#### Notes

As of the latest version in dev, an issue can occur when launching the example launch file in the `leddar_ros` package.
Please look at the issue board if you have any problems.

## Goals

- [ ] Replacing the Arduino by a Controllino
- [ ] On-board raspberry pi for wireless control
- [ ] Implementing SLAM for autonomous navigation [RUNNING] 

Further info on goals can be found in the 'Projects' page
