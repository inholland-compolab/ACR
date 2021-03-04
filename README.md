# Project ACR

## Description

The ACR is a robot designed to detect and repair damage on wind turbine wings.
Using an arduino and the ROS middleware, the robot can currently:
- Drive around; controlled by keyboard input via a usb connection
- Stick to surfaces using an external vacuum pump

### Hardware

- **Lidar:** Leddar Vu8
- **Controller:** Arduino ATMega2560 with custom shield (see docs folder for documentation on the shield)
- **Stepper Drivers:** TB6600 (4x)
- **Stepper Motors:** NEMA17 (4x)


## Running and Installation

For running the robot, there are two options:

1. In a docker container
3. On a virtual machine or normal PC with Ubuntu 18.04 (not anything higher, or lower)

I highly recommend doing it in a docker container, especially if you don't have experience with any Linux-related stuff.

### In a docker container

#### Prerequisites

1. Make sure you have docker [installed](https://docs.docker.com/get-docker/) your computer.

2. Run `build.sh` as Admin or using the `sudo` command in Linux.

#### Running

1. Run `run.sh` as Admin or using the `sudo` command in Linux. It will then say: I need more parameters! To which you respond with the appropriate parameters. They're the virtual device files corresponding to the lidar, and arduino. Often one of the following:
```
 - Windows: COM[0-9]

 - Linux: /dev/ACM[0-9] or /dev/USB[0-9]
```

2. Control the robot using the `i`, `j`, `k`, `l`, `m`, `,` and `.` keys. Keep the terminal with teleop\_twist focused, as it will only listen to the keyboard when it is focused

3. That was it. That's why you should use docker.

>  Make sure the robot is not in full speed !

### On a (virtual) machine with Ubuntu 18.04:

#### Prerequisites

1. Follow the [following instructions](http://wiki.ros.org/melodic/Installation/Ubuntu) to install ROS. 

2. For installing everything needed concerning the Lidar, you'll need to head to the [LeddarSDK github page](https://github.com/leddartech/LeddarSDK)

3. Upload code to the arduino. The source code is located in `rosssystem/src/acr_control/src`.

4. Build everything using catkin\_make

#### Running

1. You can simply launch the project using `roslaunch acr_setup acr_setup.launch`. If you get any errors saying it can't connect on the given usb ports, you should change the parameters to match the ones on you machine. These are often the following:

```
 - Windows: COM[0-9]

 - Linux: /dev/ACM[0-9] or /dev/USB[0-9]
```

>  Make sure the robot is not in full speed !

### Calibrating the Robot

There is no dedicated odometry device (like a potmeter to check the angle of the wheels), so current the speed of the robot is matched by the rviz simulation using a calibration factor in the arduino code. This code, and instructions can be found in `arduino_src/arduino_src.ino`.

## Notes

> I've run into issues with the leddar sdk a while ago (early 2020), and have seen that they since then have updated their repo and guides. You might want to check out a new version of their sdk.

> The project is build on ROS Melodic. The reason I've not migrated to Noetic is that the rosserial\_python package hasn't been updated accordingly. You might want to keep an eye out for this.

## Ideas

- [ ] Replacing the Arduino by a Controllino
- [ ] On-board raspberry pi for wireless control
- [ ] Implementing SLAM for autonomous navigation [RUNNING] 

Further info on goals can be found in the 'Projects' page
