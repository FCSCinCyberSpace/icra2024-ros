# ----- Under Construction -----

# ROS packages for Competitions of Future Convenience Store Challenge.

See also [wiki page](https://github.com/FCSCinCyberSpace/documents).


## Prerequisites

Same as below for OS and ROS version.  
https://github.com/FCSCinCyberSpace/documents/blob/master/SoftwareManual/Environment.md#ubuntu-pc

The recommended ROS package is ros-noetic-desktop-full.  
OpenCV is included and is required to run the sample programs.

## How to Install

### Install Rosbridge Server

Please see below.  
http://wiki.ros.org/rosbridge_suite

### Install ROS Packages For Competitions

```bash:
$ cd ~/catkin_ws/src
$ git clone https://github.com/FCSCinCyberSpace/icra2024-ros.git
$ cd ..
$ catkin_make
```

## How to Execute

### Run Sample ROS Node of Interactive Customer Service

This is a simple ROS node for the interactive customer service application.

```bash:
$ roslaunch interactive_customer_service sample.launch
```

There is also a keyboard-operated sample ROS node for debugging.

```bash:
$ roslaunch interactive_customer_service teleop_key.launch 
```

## License

This project is licensed under the SIGVerse License - see the LICENSE.txt file for details.
