## ROS Wrapper for JAKA MiniCobo

This is the package for using JAKA MiniCobo with ROS.

## Usage Instructions

### Start the robot node

To start the robot node in ROS:
```bash
roslaunch jaka_ros_driver start.launch
```
This will stream all the robot sensors and effectors, which will publish on the appropriate ROS topics.
