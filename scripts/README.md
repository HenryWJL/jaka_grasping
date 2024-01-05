# JAKA Panel Control
Most of the codes in this package were not written by me, but instead, downloaded from the [JAKA website](https://www.jaka.com/index).
My job was to modify the original codes and add some new codes in order to achieve the functionality of panel control.

# Usage
1. Move this package to your workspace and run:
```bash
catkin_make
source devel/setup.bash
```
2. Connect to the robot's wifi.
3. Enable the robot.
```bash
roslaunch jaka_ros_driver start.launch
```
4. Open the panel and start to have fun!
```bash
rosrun jaka_jog_panel jakajogpanel
```
