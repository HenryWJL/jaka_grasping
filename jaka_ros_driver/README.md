## ROS Wrapper for JAKA MiniCobo

This is the package for using JAKA MiniCobo with ROS.

## Usage Instructions

### Start the robot node

To start the robot node in ROS:
```bash
roslaunch jaka_ros_driver start.launch
```
This will stream all the robot sensors and effectors, which will publish on the appropriate ROS topics.

### Published Topics

After running the above command, the following topics will be available:

- /robot_driver/move_line
- /robot_driver/move_joint
- /robot_driver/move_jog
- /robot_driver/stop_move
- /robot_driver/tool_point
- /robot_driver/joint_states
- /robot_driver/robot_states
- /robot_driver/set_user_frame
- /robot_driver/set_tcp
- /robot_driver/teach_drag
- /robot_driver/set_payload
- /robot_driver/clear_err
- /robot_driver/set_collision

For more information, please see [here](https://github.com/HenryWJL/RGB-D_Camera_Based_Robotic_Grasping_Project/blob/main/Doc/jaka_driver_interface.pdf).
