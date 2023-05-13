# Hand-Eye Calibration

This package is used for hand-eye calibration. **Intel Realsense D455** camera and **JAKA MiniCobo** robot are required.

## Usage Instruction

#### Step 1: Print an ArUco target
- [ArUco](https://chev.me/arucogen/)
- **Attention**: When you enter the download interface, please choose 'Original ArUco' dictionary. 

#### Step 2: Modify the values of the following arguments in the `/handeye_calibration/launch/aruco_start.launch`
```launch
<launch>

    <arg name="image_topic"     default="/camera/color/image_raw"/>
    <arg name="camera_info"     default="/camera/color/camera_info"/> 
    <arg name="markerId"        default="250"/>
    <arg name="markerSize"      default="0.04"/>
    ...
</launch>
```

#### Step 3: Start the camera node
```bash
roslaunch realsense2_camera rs_camera.launch
```
#### Step 4: Start the pose estimation node
- ArUco
```bash
roslaunch handeye_calibration aruco_start.launch
```
- AprilTag
```bash
roslaunch handeye_calibration apriltag_start.launch
```
#### Step 5: Start the robot node
```bash
roslaunch handeye_calibration robot_start.launch
```
#### Step 6: Start the calibration node. 
```bash
rosrun handeye_calibration hand_to_eye_calib.py
```
When everything is prepared, move the calibration target and type 'r' in the terminal to record calibration data. After recording more than two data, type 'c' in the terminal to calculate the calibration result. After a few seconds, you can view the result on the screen. If you want to save the result, type 's' in the terminal. This will save the result as a **.yaml** file.
