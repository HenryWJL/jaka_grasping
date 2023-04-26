# Hand-Eye Calibration

This package is used for hand-eye calibration. Intel Realsense D455 camera and JAKA MiniCobo robot are required.

## Usage Instruction

### Step 1: Print an ArUco target
- [ArUco](https://chev.me/arucogen/)

### Step 2: Start the camera node
```bash
roslaunch realsense2_camera rs_camera.launch
```
### Step 3: Start the robot node
```bash
roslaunch jaka_ros_driver start.launch
```
### Step 4: Start the calibration node. 
```bash
rosrun handeye_calibration hand_to_eye_calib.py
```
When everything is prepared, move the calibration target and type 'r' in the terminal to record calibration data. After recording more than one data, type 'c' in the terminal to calculate the calibration result. After a few seconds, you can view the result on the screen. If you want to save the result, type 's' in the terminal. This will save the result as a **.yaml** file.
