# JAKA MiniCobo Based Robotic Grasping

These are the packages for performing visual grasp based on **Intel Realsense D455 camera**, **JAKA MiniCobo robot** and **DH-Robotic PGE-50-26 gripper**. To fulfill the functions of this package, you need to install some extra packages and make some modifications to the launch files. Follow the instructions below to complete the grasping task. For more information, please see [here](https://github.com/HenryWJL/RGB-D_Camera_Based_Robotic_Grasping_Project).

## Installation Instructions

### Step 1: Download Intel Realsense SDK and ROS package
- [librealsense](https://github.com/IntelRealSense/librealsense)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)

### Step 2: Download JAKA MiniCobo ROS package
- [jaka_ros](https://github.com/JAKARobotics/JAKA_ROS_Driver)

### Step 3: Download DH-gripper ROS package
- [dh_gripper_ros](https://github.com/DH-Robotics/dh_gripper_ros)

### Step 4: Download ArUco ROS package 
- [aruco_ros](https://github.com/pal-robotics/aruco_ros)

### Step 5: Download Apriltag ROS package
- [apriltag_ros](https://github.com/AprilRobotics/apriltag_ros)

### Step 6: Clone this repository to your workspace
```bash
cd ~/Your Worksapce/src
git clone https://github.com/HenryWJL/jaka_grasping.git
```

### Step 7: Compile
```bash
cd ..
catkin_make
source devel/setup.bash
```
If your workspace contains Apriltag packages, run:
```bash
catkin_make_isolated
source devel_isolated/setup.bash
```

## Modification

#### (1) Modify the following arguments in the `/launch/gripper_init.launch`:
```launch
<launch>
	
    <arg name="GripperID"    default="1"/>
    <arg name="GripperModel" default="PGE"/>
    <arg name="Connectport"  default="/dev/ttyUSB0"/>
    <arg name="Baudrate"     default="115200"/>
    <arg name="test_run"     default="false"/>
    ...
</launch>
```
#### (2) Modify the following arguments in the `/launch/object_detection_aruco.launch`: 
```launch
<launch>

    <arg name="image_topic"     default="/camera/color/image_raw"/>
    <arg name="camera_info"     default="/camera/color/camera_info"/> 
    <arg name="markerId"        default="250"/>
    <arg name="markerSize"      default="0.4"/>
    ...
</launch>
```
#### (3) Modify the following arguments in the `/launch/object_detection_apriltag.launch`:
```launch
<launch>

    <arg name="image_topic"       default="/camera/color/image_raw"/>
    <arg name="camera_info"       default="/camera/color/camera_info"/>
    <arg name="publish_tag_image" default="true" />
    <arg name="queue_size"        default="1" />
    <arg name="launch_prefix"     default="" />
    ...
</launch>
```
#### (4) Modify the parameters in the `/config/tags.yaml` and `/config/settings.yaml`:
```yaml
standalone_tags:
  [
     {id: 250, size: 0.0215}
  ]
```
```yaml
tag_family:        'tagStandard41h12' 
tag_threads:       2          
tag_decimate:      1.0        
tag_blur:          0.0        
tag_refine_edges:  1          
tag_debug:         0          
max_hamming_dist:  2          
publish_tf:        true       
transport_hint:    "raw"
```

## Usage Instructions

### Step 1: Hand-eye calibration
- [handeye_calibration](https://github.com/HenryWJL/hand_eye_calibration)

### Step 2: Enable gripper
```bash
roslaunch jaka_grasping gripper_init.launch
```
### Step 3: Start object detection node
There are two options for you. One is using **AprilTag**, the other is using **ArUco**. You need to paste an AprilTag marker or an ArUco marker on the object and modify the arguments in the relevant launch file (see **Modification**).

- Using AprilTag
```bash
roslaunch jaka_grasping object_detection_apriltag.launch
```
- Using ArUco
```bash
roslaunch jaka_grasping object_detection_aruco.launch
```
### Step 4: Start object location node
```bash
rosrun jaka_grasping object_location.py
```
### Step 5: Start grasping
```bash
rosrun jaka_grasping jaka_grasp.py
```
