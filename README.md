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

### Step 6: Download find-object-2d ROS package (option)
- [find-object](https://github.com/introlab/find-object)

### Step 7: Clone this repository to your workspace
```bash
cd ~/Your Worksapce/src
git clone https://github.com/HenryWJL/jaka_grasping.git
```

### Step 8: Compile
```bash
cd ..
catkin_make
source devel/setup.bash
```

## Modification

#### (1) Modify the values of the following arguments in the `/visual_grasp/launch/robot_init.launch`:
```launch
<launch>
	
    <arg name="GripperID"    default="1"/>
    <arg name="GripperModel" default="PGE"/>
    <arg name="Connectport"  default="/dev/ttyUSB0"/>
    <arg name="Baudrate"     default="115200"/>
    <arg name="test_run"     default="false"/>
	
    <arg name="robot_ip"     default="192.168.200.100"/>
    ...
</launch>
 ```
#### (2) Modify the values of `markerId` and `markerSize` in the `/visual_grasp/launch/object_detection_aruco.launch`: 
```launch
<launch>
     
    <arg name="markerId"        default="250"/>
    <arg name="markerSize"      default="0.4"/>
    ...
</launch>
```
#### (3) Modify the values of ... in the apriltag
```launch
```

#### (4) Modify the `/find_object_2d/launch/ros1/find_object_3d.launch` like this (option):
```launch
<launch>
	<!-- Example finding 3D poses of the objects detected -->
	<!-- $roslaunch openni_launch openni.launch depth_registration:=true -->
	
	<arg name="object_prefix"     default="object"/>
	<arg name="objects_path"      default=""/>
	<arg name="gui"               default="true"/>
	<arg name="approx_sync"       default="true"/>
	<arg name="pnp"               default="true"/>
	<arg name="tf_example"        default="true"/>
	<arg name="settings_path"     default="~/.ros/find_object_2d.ini"/>
	<arg name="target_frame_id"   default="/camera_link"/>
	
	<arg name="rgb_topic"         default="/camera/color/image_raw"/>
        <arg name="depth_topic"       default="/camera/depth/image_rect_raw"/>
        <arg name="camera_info_topic" default="/camera/color/camera_info"/>
	
	<node name="find_object_3d" pkg="find_object_2d" type="find_object_2d" output="screen">
		<param name="gui" value="$(arg gui)" type="bool"/>
		<param name="settings_path" value="$(arg settings_path)" type="str"/>
		<param name="subscribe_depth" value="true" type="bool"/>
		<param name="objects_path" value="$(arg objects_path)" type="str"/>
		<param name="object_prefix" value="$(arg object_prefix)" type="str"/>
		<param name="approx_sync" value="$(arg approx_sync)" type="bool"/>
		<param name="pnp" value="$(arg pnp)" type="bool"/>
		
		<remap from="rgb/image_rect_color" to="$(arg rgb_topic)"/>
		<remap from="depth_registered/image_raw" to="$(arg depth_topic)"/>
		<remap from="depth_registered/camera_info" to="$(arg camera_info_topic)"/>
	</node>
	
	<!-- Example of tf synchronisation with the objectsStamped message -->
	<node if="$(arg tf_example)" name="tf_example" pkg="find_object_2d" type="tf_example" output="screen">
		<param name="object_prefix" value="$(arg object_prefix)" type="str"/>
		<param name="target_frame_id" value="$(arg target_frame_id)" type="str"/>
	</node>
</launch>
```

## Usage Instructions

### Step 1: Hand-eye calibration
- [handeye_calibration](https://github.com/HenryWJL/jaka_grasping/tree/main/handeye_calibration)

### Step 2: Enable robot and gripper
```bash
roslaunch visual_grasp robot_init.launch
```
### Step 3: Start object detection node
There are two options for you. One is using ROS `find-object-2d` package to detect objects. This requires you to provide a template for the camera to identify. The other is using **ArUco** to detect objects. You need to paste an ArUco target on the object and modify the arguments' values in the relevant launch file (see **Modification**).

- Using find-object-2d
```bash
roslaunch find_object_2d find_object_3d.launch
```
- Using ArUco
```bash
roslaunch visual_grasp object_detection_aruco.launch
```
### Step 4: Start object location node
```bash
rosrun visual_grasp object_location.py
```
### Step 5: Start grasping
```bash
rosrun visual_grasp jaka_grasp.py
```
