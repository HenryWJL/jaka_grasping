# RGB-D Camera Based Robotic Grasping

These are the packages for performing visual grasp based on **Intel Realsense D455 camera**, **JAKA MiniCobo robot** and **DH-Robotic PGE-50-26 gripper**. For more information, please see [here](https://github.com/HenryWJL/RGB-D_Camera_Based_Robotic_Grasping_Project).

## Installation Instructions

### Step 1: Download Intel Realsense SDK and ROS package
- [librealsense](https://github.com/IntelRealSense/librealsense)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)

### Step 2: Download DH-gripper ROS package
- [dh_gripper_ros](https://github.com/DH-Robotics/dh_gripper_ros)

### Step 3: Download ArUco ROS package 
- [aruco_ros](https://github.com/pal-robotics/aruco_ros)

### Step 4: Download find-object-2d ROS package
- [find-object](https://github.com/introlab/find-object)

### Step 5: Clone this repository to your workspace
```bash
cd ~/Your Worksapce/src
git clone https://github.com/HenryWJL/jaka_grasping.git
```

### Step 6: Compile
```bash
cd ..
catkin_make
source devel/setup.bash
```

## Configuration


## Usage Instructions

### Step 1: Hand-eye calibration
- [handeye_calibration](https://github.com/HenryWJL/jaka_grasping/tree/main/handeye_calibration)

### Step 2: Start object detection node
There are two options for you. One is using ROS **find-object-2d** package to detect objects. This requires you to provide a template for the camera to identify. The other is using **ArUco** to detect objects. You need to paste an ArUco target on the object and modify the arguments' values in the relevant launch file. Moreover, before you run the following commands, make some modifications to the relevant launch file. See [here](https://github.com/HenryWJL/jaka_grasping/tree/main/visual_grasp).

- Using find-object-2d
```bash
roslaunch find_object_2d find_object_3d.launch
```
- Using ArUco
```bash
roslaunch visual_grasp object_detection_aruco.launch
```

### Step 3: Start object location node
```bash
rosrun  visual_grasp object_location.py
```

### Step 4: Enable DH-gripper
```bash
roslaunch dh_gripper_driver dh_gripper.launch
```

### Step 4: Start grasping
```bash
rosrun visual_grasp jaka_grasp.py
```
