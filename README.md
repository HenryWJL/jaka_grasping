# RGB-D Camera Based Robotic Grasping

These are the packages for performing visual grasp based on JAKA MiniCobo and Intel Realsense D455 camera. For more information, please see [here](https://github.com/HenryWJL/RGB-D_Camera_Based_Robotic_Grasping_Project).

## Installation Instructions

### Step 1: Download Intel Realsense SDK and ROS package
- [librealsense](https://github.com/IntelRealSense/librealsense)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy)

### Step 2: Download ArUco ROS package 
- [aruco_ros](https://github.com/pal-robotics/aruco_ros)

### Step 3: Download find-object-2d ROS package
- [find-object](https://github.com/introlab/find-object)

### Step 4: Clone this repository to your workspace
```bash
cd ~/Your Worksapce/src
git clone https://github.com/HenryWJL/jaka_grasping.git
```

### Step 5: 
```bash
cd ..
catkin_make
source devel/setup.bash
```

## Usage Instructions

### Step 1: Hand-eye calibration
- [handeye_calibration](https://github.com/HenryWJL/jaka_grasping/tree/main/handeye_calibration)

### Step 2: 

```bash
roslaunch visual_grasp start.launch
```
