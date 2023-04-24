## RGB-D Camera Based Robotic Grasping

These are the packages for performing visual grasp based on JAKA MiniCobo and Intel Realsense D455 camera. For more information, please see [here](https://github.com/HenryWJL/RGB-D_Camera_Based_Robotic_Grasping_Project).

## Installation Instructions

### Step 1: Download Intel Realsense SDK and ROS package
- [librealsense](https://github.com/IntelRealSense/librealsense)
- [realsense-ros](https://github.com/IntelRealSense/realsense-ros)

### Step 2: Download ArUco ROS package 
- [aruco_ros](https://github.com/pal-robotics/aruco_ros)

### Step 3: Download find-object-2d ROS package
- [find-object](https://github.com/introlab/find-object)

### Step 4: Clone this repository to your workspace
```bash
git clone https://github.com/HenryWJL/jaka_grasping.git
```

### Step 5: Run **catkin_make**
```bash
cd ..
catkin_make
```

### Step 6: Source **setup.bash**
```bash
source devel/setup.bash
```
