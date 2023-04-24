## RGB-D Camera Based Robotic Grasping

These are the packages for performing visual grasp based on JAKA MiniCobo and Intel Realsense D455 camera. For more information, please see [here](https://github.com/HenryWJL/RGB-D_Camera_Based_Robotic_Grasping_Project).

## Installation Instructions

- Step 1: Download **ArUco** 
```bash
cd ~/Your Workspace/src
git clone -b melodic-devel https://github.com/pal-robotics/aruco_ros.git
```

- Step 2: Download **find-object-2d** package
```bash
git clone https://github.com/introlab/find-object.git src/find_object_2d
```

- Step 3: Clone this repository to your workspace
```bash
git clone https://github.com/HenryWJL/jaka_grasping.git
```

- Step 4: Run **catkin_make**
```bash
cd ..
catkin_make
```

- Step 5: Source **setup.bash**
```bash
source devel/setup.bash
```
