# go2_ros2_toolbox

A (unofficial) toolbox for Unitree Go2 in SLAM and navigation (maybe more later)

<div align="center">
  <img src="asset/demo.gif" alt="Go2 ROS2 Toolbox Demo">
</div>

# Features

- LiDAR Message in ROS & Point Cloud Accumulation
- Camera Gstream Capture
- SLAM Toolbox
- Navigation2

# Installation

```bash
mkdir -p go2_ros2_ws/src
cd go2_ros2_ws/src
git clone https://github.com/andy-zhuo-02/go2_ros2_toolbox.git
cd ..
colcon build
```

# Usage

```bash
source install/setup.bash
ros2 launch go2_core go2_startup.launch.py
```
