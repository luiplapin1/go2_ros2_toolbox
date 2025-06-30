# Go2 ROS2 Toolbox

[![ROS2](https://img.shields.io/badge/ROS2-Foxy-green.svg)](https://docs.ros.org/en/foxy/)
[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Ubuntu%2020.04-orange.svg)](https://ubuntu.com/)

[🇨🇳 中文版 README](./README_zh.md)

A comprehensive ROS2 toolbox for Unitree Go2 EDU robot, providing SLAM and navigation capabilities for autonomous operation.

<div align="center">
  <img src="asset/demo.gif" alt="Go2 ROS2 Toolbox Demo" width="100%" style="background-color: #1a1a1a;">
</div>

---

## 🚀 Features

- **LiDAR Integration**: Real-time point cloud processing and accumulation
- **Camera Support**: GStreamer-based camera capture and streaming
- **SLAM Capabilities**: Integration with SLAM Toolbox for mapping
- **Navigation Stack**: Full Navigation2 integration for autonomous navigation
- **ROS2 Native**: Built specifically for ROS2 Foxy ecosystem

---

## 📋 Prerequisites

> **⚠️ Note:** The features of this repository have only been tested on the onboard expansion dock computer of Go2 EDU. Compatibility and functionality on other environments (like PC wired to Go2 dock computer) have not been verified.

This toolbox is developed and tested on Unitree Go2 EDU with the expansion dock environment:

- **OS:** Ubuntu 20.04
- **ROS2:** Foxy
- **Firmware:** v1.1.7 (tested)

---

## 🛠️ Installation

### 1. Install Official Unitree ROS2 Package

Follow the official installation guide:

```bash
# https://github.com/unitreerobotics/unitree_ros2
```

### 2. Install Dependencies

If you are using Ubuntu 20.04 ARM64, you must follow these steps carefully to build all components successfully:

#### System Dependencies

```bash
sudo apt update
sudo apt install -y \
  google-mock \
  libatlas-base-dev \
  libsuitesparse-dev \
  libceres-dev \
  protobuf-compiler \
  libprotobuf-dev \
  liblua5.3-dev \
  libboost-iostreams-dev \
  libboost-program-options-dev \
  libboost-regex-dev \
  libeigen3-dev \
  libgflags-dev \
  libgoogle-glog-dev \
  libpqxx-dev
```

#### Python Dependencies

```bash
pip3 install transforms3d
```

### 3. Install Abseil C++ (C++17)

Cartographer requires Abseil C++ compiled with C++17:

```bash
git clone https://github.com/abseil/abseil-cpp.git ~/abseil-cpp
cd ~/abseil-cpp
rm -rf build
mkdir build
cd build
cmake -DCMAKE_CXX_STANDARD=17 -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make -j$(nproc)
sudo make install
```

### 4. Build Cartographer

```bash
mkdir -p ~/cartographer_ws/src
cd ~/cartographer_ws/src
git clone https://github.com/cartographer-project/cartographer.git
```

Remove thread annotations (required with modern Abseil):

```bash
cd ~/cartographer_ws/src/cartographer
find . -type f \( -name "*.h" -o -name "*.cc" \) -exec sed -i 's/ LOCKS_EXCLUDED([^)]*)//g' {} +
find . -type f \( -name "*.h" -o -name "*.cc" \) -exec sed -i 's/ GUARDED_BY([^)]*)//g' {} +
find . -type f \( -name "*.h" -o -name "*.cc" \) -exec sed -i 's/ EXCLUSIVE_LOCKS_REQUIRED([^)]*)//g' {} +
```

Build Cartographer:

```bash
cd ~/cartographer_ws
colcon build --symlink-install
```

Source the workspace:

```bash
source ~/cartographer_ws/install/setup.bash
```

### 5. Build slam_toolbox

```bash
mkdir -p ~/go2_ros2_ws/src
cd ~/go2_ros2_ws/src
git clone https://github.com/SteveMacenski/slam_toolbox.git -b foxy
```

Build:

```bash
cd ~/go2_ros2_ws
colcon build
```

### 6. Install Navigation2 and nav2_bringup

```bash
sudo apt update
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
```

Verify the launch files:

```bash
ls /opt/ros/foxy/share/nav2_bringup/launch/
```

### 7. Build Go2 ROS2 Toolbox

```bash
cd ~/go2_ros2_ws/src
git clone https://github.com/andy-zhuo-02/go2_ros2_toolbox.git
cd ~/go2_ros2_ws
colcon build
```

### 8. Environment Setup

Before launching anything, always source these workspaces in this order:

```bash
source /opt/ros/foxy/setup.bash
source ~/cartographer_ws/install/setup.bash
source ~/go2_ros2_ws/install/setup.bash
```

Add to ~/.bashrc to load automatically:

```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source ~/cartographer_ws/install/setup.bash" >> ~/.bashrc
echo "source ~/go2_ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## 🎯 Usage

### Launch the Go2 Node

```bash
ros2 launch go2_core go2_startup.launch.py
```

### Launch Navigation2

```bash
ros2 launch nav2_bringup bringup_launch.py
```

### Launch SLAM Toolbox

For example:

```bash
ros2 launch slam_toolbox online_async_launch.py
```

### RViz

Open RViz2:

```bash
rviz2
```

Add displays for:
- `/map`
- `/odom`
- `/tf`
- `/scan`

---

## ✅ Verification

After sourcing, verify that packages are visible:

```bash
ros2 pkg list | grep slam_toolbox
ros2 pkg list | grep nav2_bringup
```

Both must appear.

---

## 🔧 Development

### Frame Reference

| Frame | Description | Source |
|-------|-------------|---------|
| `/odom` | Odometry frame | Unitree Go2 odometry topic |
| `/map` | Map frame | SLAM Toolbox |
| `/base_link` | Base link frame | Unitree Go2 odometry topic |

---

## 🙏 Acknowledgments

- Original repository author: andy-zhuo-02
- Unitree Robotics for the Go2 EDU platform
- ROS2 community for Navigation2 and SLAM Toolbox

This README includes additional instructions tested on Ubuntu 20.04 ARM64.

**Note:** This is an unofficial fork and is not affiliated with Unitree Robotics.
