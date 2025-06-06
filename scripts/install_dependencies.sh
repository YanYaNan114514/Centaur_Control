#!/bin/bash

# 更新包列表
sudo apt update

# 安装Qt6
sudo apt install -y qt6-base-dev qt6-tools-dev qt6-tools-dev-tools
sudo apt install -y qt6-base-dev-tools qt6-qpa-plugins
sudo apt install -y libqt6opengl6-dev libqt6openglwidgets6

# 安装OpenCV
sudo apt install -y libopencv-dev python3-opencv

# 安装PCL
sudo apt install -y libpcl-dev

# 安装Eigen3
sudo apt install -y libeigen3-dev

# 安装ROS2 Humble (如果还没安装)
sudo apt install -y ros-humble-desktop
sudo apt install -y ros-humble-cv-bridge
sudo apt install -y ros-humble-image-transport
sudo apt install -y ros-humble-tf2
sudo apt install -y ros-humble-tf2-ros

# 安装深度学习相关库
sudo apt install -y libonnx-dev  # ONNX Runtime
# 或者安装TensorFlow C++ API
# sudo apt install -y libtensorflow-dev

echo "Dependencies installation completed!"