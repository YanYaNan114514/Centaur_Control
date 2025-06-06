#!/bin/bash

# 设置ROS2环境
source /opt/ros/humble/setup.bash

# 安装Conan依赖
echo "Installing Conan dependencies..."
conan install . --output-folder=build --build=missing --profile=default

# 创建工作空间（如果不存在）
if [ ! -d "../ws" ]; then
    mkdir -p ../ws/src
    cd ../ws
    ln -s ../CentaurControl src/centaur_control
else
    cd ../ws
fi

# 构建项目
echo "Building project..."
colcon build --packages-select centaur_control --cmake-args -DCMAKE_TOOLCHAIN_FILE=../CentaurControl/build/conan_toolchain.cmake

# 设置环境
source install/setup.bash

echo "Build completed! Run with: ros2 run centaur_control CentaurControl"