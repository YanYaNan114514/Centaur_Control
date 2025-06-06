cmake_minimum_required(VERSION 3.16)
project(CentaurControl VERSION 1.0.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Conan集成
include(${CMAKE_BINARY_DIR}/conan_toolchain.cmake OPTIONAL)
include(${CMAKE_BINARY_DIR}/conan_deps.cmake OPTIONAL)

# 查找ROS2依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Qt相关
find_package(Qt6 REQUIRED COMPONENTS Core Widgets OpenGL OpenGLWidgets)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Conan依赖
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(nlohmann_json REQUIRED)

# 包含目录
include_directories(
  src
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

# 源文件
set(SOURCES
  src/main.cpp
  src/core/application.cpp
  src/ui/main_window.cpp
  src/ui/widgets/robot_status_widget.cpp
  src/ui/widgets/camera_display_widget.cpp
  src/ui/widgets/slam_visualization_widget.cpp
  src/ui/widgets/arm_control_widget.cpp
  src/ui/dialogs/settings_dialog.cpp
  src/communication/ros2_interface.cpp
  src/communication/chassis_controller.cpp
  src/communication/arm_controller.cpp
  src/sensors/camera_manager.cpp
  src/sensors/lidar_processor.cpp
  src/algorithms/slam_processor.cpp
  src/algorithms/deep_learning/inference_engine.cpp
  src/utils/logger.cpp
)

# UI文件
set(UI_FILES
  src/ui/main_window.ui
)

# 创建可执行文件
add_executable(${PROJECT_NAME}
  ${SOURCES}
  ${UI_FILES}
)

# 链接库
target_link_libraries(${PROJECT_NAME}
  Qt6::Core
  Qt6::Widgets
  Qt6::OpenGL
  Qt6::OpenGLWidgets
  opencv::opencv
  Boost::boost
  Eigen3::Eigen
  fmt::fmt
  nlohmann_json::nlohmann_json
)

# ROS2依赖
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  cv_bridge
  image_transport
)

# 安装
install(TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  OPTIONAL
)

ament_package()