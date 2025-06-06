#include "ros2_interface.h"
#include "utils/logger.h"

ROS2Interface::ROS2Interface(QObject *parent)
    : QObject(parent), rclcpp::Node("centaur_control_node")
{
    spin_timer_ = new QTimer(this);
    connect(spin_timer_, &QTimer::timeout, this, &ROS2Interface::spinOnce);
}

ROS2Interface::~ROS2Interface()
{
    shutdown();
}

bool ROS2Interface::initialize()
{
    try {
        setupPublishers();
        setupSubscribers();
        
        // 启动定时器，定期处理ROS2消息
        spin_timer_->start(10); // 100Hz
        
        Logger::info("ROS2 interface initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        Logger::error("Failed to initialize ROS2 interface: " + std::string(e.what()));
        return false;
    }
}

void ROS2Interface::shutdown()
{
    if (spin_timer_) {
        spin_timer_->stop();
    }
    
    chassis_cmd_pub_.reset();
    camera_sub_.reset();
    lidar_sub_.reset();
    map_sub_.reset();
}

void ROS2Interface::setupPublishers()
{
    chassis_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10);
}

void ROS2Interface::setupSubscribers()
{
    camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            emit imageReceived(msg);
        });
    
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/scan_cloud", 10,
        [this](const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            emit pointCloudReceived(msg);
        });
    
    map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/map", 10,
        [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
            emit mapReceived(msg);
        });
}

void ROS2Interface::publishChassisVelocity(double linear_x, double linear_y, double angular_z)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.linear.y = linear_y;
    msg.angular.z = angular_z;
    
    if (chassis_cmd_pub_) {
        chassis_cmd_pub_->publish(msg);
    }
}

void ROS2Interface::publishArmCommand(const std::string& arm_id, const std::vector<double>& joint_positions)
{
    // TODO: 实现机械臂控制命令发布
    Logger::info("Publishing arm command for: " + arm_id);
}

void ROS2Interface::spinOnce()
{
    rclcpp::spin_some(this->get_node_base_interface());
}