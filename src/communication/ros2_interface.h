#ifndef ROS2_INTERFACE_H
#define ROS2_INTERFACE_H

#include <QObject>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

class ROS2Interface : public QObject, public rclcpp::Node
{
    Q_OBJECT

public:
    explicit ROS2Interface(QObject *parent = nullptr);
    ~ROS2Interface();
    
    bool initialize();
    void shutdown();
    
    // 底盘控制
    void publishChassisVelocity(double linear_x, double linear_y, double angular_z);
    
    // 机械臂控制
    void publishArmCommand(const std::string& arm_id, const std::vector<double>& joint_positions);
    
signals:
    void imageReceived(const sensor_msgs::msg::Image::SharedPtr msg);
    void pointCloudReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void mapReceived(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void robotStatusUpdated(const QString& status);
    
private slots:
    void spinOnce();
    
private:
    // 发布者
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr chassis_cmd_pub_;
    
    // 订阅者
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    
    QTimer* spin_timer_;
    
    void setupPublishers();
    void setupSubscribers();
};

#endif // ROS2_INTERFACE_H