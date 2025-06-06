#include <QApplication>
#include <rclcpp/rclcpp.hpp>
#include "core/application.h"
#include "utils/logger.h"

int main(int argc, char *argv[])
{
    // 初始化ROS2
    rclcpp::init(argc, argv);
    
    // 创建Qt应用程序
    Application app(argc, argv);
    
    // 初始化日志系统
    Logger::initialize();
    
    // 初始化应用程序
    if (!app.initialize()) {
        Logger::error("Failed to initialize application");
        return -1;
    }
    
    Logger::info("CentaurControl application started");
    
    // 运行应用程序
    int result = app.exec();
    
    // 清理
    app.shutdown();
    rclcpp::shutdown();
    
    Logger::info("CentaurControl application terminated");
    return result;
}