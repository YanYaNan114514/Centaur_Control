#include "application.h"
#include "utils/logger.h"
#include <QMessageBox>

Application::Application(int &argc, char **argv)
    : QApplication(argc, argv)
{
    setApplicationName("CentaurControl");
    setApplicationVersion("1.0.0");
    setOrganizationName("CentaurRobotics");
}

Application::~Application()
{
    shutdown();
}

bool Application::initialize()
{
    try {
        // 创建ROS2接口
        ros2_interface_ = std::make_unique<ROS2Interface>();
        if (!ros2_interface_->initialize()) {
            Logger::error("Failed to initialize ROS2 interface");
            return false;
        }
        
        // 创建主窗口
        main_window_ = std::make_unique<MainWindow>(ros2_interface_.get());
        main_window_->show();
        
        Logger::info("Application initialized successfully");
        return true;
    }
    catch (const std::exception& e) {
        Logger::error("Exception during initialization: " + std::string(e.what()));
        QMessageBox::critical(nullptr, "Initialization Error", e.what());
        return false;
    }
}

void Application::shutdown()
{
    if (ros2_interface_) {
        ros2_interface_->shutdown();
        ros2_interface_.reset();
    }
    
    if (main_window_) {
        main_window_.reset();
    }
}