#ifndef APPLICATION_H
#define APPLICATION_H

#include <QApplication>
#include <memory>
#include "communication/ros2_interface.h"
#include "ui/main_window.h"

class Application : public QApplication
{
    Q_OBJECT

public:
    Application(int &argc, char **argv);
    ~Application();
    
    bool initialize();
    void shutdown();
    
private:
    std::unique_ptr<ROS2Interface> ros2_interface_;
    std::unique_ptr<MainWindow> main_window_;
};

#endif // APPLICATION_H