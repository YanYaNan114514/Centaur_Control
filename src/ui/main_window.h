#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QGridLayout>
#include <QSplitter>
#include <QTabWidget>
#include <QStatusBar>
#include <QMenuBar>
#include <QToolBar>

#include "widgets/robot_status_widget.h"
#include "widgets/camera_display_widget.h"
#include "widgets/slam_visualization_widget.h"
#include "widgets/arm_control_widget.h"
#include "communication/ros2_interface.h"

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(ROS2Interface* ros2_interface, QWidget *parent = nullptr);
    ~MainWindow();
    
private slots:
    void onRobotStatusUpdated(const QString& status);
    void onEmergencyStop();
    void onSettingsTriggered();
    
private:
    Ui::MainWindow *ui;
    ROS2Interface* ros2_interface_;
    
    // 主要组件
    RobotStatusWidget* robot_status_widget_;
    CameraDisplayWidget* camera_display_widget_;
    SlamVisualizationWidget* slam_visualization_widget_;
    ArmControlWidget* arm_control_widget_;
    
    void setupUI();
    void setupMenuBar();
    void setupToolBar();
    void setupStatusBar();
    void connectSignals();
};

#endif // MAIN_WINDOW_H