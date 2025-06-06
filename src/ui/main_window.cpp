#include "main_window.h"
#include "ui_main_window.h"
#include "dialogs/settings_dialog.h"
#include "utils/logger.h"
#include <QMessageBox>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QSplitter>

MainWindow::MainWindow(ROS2Interface* ros2_interface, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
    , ros2_interface_(ros2_interface)
{
    ui->setupUi(this);
    setupUI();
    setupMenuBar();
    setupToolBar();
    setupStatusBar();
    connectSignals();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::setupUI()
{
    setWindowTitle("CentaurControl - Robot Control Interface");
    setMinimumSize(1200, 800);
    
    // 创建中央部件
    auto centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    
    // 创建主布局
    auto mainLayout = new QVBoxLayout(centralWidget);
    
    // 创建分割器
    auto mainSplitter = new QSplitter(Qt::Horizontal, this);
    
    // 左侧控制面板
    auto leftPanel = new QWidget();
    auto leftLayout = new QVBoxLayout(leftPanel);
    
    robot_status_widget_ = new RobotStatusWidget(this);
    arm_control_widget_ = new ArmControlWidget(this);
    
    leftLayout->addWidget(robot_status_widget_);
    leftLayout->addWidget(arm_control_widget_);
    leftLayout->addStretch();
    
    // 右侧显示面板
    auto rightPanel = new QWidget();
    auto rightLayout = new QVBoxLayout(rightPanel);
    
    camera_display_widget_ = new CameraDisplayWidget(this);
    slam_visualization_widget_ = new SlamVisualizationWidget(this);
    
    rightLayout->addWidget(camera_display_widget_);
    rightLayout->addWidget(slam_visualization_widget_);
    
    mainSplitter->addWidget(leftPanel);
    mainSplitter->addWidget(rightPanel);
    mainSplitter->setStretchFactor(0, 1);
    mainSplitter->setStretchFactor(1, 2);
    
    mainLayout->addWidget(mainSplitter);
}

void MainWindow::setupMenuBar()
{
    auto fileMenu = menuBar()->addMenu("&File");
    auto settingsAction = fileMenu->addAction("&Settings");
    connect(settingsAction, &QAction::triggered, this, &MainWindow::onSettingsTriggered);
    
    fileMenu->addSeparator();
    auto exitAction = fileMenu->addAction("E&xit");
    connect(exitAction, &QAction::triggered, this, &QWidget::close);
    
    auto helpMenu = menuBar()->addMenu("&Help");
    auto aboutAction = helpMenu->addAction("&About");
    connect(aboutAction, &QAction::triggered, [this]() {
        QMessageBox::about(this, "About CentaurControl", 
                          "CentaurControl v1.0.0\nRobot Control Interface");
    });
}

void MainWindow::setupToolBar()
{
    auto toolbar = addToolBar("Main");
    
    auto emergencyStopAction = toolbar->addAction("Emergency Stop");
    emergencyStopAction->setIcon(style()->standardIcon(QStyle::SP_DialogCancelButton));
    connect(emergencyStopAction, &QAction::triggered, this, &MainWindow::onEmergencyStop);
    
    toolbar->addSeparator();
    
    auto settingsAction = toolbar->addAction("Settings");
    settingsAction->setIcon(style()->standardIcon(QStyle::SP_ComputerIcon));
    connect(settingsAction, &QAction::triggered, this, &MainWindow::onSettingsTriggered);
}

void MainWindow::setupStatusBar()
{
    statusBar()->showMessage("Ready");
}

void MainWindow::connectSignals()
{
    if (ros2_interface_) {
        connect(ros2_interface_, &ROS2Interface::robotStatusUpdated,
                this, &MainWindow::onRobotStatusUpdated);
        
        connect(ros2_interface_, &ROS2Interface::imageReceived,
                camera_display_widget_, &CameraDisplayWidget::updateImage);
        
        connect(ros2_interface_, &ROS2Interface::mapReceived,
                slam_visualization_widget_, &SlamVisualizationWidget::updateMap);
    }
}

void MainWindow::onRobotStatusUpdated(const QString& status)
{
    statusBar()->showMessage(status);
    robot_status_widget_->updateStatus(status);
}

void MainWindow::onEmergencyStop()
{
    if (ros2_interface_) {
        ros2_interface_->publishChassisVelocity(0.0, 0.0, 0.0);
    }
    
    QMessageBox::warning(this, "Emergency Stop", "Emergency stop activated!");
    Logger::warning("Emergency stop activated by user");
}

void MainWindow::onSettingsTriggered()
{
    SettingsDialog dialog(this);
    dialog.exec();
}