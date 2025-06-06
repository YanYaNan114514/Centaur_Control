#include "robot_status_widget.h"

RobotStatusWidget::RobotStatusWidget(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
}

void RobotStatusWidget::setupUI()
{
    auto layout = new QVBoxLayout(this);
    
    auto groupBox = new QGroupBox("Robot Status", this);
    auto groupLayout = new QVBoxLayout(groupBox);
    
    status_label_ = new QLabel("Status: Unknown", this);
    connection_label_ = new QLabel("Connection: Disconnected", this);
    
    battery_bar_ = new QProgressBar(this);
    battery_bar_->setRange(0, 100);
    battery_bar_->setValue(0);
    battery_bar_->setFormat("Battery: %p%");
    
    groupLayout->addWidget(status_label_);
    groupLayout->addWidget(connection_label_);
    groupLayout->addWidget(battery_bar_);
    
    layout->addWidget(groupBox);
    layout->addStretch();
}

void RobotStatusWidget::updateStatus(const QString& status)
{
    status_label_->setText("Status: " + status);
}

void RobotStatusWidget::updateBatteryLevel(int level)
{
    battery_bar_->setValue(level);
}

void RobotStatusWidget::updateConnectionStatus(bool connected)
{
    connection_label_->setText(connected ? "Connection: Connected" : "Connection: Disconnected");
    connection_label_->setStyleSheet(connected ? "color: green;" : "color: red;");
}