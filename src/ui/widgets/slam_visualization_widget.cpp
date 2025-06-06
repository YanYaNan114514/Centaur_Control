#include "slam_visualization_widget.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QGroupBox>

SlamVisualizationWidget::SlamVisualizationWidget(QWidget *parent)
    : QWidget(parent)
{
    setupUI();
}

void SlamVisualizationWidget::setupUI()
{
    auto layout = new QVBoxLayout(this);
    auto groupBox = new QGroupBox("SLAM Visualization", this);
    auto groupLayout = new QVBoxLayout(groupBox);
    
    auto mapLabel = new QLabel("Map display area", this);
    mapLabel->setMinimumSize(400, 300);
    mapLabel->setStyleSheet("border: 1px solid gray;");
    mapLabel->setAlignment(Qt::AlignCenter);
    
    groupLayout->addWidget(mapLabel);
    layout->addWidget(groupBox);
}

void SlamVisualizationWidget::updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    // TODO: 实现地图可视化
}