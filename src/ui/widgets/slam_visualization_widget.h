#ifndef SLAM_VISUALIZATION_WIDGET_H
#define SLAM_VISUALIZATION_WIDGET_H

#include <QWidget>
#include <nav_msgs/msg/occupancy_grid.hpp>

class SlamVisualizationWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SlamVisualizationWidget(QWidget *parent = nullptr);
    
public slots:
    void updateMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    
private:
    void setupUI();
};

#endif // SLAM_VISUALIZATION_WIDGET_H