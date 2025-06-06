#ifndef ROBOT_STATUS_WIDGET_H
#define ROBOT_STATUS_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QGroupBox>
#include <QProgressBar>

class RobotStatusWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RobotStatusWidget(QWidget *parent = nullptr);
    
public slots:
    void updateStatus(const QString& status);
    void updateBatteryLevel(int level);
    void updateConnectionStatus(bool connected);
    
private:
    QLabel* status_label_;
    QLabel* connection_label_;
    QProgressBar* battery_bar_;
    
    void setupUI();
};

#endif // ROBOT_STATUS_WIDGET_H