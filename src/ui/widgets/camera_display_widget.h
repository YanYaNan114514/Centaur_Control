#ifndef CAMERA_DISPLAY_WIDGET_H
#define CAMERA_DISPLAY_WIDGET_H

#include <QWidget>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QComboBox>
#include <QPushButton>
#include <QTimer>
#include <sensor_msgs/msg/image.hpp>

class CameraDisplayWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraDisplayWidget(QWidget *parent = nullptr);
    
public slots:
    void updateImage(const sensor_msgs::msg::Image::SharedPtr msg);
    
private slots:
    void onCameraSelectionChanged(const QString& camera_name);
    void onRecordToggled(bool recording);
    
private:
    QLabel* image_label_;
    QComboBox* camera_selector_;
    QPushButton* record_button_;
    
    bool is_recording_;
    
    void setupUI();
};

#endif // CAMERA_DISPLAY_WIDGET_H