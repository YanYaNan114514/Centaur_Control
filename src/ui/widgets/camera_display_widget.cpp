#include "camera_display_widget.h"
#include <QGroupBox>

CameraDisplayWidget::CameraDisplayWidget(QWidget *parent)
    : QWidget(parent), is_recording_(false)
{
    setupUI();
}

void CameraDisplayWidget::setupUI()
{
    auto layout = new QVBoxLayout(this);
    
    auto groupBox = new QGroupBox("Camera Display", this);
    auto groupLayout = new QVBoxLayout(groupBox);
    
    // 控制面板
    auto controlLayout = new QHBoxLayout();
    
    camera_selector_ = new QComboBox(this);
    camera_selector_->addItems({"Camera 1", "Camera 2", "Depth Camera"});
    connect(camera_selector_, &QComboBox::currentTextChanged,
            this, &CameraDisplayWidget::onCameraSelectionChanged);
    
    record_button_ = new QPushButton("Record", this);
    record_button_->setCheckable(true);
    connect(record_button_, &QPushButton::toggled,
            this, &CameraDisplayWidget::onRecordToggled);
    
    controlLayout->addWidget(camera_selector_);
    controlLayout->addWidget(record_button_);
    controlLayout->addStretch();
    
    // 图像显示
    image_label_ = new QLabel(this);
    image_label_->setMinimumSize(400, 300);
    image_label_->setStyleSheet("border: 1px solid gray;");
    image_label_->setAlignment(Qt::AlignCenter);
    image_label_->setText("No Image");
    
    groupLayout->addLayout(controlLayout);
    groupLayout->addWidget(image_label_);
    
    layout->addWidget(groupBox);
}

void CameraDisplayWidget::updateImage(const sensor_msgs::msg::Image::SharedPtr msg)
{
    // TODO: 实现图像转换和显示
    image_label_->setText(QString("Image received: %1x%2")
                         .arg(msg->width).arg(msg->height));
}

void CameraDisplayWidget::onCameraSelectionChanged(const QString& camera_name)
{
    // TODO: 切换相机源
}

void CameraDisplayWidget::onRecordToggled(bool recording)
{
    is_recording_ = recording;
    record_button_->setText(recording ? "Stop" : "Record");
}