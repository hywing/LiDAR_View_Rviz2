#include "play_panel.h"

#include <stdio.h>

class QLineEdit;

namespace LidarViewRos2 {
namespace RvizPlugin {

TeleopPanel::TeleopPanel(QWidget* parent) : rviz_common::Panel(parent), playRate_(1.0)
{
    signalPub_ = nh_->create_publisher<std_msgs::msg::Int16>("/pixel/lv/run_signal", 5);
    beginPub_ = nh_->create_publisher<std_msgs::msg::Float32>("/pixel/lv/begin_signal", 5);
    ratePub_ = nh_->create_publisher<std_msgs::msg::Float32>("/pixel/lv/rate_signal", 5);

    currTimeSub_ = nh_->create_subscription<std_msgs::msg::String>("/pixel/lv/current_time", 10, std::bind(&TeleopPanel::CurrTimeSub, this, std::placeholders::_1));
    selectPtSub_ = nh_->create_subscription<sensor_msgs::msg::PointCloud2>("/rviz_selected_points", 10, std::bind(&TeleopPanel::SelectPtSub, this, std::placeholders::_1));

    std::thread t(&TeleopPanel::StartSpin, this);
    t.detach();

    SetPanelLayout();
}

#ifdef ROS2_FOXY_VERSION
void TeleopPanel::CurrTimeSub(const std_msgs::msg::String::SharedPtr msg)
#else
void TeleopPanel::CurrTimeSub(const std_msgs::msg::String& msg)
#endif
{
#ifdef ROS2_FOXY_VERSION
    QString currTime = QString::fromStdString(msg->data);
#else
    QString currTime = QString::fromStdString(msg.data);
#endif
    currentTimeEditor_->setText(currTime);
}

#ifdef ROS2_FOXY_VERSION
void TeleopPanel::SelectPtSub(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
#else
void TeleopPanel::SelectPtSub(const sensor_msgs::msg::PointCloud2& msg)
#endif
{
#ifdef ROS2_FOXY_VERSION
    const auto ptsNum = msg->width;
#else
    const auto ptsNum = msg.width;
#endif
    QString ptsNumQStr = QString::fromStdString(std::to_string(ptsNum));
    selectPtsEditor_->setText(ptsNumQStr);
}

void TeleopPanel::StartSpin()
{
    while (rclcpp::ok()) {
        rclcpp::spin_some(nh_);
    }
}

bool TeleopPanel::IsEmptyInput(const std::string& input)
{
    if (input.empty()) {
        return true;
    }
    for (const auto& c : input) {
        if (c != ' ') {
            return false;
        }
    }
    return true;
}

void TeleopPanel::SetPanelLayout()
{
    QVBoxLayout* vLayout = new QVBoxLayout(this);

    QHBoxLayout* hLayout0 = new QHBoxLayout();
    hLayout0->addWidget(new QLabel("Start "));
    startTimeEditor_ = new QLineEdit;
    hLayout0->addWidget(startTimeEditor_);
    connect(startTimeEditor_, SIGNAL(editingFinished()), this, SLOT(OnUpdateBeginTime()));
    hLayout0->addWidget(new QLabel("Time "));
    currentTimeEditor_ = new QLineEdit;
    currentTimeEditor_->setReadOnly(true);
    hLayout0->addWidget(currentTimeEditor_);

    vLayout->addLayout(hLayout0);

    QHBoxLayout* hLayout2 = new QHBoxLayout();
    hLayout2->addWidget(new QLabel("Rate "));
    rateEditor_ = new QLineEdit;
    hLayout2->addWidget(rateEditor_);
    connect(rateEditor_, SIGNAL(editingFinished()), this, SLOT(OnUpdatePlayRate()));

    QPushButton* pushButtonSpdUp = new QPushButton("Speed Up");
    hLayout2->addWidget(pushButtonSpdUp);
    connect(pushButtonSpdUp, SIGNAL(clicked()), this, SLOT(SpeedUp()));

    QPushButton* pushButtonSpdDown = new QPushButton("Speed Down");
    hLayout2->addWidget(pushButtonSpdDown);
    connect(pushButtonSpdDown, SIGNAL(clicked()), this, SLOT(SpeedDown()));
    vLayout->addLayout(hLayout2);

    QHBoxLayout* hLayout1 = new QHBoxLayout();
    QPushButton* prev;
    prev = new QPushButton("Prev");
    hLayout1->addWidget(prev);
    connect(prev, SIGNAL(clicked()), this, SLOT(OnPrevFrame()));

    // QPushButton* pushButtonStart_;
    pushButtonStart_ = new QPushButton("Run");
    hLayout1->addWidget(pushButtonStart_);
    connect(pushButtonStart_, SIGNAL(clicked()), this, SLOT(StartButtonClick()));

    QPushButton* next;
    next = new QPushButton("Next");
    hLayout1->addWidget(next);
    connect(next, SIGNAL(clicked()), this, SLOT(OnNextFrame()));
    vLayout->addLayout(hLayout1);

    QPushButton* pushButtonCapture = new QPushButton("Capture");
    vLayout->addWidget(pushButtonCapture);
    connect(pushButtonCapture, SIGNAL(clicked()), this, SLOT(OnCaptureFrame()));

    QHBoxLayout* hLayout3 = new QHBoxLayout();
    hLayout3->addWidget(new QLabel("Selected Points Number: "));
    selectPtsEditor_ = new QLineEdit;
    selectPtsEditor_->setReadOnly(true);
    hLayout3->addWidget(selectPtsEditor_);
    vLayout->addLayout(hLayout3);
}

void TeleopPanel::StartButtonClick()
{
    SignalStartStopPub();
}

void TeleopPanel::SignalStartStopPub()
{
    if (signalState_ == 0) {
        signalState_ = 1;
        pushButtonStart_->setText("Pause");
        std_msgs::msg::Int16 msg;
        msg.data = signalState_;

        signalPub_->publish(msg);
    } else if (signalState_ == 1) {
        pushButtonStart_->setText("Run");
        StopFrame();
    }
}

void TeleopPanel::OnPrevFrame()
{
    StopFrame();

    if (signalState_ == 0) {
        std_msgs::msg::Int16 msg;
        msg.data = 2;
        signalPub_->publish(msg);
    }
}

void TeleopPanel::OnNextFrame()
{
    StopFrame();

    if (signalState_ == 0) {
        std_msgs::msg::Int16 msg;
        msg.data = 3;
        signalPub_->publish(msg);
    }
}

void TeleopPanel::StopFrame()
{
    signalState_ = 0;
    std_msgs::msg::Int16 msg;
    msg.data = signalState_;

    signalPub_->publish(msg);
}

void TeleopPanel::OnCaptureFrame()
{
    StopFrame();

    std_msgs::msg::Int16 msg;
    msg.data = 4;

    signalPub_->publish(msg);
}

void TeleopPanel::OnUpdateBeginTime()
{
    QString temp_string = startTimeEditor_->text();
    const std::string std_str = temp_string.toStdString();
    if (IsEmptyInput(std_str)) {
        return;
    }
    StopFrame();
    std_msgs::msg::Float32 msg;
    msg.data = temp_string.toFloat();
    beginPub_->publish(msg);
}

void TeleopPanel::OnUpdatePlayRate()
{
    QString temp_string = rateEditor_->text();
    const std::string std_str = temp_string.toStdString();
    if (IsEmptyInput(std_str)) {
        return;
    }
    playRate_ = temp_string.toFloat();
    playRate_ = std::min(playRate_, 10.0F);
    playRate_ = std::max(playRate_, 0.1F);
    std_msgs::msg::Float32 msg;
    msg.data = playRate_;
    ratePub_->publish(msg);
}

void TeleopPanel::SpeedUp()
{
    uint32_t rateTmp = std::round(playRate_ / 0.5);
    playRate_ = (static_cast<float>(rateTmp) + 1.0) * 0.5;
    playRate_ = std::min(playRate_, 10.0F);
    std_msgs::msg::Float32 msg;
    msg.data = playRate_;
    ratePub_->publish(msg);
    std::cout << "Spd Up: " << playRate_ << std::endl;

    const std::string rateStr = std::to_string(playRate_).substr(0, 3);
    const QString display_text = QString::fromStdString(rateStr);
    rateEditor_->setText(display_text);
}

void TeleopPanel::SpeedDown()
{
    uint32_t rateTmp = std::round(playRate_ / 0.5);
    playRate_ = (static_cast<float>(rateTmp) - 1.0) * 0.5;
    playRate_ = std::max(playRate_, 0.1F);
    std_msgs::msg::Float32 msg;
    msg.data = playRate_;
    ratePub_->publish(msg);
    std::cout << "Spd Down: " << playRate_ << std::endl;

    const std::string rateStr = std::to_string(playRate_).substr(0, 3);
    const QString display_text = QString::fromStdString(rateStr);
    rateEditor_->setText(display_text);
}

void TeleopPanel::save(rviz_common::Config config) const { rviz_common::Panel::save(config); }

void TeleopPanel::load(const rviz_common::Config& config) { rviz_common::Panel::load(config); }
} // namespace RvizPlugin
} // namespace LidarViewRos2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(LidarViewRos2::RvizPlugin::TeleopPanel, rviz_common::Panel)
// END_TUTORIAL
