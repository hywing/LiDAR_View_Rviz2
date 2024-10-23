#ifndef LIDARVIEWROS2_RVIZPLUGIN_TeleopPanel_H_
#define LIDARVIEWROS2_RVIZPLUGIN_TeleopPanel_H_

#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>

namespace LidarViewRos2 {
namespace RvizPlugin {
class TeleopPanel : public rviz_common::Panel {
    Q_OBJECT

public:
    TeleopPanel(QWidget* parent = nullptr);
    virtual ~TeleopPanel() {}

    virtual void load(const rviz_common::Config& config);
    virtual void save(rviz_common::Config config) const;

private Q_SLOTS:
    void StartButtonClick();
    void OnNextFrame();
    void OnPrevFrame();
    void StopFrame();
    void OnUpdateBeginTime();
    void OnUpdatePlayRate();
    void SpeedUp();
    void SpeedDown();
    void OnCaptureFrame();

private:
    void SetPanelLayout();
    void SignalStartStopPub();
#ifdef ROS2_FOXY_VERSION
    void CurrTimeSub(const std_msgs::msg::String::SharedPtr msg);
    void SelectPtSub(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
#else
    void CurrTimeSub(const std_msgs::msg::String& msg);
    void SelectPtSub(const sensor_msgs::msg::PointCloud2& msg);
#endif
    void StartSpin();

private:
    QLineEdit* startTimeEditor_;

    QLineEdit* rateEditor_;
    QLineEdit* currentTimeEditor_;
    QLineEdit* selectPtsEditor_;
    float playRate_;

    int16_t signalState_ = 0;

    static bool IsEmptyInput(const std::string& input);

    // The ROS relevant
    rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("plugin_test");
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velPublisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr signalPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr beginPub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr ratePub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr currTimeSub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr selectPtSub_;

    // UI
    QPushButton* pushButtonStart_;
};
} // namespace RvizPlugin
} // namespace LidarViewRos2

#endif // TELEOP_PANEL_H
