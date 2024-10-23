#ifndef LIDARVIEWRVIZ2_SENSORPROC_NODE_H_
#define LIDARVIEWRVIZ2_SENSORPROC_NODE_H_

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int16.hpp>

#include "rosbag_reader/rosbag_reader.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

namespace LidarViewRos2 {
namespace SensorProc {
class PclNode : public rclcpp::Node {
public:
    PclNode() : Node("lidar_view_sensor_proc_node") {}
    void Init();
#ifdef ROS2_FOXY_VERSION
    void RunSignalCallback(const std_msgs::msg::Int16::SharedPtr msg);
    void BeginSignalCallback(const std_msgs::msg::Float32::SharedPtr msg);
#else
    void RunSignalCallback(const std_msgs::msg::Int16& msg);
    void BeginSignalCallback(const std_msgs::msg::Float32& msg);
#endif
    void SendStaticTF(const std::vector<double> &transform, const std::string &childFrameid);
#ifdef ROS2_FOXY_VERSION
    void RateSignalCallback(const std_msgs::msg::Float32::SharedPtr msg);
#else
    void RateSignalCallback(const std_msgs::msg::Float32& msg);
#endif

private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcPub0_;
    rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr runSignalSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr beginSignalSub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rateSignalSub_;

    std::shared_ptr<FrameStore> pcProc_ = nullptr;
    std::unique_ptr<RosbagReader> rosbag_reader_ = nullptr;
};
} // namespace SensorProc
} // namespace LidarViewRos2

#endif
