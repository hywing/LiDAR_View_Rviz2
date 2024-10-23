#ifndef PC_PROCESS_H
#define PC_PROCESS_H

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "rclcpp/rclcpp.hpp"

namespace LidarViewRos2 {
namespace SensorProc {
class FrameStore {
public:
    void Init();
    FrameStore() = default;
    FrameStore(rclcpp::Node& nh, const std::string& topic, const std::string& filepath) : node_(nh), mainLidarTopic_(topic), filePath_(filepath) { timeQueue_.clear(); }

    void Input(const std::string& topicName, const sensor_msgs::msg::PointCloud2&& pc);

    void Input(const std::string& topicName, const sensor_msgs::msg::Imu&& msg);

    void SetEndTime();

    void Play();
    void Pause(const int16_t state);
    void StepNext();
    void StepPrev();

    void SaveFrame();

    void SetBeginFrame(const float t);

    inline void SetRate(const float rate) { loopTime_ = deltaTime_ / rate; }

private:
    void PubPointCloud(const double timestamp, const bool isForward = true);
    void PubImu(const double timestamp, const bool isForward = true);
    void PubCurrTime(const double time);
    void FindPcBeginIndex(const double currTime);
    void FindImuBeginIndex(const double currTime);
    void StepIndex(const bool isForward);

private:
    rclcpp::Node& node_;

    std::vector<double> timeQueue_;
    using pcPublisherPtr = rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr;
    using imuPublisherPtr = rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr currTimePub_;
    std::map<std::string, pcPublisherPtr> pcPubs_;
    std::map<std::string, std::vector<sensor_msgs::msg::PointCloud2>> pcStores_;
    std::map<std::string, imuPublisherPtr> imuPubs_;
    std::map<std::string, std::vector<sensor_msgs::msg::Imu>> imuStores_;
    std::map<std::string, int32_t> sensorsIndex_;
    std::map<std::string, int32_t> sensorsIndexMax_;

    std::string mainLidarTopic_;
    std::string filePath_;
    pcl::PCDWriter writer_;

    bool isForward_ = true;

    int index_ = 0;
    int playStatus_ = 0;
    double startTime_ = 0.0;
    double endTime_ = 0.0;
    const double deltaTime_ = 0.001;
    double loopTime_ = deltaTime_;
    double currTime_ = 0.0;
};

} // namespace SensorProc
} // namespace LidarViewRos2

#endif