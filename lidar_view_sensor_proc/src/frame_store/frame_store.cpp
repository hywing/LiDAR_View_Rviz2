#include "frame_store/frame_store.h"

#include <boost/filesystem.hpp>

namespace LidarViewRos2 {
namespace SensorProc {

void FrameStore::Init()
{
    currTimePub_ = node_.create_publisher<std_msgs::msg::String>("/pixel/lv/current_time", 10);
    std::thread t(&FrameStore::Play, this);
    t.detach();
}

void FrameStore::Input(const std::string& topicName, const sensor_msgs::msg::PointCloud2&& pc)
{
    if (topicName == mainLidarTopic_) {
        const double timestamp = pc.header.stamp.sec + pc.header.stamp.nanosec / 1e9;
        if (timeQueue_.size() == 0) {
            startTime_ = timestamp;
        }
        timeQueue_.emplace_back(timestamp);
    }

    if (pcStores_.find(topicName) == pcStores_.end()) {
        pcPubs_[topicName] = node_.create_publisher<sensor_msgs::msg::PointCloud2>(topicName, 10);
        sensorsIndex_[topicName] = 0;
    }
    pcStores_[topicName].emplace_back(pc);
}

void FrameStore::Input(const std::string& topicName, const sensor_msgs::msg::Imu&& msg)
{
    if (imuStores_.find(topicName) == imuStores_.end()) {
        imuPubs_[topicName] = node_.create_publisher<sensor_msgs::msg::Imu>(topicName, 10);
        sensorsIndex_[topicName] = 0;
    }
    imuStores_[topicName].emplace_back(msg);
}

void FrameStore::SetEndTime()
{
    if (timeQueue_.size() > 0) {
        endTime_ = timeQueue_.back();
    }
    for (const auto& pcMap : pcStores_) {
        std::cout << pcMap.first << ": " << pcMap.second.size() << std::endl;
        sensorsIndexMax_[pcMap.first] = pcMap.second.size();
    }
    for (const auto& imuMap : imuStores_) {
        sensorsIndexMax_[imuMap.first] = imuMap.second.size();
    }
}

void FrameStore::Play()
{
    currTime_ = startTime_;
    while (rclcpp::ok()) {
        if (playStatus_ == 1) {
            if (!isForward_) {
                StepIndex(true);
            }
            currTime_ += deltaTime_;
            PubPointCloud(currTime_);
            PubImu(currTime_);
            currTime_ = std::min(currTime_, endTime_);
            isForward_ = true;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(static_cast<uint32_t>(loopTime_ * 1e6)));
    }
}

void FrameStore::PubPointCloud(const double timestamp, const bool isForward)
{
    for (const auto& pcQueue : pcStores_) {
        auto& index = sensorsIndex_[pcQueue.first];
        if (isForward) {
            while ((index < (static_cast<int32_t>(pcQueue.second.size()) - 1)) && ((pcQueue.second.at(index).header.stamp.sec + pcQueue.second.at(index).header.stamp.nanosec / 1e9) < timestamp)) {
                pcPubs_[pcQueue.first]->publish(pcQueue.second.at(index));
                PubCurrTime(timestamp);
                ++index;
            }
            index = std::min(index, static_cast<int32_t>(pcQueue.second.size()) - 1);
        } else {
            while ((index >= 0) && ((pcQueue.second.at(index).header.stamp.sec + pcQueue.second.at(index).header.stamp.nanosec / 1e9) > timestamp)) {
                pcPubs_[pcQueue.first]->publish(pcQueue.second.at(index));
                PubCurrTime(timestamp);
                --index;
            }
            index = std::max(index, 0);
        }
    }
}

void FrameStore::PubImu(const double timestamp, const bool isForward)
{
    for (const auto& imuQueue : imuStores_) {
        auto& index = sensorsIndex_[imuQueue.first];
        if (isForward) {
            while ((index < imuQueue.second.size()) && ((imuQueue.second.at(index).header.stamp.sec + imuQueue.second.at(index).header.stamp.nanosec / 1e9) < timestamp)) {
                imuPubs_[imuQueue.first]->publish(imuQueue.second.at(index));
                ++index;
            }
        } else {
            while ((index > 0) && ((imuQueue.second.at(index).header.stamp.sec + imuQueue.second.at(index).header.stamp.nanosec / 1e9) > timestamp)) {
                imuPubs_[imuQueue.first]->publish(imuQueue.second.at(index));
                --index;
            }
        }
    }
}

void FrameStore::Pause(const int16_t state) { playStatus_ = state; }

void FrameStore::StepNext()
{
    if (pcStores_[mainLidarTopic_].empty()) {
        return;
    }

    if (!isForward_) {
        StepIndex(true);
        StepIndex(true);
    }

    auto index = sensorsIndex_[mainLidarTopic_];
    currTime_ = timeQueue_[index] + deltaTime_;
    PubPointCloud(currTime_);
    PubImu(currTime_);
    isForward_ = true;
}

void FrameStore::StepPrev()
{
    if (pcStores_[mainLidarTopic_].empty()) {
        return;
    }

    if (isForward_) {
        StepIndex(false);
        StepIndex(false);
    }

    auto index = sensorsIndex_[mainLidarTopic_];
    index = std::max(index, 0);
    currTime_ = timeQueue_[index] - deltaTime_;

    PubPointCloud(currTime_, false);
    PubImu(currTime_, false);
    isForward_ = false;
}

void FrameStore::SaveFrame()
{
    auto savePath = filePath_ + "/captures";
    if (!boost::filesystem::exists(savePath)) {
        boost::filesystem::create_directories(filePath_ + "/captures");
    }
    for (const auto& pcQueue : pcStores_) {
        std::string pcName = pcQueue.first;
        auto& index = sensorsIndex_[pcQueue.first];
        auto& pc = pcQueue.second.at(index);
        std::replace(pcName.begin(), pcName.end(), '/', '_');
        pcName = savePath + "/" + pcName + "_" + std::to_string(pc.header.stamp.sec + pc.header.stamp.nanosec / 1e9) + ".pcd";
        pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
        pcl_conversions::toPCL(pc, *cloud);
        writer_.writeBinary(pcName, *cloud);
    }
}

void FrameStore::SetBeginFrame(const float t)
{
    currTime_ = t > 0 ? (startTime_ + t) : startTime_;
    currTime_ = std::min(currTime_, endTime_);

    FindPcBeginIndex(currTime_);
    FindImuBeginIndex(currTime_);
}

void FrameStore::FindPcBeginIndex(const double currTime)
{
    for (const auto& pcQueue : pcStores_) {
        for (size_t i = 0U; i < pcQueue.second.size(); ++i) {
            const auto& pc = pcQueue.second.at(i);
            const double timestamp = static_cast<double>(pc.header.stamp.sec) + static_cast<double>(pc.header.stamp.nanosec) / 1e9;
            sensorsIndex_[pcQueue.first] = i;
            if (timestamp > currTime) {
                break;
            }
        }
        pcPubs_[pcQueue.first]->publish(pcQueue.second.at(sensorsIndex_[pcQueue.first]));
        PubCurrTime(currTime);
    }
}

void FrameStore::FindImuBeginIndex(const double currTime)
{
    for (const auto& imuQueue : imuStores_) {
        for (size_t i = 0U; i < imuQueue.second.size(); ++i) {
            const auto& imu = imuQueue.second.at(i);
            const double timestamp = static_cast<double>(imu.header.stamp.sec) + static_cast<double>(imu.header.stamp.nanosec) / 1e9;
            sensorsIndex_[imuQueue.first] = i;
            if (timestamp > currTime) {
                break;
            }
        }
        imuPubs_[imuQueue.first]->publish(imuQueue.second.at(sensorsIndex_[imuQueue.first]));
    }
}

void FrameStore::StepIndex(const bool isForward)
{
    for (auto& index : sensorsIndex_) {
        if (isForward) {
            ++index.second;
            index.second = std::min(index.second, sensorsIndexMax_[index.first]);
        } else {
            --index.second;
            index.second = std::max(index.second, 0);
        }
    }
}

void FrameStore::PubCurrTime(const double time)
{
    auto time1 = std::to_string(time - startTime_);
    auto time2 = std::to_string(endTime_ - startTime_);
    std::string currTime = time1.substr(0, time1.find(".") + 2) + "/" + time2.substr(0, time2.find(".") + 2);
    std_msgs::msg::String msg;
    msg.data = currTime;
    currTimePub_->publish(msg);
}

} // namespace SensorProc
} // namespace LidarViewRos2