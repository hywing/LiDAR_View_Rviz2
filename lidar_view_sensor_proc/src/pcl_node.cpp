#include "pcl_node.h"

#include "config.h"
#include "read_param.h"

namespace LidarViewRos2 {
namespace SensorProc {
void PclNode::Init()
{
    std::cout << "read parameters" << std::endl;
    ReadParam reader(*this);
    auto& config = Config::GetInstance();
    reader.Read(config);

    if (config.topics.size() == 0) {
        std::cout << "Must set input topics in config.yaml!!" << std::endl;
        exit(0);
    }

    // commands sub
    runSignalSub_ = this->create_subscription<std_msgs::msg::Int16>("/pixel/lv/run_signal", 10, std::bind(&PclNode::RunSignalCallback, this, std::placeholders::_1));
    beginSignalSub_ = this->create_subscription<std_msgs::msg::Float32>("/pixel/lv/begin_signal", 10, std::bind(&PclNode::BeginSignalCallback, this, std::placeholders::_1));
    rateSignalSub_ = this->create_subscription<std_msgs::msg::Float32>("/pixel/lv/rate_signal", 10, std::bind(&PclNode::RateSignalCallback, this, std::placeholders::_1));

    pcProc_ = std::make_shared<FrameStore>(*this, config.topics[0], config.db3FilePath);

    rosbag_reader_ = std::make_unique<RosbagReader>(pcProc_);
    rosbag_reader_->Init(config);
    pcProc_->Init();
    pcProc_->SetBeginFrame(0.0);
}

#ifdef ROS2_FOXY_VERSION
void PclNode::RunSignalCallback(const std_msgs::msg::Int16::SharedPtr msg)
#else
void PclNode::RunSignalCallback(const std_msgs::msg::Int16& msg)
#endif
{
#ifdef ROS2_FOXY_VERSION
    auto cmd = msg->data;
#else
    auto cmd = msg.data;
#endif
    switch (cmd) {
        case 0: { // stop mode
            pcProc_->Pause(cmd);
            break;
        }
        case 1: { // play mode
            pcProc_->Pause(cmd);
            break;
        }
        case 2: { // back mode
            pcProc_->StepPrev();
            break;
        }
        case 3: { // step mode
            pcProc_->StepNext();
            break;
        }
        case 4: { // save mode
            pcProc_->SaveFrame();
            break;
        }
        default: {
            break;
        }
    }
}

#ifdef ROS2_FOXY_VERSION
void PclNode::BeginSignalCallback(const std_msgs::msg::Float32::SharedPtr msg) { pcProc_->SetBeginFrame(msg->data); }
#else
void PclNode::BeginSignalCallback(const std_msgs::msg::Float32& msg) { pcProc_->SetBeginFrame(msg.data); }
#endif
#ifdef ROS2_FOXY_VERSION
void PclNode::RateSignalCallback(const std_msgs::msg::Float32::SharedPtr msg) { pcProc_->SetRate(msg->data); }
#else
void PclNode::RateSignalCallback(const std_msgs::msg::Float32& msg) { pcProc_->SetRate(msg.data); }
#endif

} // namespace SensorProc
} // namespace LidarViewRos2
