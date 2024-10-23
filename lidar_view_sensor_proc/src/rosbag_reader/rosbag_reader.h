#ifndef DB3_READER_H_
#define DB3_READER_H_

#include <memory>

#include "config.h"
#include "frame_store/frame_store.h"

namespace LidarViewRos2 {
namespace SensorProc {
class RosbagReader {
public:
    RosbagReader(const std::shared_ptr<FrameStore>& pcProc) : pc_proc_(pcProc) {}
    ~RosbagReader() = default;

    void Init(const Config& conf);

private:
    void ReadDatas(const std::string& filePath, const Config& conf);
    void TransformPointCloud(sensor_msgs::msg::PointCloud2& pc, Eigen::Affine3d& affine);

private:
    std::shared_ptr<FrameStore> pc_proc_ = nullptr;
    std::unordered_map<std::string, std::string> topics_name2type_;
};

} // namespace SensorProc
} // namespace LidarViewRos2

#endif
