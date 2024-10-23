#include "read_param.h"

namespace LidarViewRos2 {
namespace SensorProc {
void ReadParam::Read(Config& conf)
{
    GetParam("file_path", conf.db3FilePath);
    GetParam("file_name", conf.db3FileName);
    GetParam("topics", conf.topics);

    std::vector<std::string> frameIds;
    GetParam("frame_ids", frameIds);
    assert(frameIds.size() == conf.topics.size());

    for (int i = 0; i < conf.topics.size(); i++) {
        const auto& topic = conf.topics[i];
        // extrisic name
        std::string extrisicName = topic + "/extrinsic";
        std::vector<double> extrisic;
        GetParam(extrisicName, extrisic);
        Eigen::Affine3d affine;
        GetAffine3d(extrisic, affine);
        conf.extrinsicMap[topic] = affine;
        conf.frameIdMap[topic] = frameIds[i];
    }
}
} // namespace SensorProc
} // namespace LidarViewRos2