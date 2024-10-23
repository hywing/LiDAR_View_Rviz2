#ifndef LIDARVIEWROS2_SENSORSPROC_READPARAM_H_
#define LIDARVIEWROS2_SENSORSPROC_READPARAM_H_

#include "param_reader_base.h"
#include "config.h"

namespace LidarViewRos2 {
namespace SensorProc {
class ReadParam : public PixelTrans::ParamReaderBase {
public:
    explicit ReadParam(rclcpp::Node& node) : ParamReaderBase(node) {}
    virtual ~ReadParam() {}

    void Read(Config& conf);

};
}   // namespace LidarViewRos2
}   // namespace SensorProc
#endif