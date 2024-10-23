#ifndef LIDARVIEWROS2_SENSORSPROC_CONFIG_H_
#define LIDARVIEWROS2_SENSORSPROC_CONFIG_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

namespace LidarViewRos2 {
namespace SensorProc {
class Config {
public:
    ~Config() { std::cout << "destructor config called" << std::endl; }

    Config(const Config&) = delete;
    Config& operator=(const Config&) = delete;

    static Config& GetInstance()
    {
        static Config instance;
        return instance;
    }

private:
    Config() {}

public:
    std::string db3FilePath = "/home/ptdev/20230801-LidarA0-IMU/";
    std::string db3FileName = "20230801-LidarA0-IMU.db3";
    std::vector<std::string> topics{};
    std::unordered_map<std::string, std::string> frameIdMap;
    std::unordered_map<std::string, Eigen::Affine3d> extrinsicMap;
};
} // namespace SensorProc
} // namespace LidarViewRos2
#endif