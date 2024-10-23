#ifndef PIXEL_TRANS_PARAMREADERBASE_H_
#define PIXEL_TRANS_PARAMREADERBASE_H_

#include <Eigen/Dense>
#include <iostream>
#include <string>

#include "rclcpp/rclcpp.hpp"
using namespace std;
namespace PixelTrans {

template <typename T>
std::ostream &operator<<(std::ostream &out, const std::vector<T> &x)
{
    out << "[";
    for (const auto &i : x) {
        out << i << " ";
    }
    out << "]" << std::endl;
    return out;
}

class ParamReaderBase {
public:
    explicit ParamReaderBase(rclcpp::Node &node) : node_(node) {}
    virtual ~ParamReaderBase() = default;

protected:
    template <typename T>
    void GetParam(const std::string &paramName, T &paramValue)
    {
        node_.declare_parameter<T>(paramName, paramValue);
        node_.get_parameter(paramName, paramValue);

        std::cout << paramName << ": " << paramValue << std::endl;
    }

    void GetAffine3d(std::vector<double> &extrisic, Eigen::Affine3d &affine)
    {
        Eigen::Quaterniond q(extrisic[6], extrisic[3], extrisic[4], extrisic[5]);
        q.normalize();
        Eigen::Vector3d t(extrisic[0], extrisic[1], extrisic[2]);
        affine = Eigen::Translation3d(t) * q.toRotationMatrix();
    }

protected:
    rclcpp::Node &node_;
};
} // namespace PixelTrans
#endif