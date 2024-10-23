#include "rosbag_reader.h"

#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rosbag2_cpp/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_storage/metadata_io.hpp>
#include <rosbag2_storage/storage_factory.hpp>
#ifdef ROS2_FOXY_VERSION
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp/serialization.hpp>
#else
#include <rosbag2_storage/storage_options.hpp>
#endif
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace LidarViewRos2 {
namespace SensorProc {

void RosbagReader::Init(const Config& conf)
{
    const std::string slash = (conf.db3FilePath.back() == '/') ? "" : "/";
    const std::string filePath = conf.db3FilePath + slash + conf.db3FileName;
    ReadDatas(filePath, conf);
}

void RosbagReader::ReadDatas(const std::string& file_path, const Config& conf)
{
#ifdef ROS2_FOXY_VERSION
    rosbag2_cpp::StorageOptions storage_options{};
#else
    rosbag2_storage::StorageOptions storage_options{};
#endif

    storage_options.uri = file_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options{};
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    rosbag2_cpp::readers::SequentialReader reader;
    reader.open(storage_options, converter_options);

    rosbag2_storage::StorageFilter storage_filter;
    storage_filter.topics = conf.topics;
    reader.set_filter(storage_filter);

    const auto topics = reader.get_all_topics_and_types();

    for (const auto& topic : topics) {
        topics_name2type_[topic.name] = topic.type;
    }

    int num = 1;

    while (reader.has_next()) {
        auto message = reader.read_next();
        auto serialized_message = rclcpp::SerializedMessage(*message->serialized_data);
        auto type = topics_name2type_[message->topic_name];
        auto frame_id = conf.frameIdMap.at(message->topic_name);
        if (type == "sensor_msgs/msg/PointCloud2") {
            auto serializer = rclcpp::Serialization<sensor_msgs::msg::PointCloud2>();
            sensor_msgs::msg::PointCloud2 pc2;
            serializer.deserialize_message(&serialized_message, &pc2);
            pc2.header.frame_id = frame_id;
            Eigen::Affine3d transform = conf.extrinsicMap.at(message->topic_name);
            TransformPointCloud(pc2, transform);
            pc_proc_->Input(message->topic_name, std::move(pc2));
            continue;
        }
        if (type == "sensor_msgs/msg/Imu") {
            auto serializer = rclcpp::Serialization<sensor_msgs::msg::Imu>();
            sensor_msgs::msg::Imu imu_data;
            serializer.deserialize_message(&serialized_message, &imu_data);
            imu_data.header.frame_id = frame_id;
            pc_proc_->Input(message->topic_name, std::move(imu_data));
            continue;
        }
    }
    pc_proc_->SetEndTime();
}

void RosbagReader::TransformPointCloud(sensor_msgs::msg::PointCloud2& pc, Eigen::Affine3d& affine)
{
    sensor_msgs::PointCloud2Iterator<float> iter_x(pc, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(pc, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(pc, "z");

    while (iter_x != iter_x.end() && iter_y != iter_y.end() && iter_z != iter_z.end()) {
        Eigen::Vector3d point(*iter_x, *iter_y, *iter_z);
        auto new_point = affine * point;
        *iter_x = new_point[0];
        *iter_y = new_point[1];
        *iter_z = new_point[2];
        ++iter_x;
        ++iter_y;
        ++iter_z;
    }
}

} // namespace SensorProc
} // namespace LidarViewRos2
