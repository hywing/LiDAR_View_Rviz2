#ifndef SELECTED_POINTS_PUBLISHER_HPP
#define SELECTED_POINTS_PUBLISHER_HPP

// #ifndef Q_MOC_RUN // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QCursor>
#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/tool.hpp>

// #endif

#include <rviz_default_plugins/tools/select/selection_tool.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

namespace rviz_plugin_selected_points_publisher {
class SelectedPointsPublisher;

class SelectedPointsPublisher : public rviz_default_plugins::tools::SelectionTool {
    Q_OBJECT
public:
    SelectedPointsPublisher();
    virtual ~SelectedPointsPublisher();
    virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event);
    virtual int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel);

public Q_SLOTS:
    void UpdateTopic();

protected:
    int ProcessSelectedArea();

    rclcpp::Node::SharedPtr nh_ = rclcpp::Node::make_shared("select_points_plugin");
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_selected_publisher_;
    // rclcpp::Subscriber pointcloud_subscriber_;

    std::string tf_frame_;
    std::string rviz_cloud_topic_;
    std::string subscribed_cloud_topic_;

    sensor_msgs::msg::PointCloud2 selected_points_;

    bool selecting_;
    int num_selected_points_;
};
} // namespace rviz_plugin_selected_points_publisher

#endif // SELECTED_POINTS_PUBLISHER_HPP
