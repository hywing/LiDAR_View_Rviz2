#include "selected_points_publisher/selected_points_publisher.hpp"

#include <QVariant>
#include <QKeyEvent>
#include <iostream>
#include <visualization_msgs/msg/marker.hpp>

#include "OgreCamera.h"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/view_controller.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/viewport_mouse_event.hpp"

namespace rviz_plugin_selected_points_publisher {
SelectedPointsPublisher::SelectedPointsPublisher() { UpdateTopic(); }

SelectedPointsPublisher::~SelectedPointsPublisher() {}

void SelectedPointsPublisher::UpdateTopic()
{
    rviz_cloud_topic_ = std::string("/rviz_selected_points");

    rviz_selected_publisher_ = nh_->create_publisher<sensor_msgs::msg::PointCloud2>(rviz_cloud_topic_.c_str(), 1);
    num_selected_points_ = 0;
}

int SelectedPointsPublisher::processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel)
{
    if (event->type() == QKeyEvent::KeyPress) {
        if (event->key() == 'c' || event->key() == 'C') {
            const auto selection_manager = context_->getSelectionManager();
            const auto selection = selection_manager->getSelection();
            selection_manager->removeHighlight();
            visualization_msgs::msg::Marker marker;
            // Set the frame ID and timestamp.  See the TF tutorials for information on these.
            marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
            marker.header.stamp = rclcpp::Clock().now();
            marker.ns = "basic_shapes";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::DELETE;
            // marker.lifetime = ros::Duration();
            num_selected_points_ = 0;
        } else if (event->key() == 'p' || event->key() == 'P') {
            rviz_selected_publisher_->publish(selected_points_);
        }
    }
    return 0;
}

int SelectedPointsPublisher::processMouseEvent(rviz_common::ViewportMouseEvent& event)
{
    int flags = rviz_default_plugins::tools::SelectionTool::processMouseEvent(event);
    if (event.alt()) {
        selecting_ = false;
    } else {
        if (event.leftDown()) {
            selecting_ = true;
        }
    }

    if (selecting_) {
        if (event.leftUp()) {
            this->ProcessSelectedArea();
        }
    }
    return flags;
}

int SelectedPointsPublisher::ProcessSelectedArea()
{
    const auto selection_manager = context_->getSelectionManager();
    const auto selection = selection_manager->getSelection();
    const auto model = selection_manager->getPropertyModel();

    selected_points_.header.frame_id = context_->getFixedFrame().toStdString();
    selected_points_.height = 1;
    selected_points_.point_step = 4 * 3;
    selected_points_.is_dense = false;
    selected_points_.is_bigendian = false;
    selected_points_.fields.resize(3);

    selected_points_.fields[0].name = "x";
    selected_points_.fields[0].offset = 0;
    selected_points_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    selected_points_.fields[0].count = 1;

    selected_points_.fields[1].name = "y";
    selected_points_.fields[1].offset = 4;
    selected_points_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    selected_points_.fields[1].count = 1;

    selected_points_.fields[2].name = "z";
    selected_points_.fields[2].offset = 8;
    selected_points_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    selected_points_.fields[2].count = 1;

    int index = 0;
    while (model->hasIndex(index, 0)) {
        selected_points_.row_step = (index + 1) * selected_points_.point_step;
        selected_points_.data.resize(selected_points_.row_step);

        QModelIndex child_index = model->index(index, 0);

        rviz_common::properties::Property* child = model->getProp(child_index);
        rviz_common::properties::VectorProperty* subchild = (rviz_common::properties::VectorProperty*)child->childAt(0);
        Ogre::Vector3 point_data = subchild->getVector();

        const uint8_t *p = &selected_points_.data[0] + index * selected_points_.point_step;
        *(float*)p = point_data.x;
        p += 4;
        *(float*)p = point_data.y;
        p += 4;
        *(float*)p = point_data.z;
        p += 4;
        index++;
    }
    num_selected_points_ = index;
    selected_points_.width = index;
    selected_points_.header.stamp = rclcpp::Clock().now();
    rviz_selected_publisher_->publish(selected_points_);
    return 0;
}
} // namespace rviz_plugin_selected_points_publisher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugin_selected_points_publisher::SelectedPointsPublisher, rviz_common::Tool)
