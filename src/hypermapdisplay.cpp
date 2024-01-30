#include "hypermap_rviz_plugin/hypermapdisplay.h"

#include <pluginlib/class_list_macros.hpp>
#include <iostream>
#include <OgreImage.h>
#include <OgreManualObject.h>

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <hypermap_msgs/msg/semantic_map.hpp>

#include <rviz_default_plugins/displays/map/map_display.hpp>
#include "hypermap_rviz_plugin/semanticmapdisplay.h"

namespace hypermap
{

HypermapDisplay::HypermapDisplay() : rviz_common::DisplayGroup()
{
    topic_property_ = new rviz_common::properties::RosTopicProperty("Topic", "", rosidl_generator_traits::data_type<hypermap_msgs::msg::HypermapMetaData>(),
                                                 "hypermap_msgs::msg::HypermapMetaData topic to subscribe to.", this, SLOT(updateTopic()));

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateLayerProps()));
}

void HypermapDisplay::setTopic(const QString &topic, const QString &datatype)
{
    topic_property_->setString(topic);
}

void HypermapDisplay::updateTopic()
{
    map_sub_.reset();
    auto rviz_ros_node_ = context_->getRosNodeAbstraction().lock();
    map_sub_ = rviz_ros_node_->get_raw_node()->create_subscription<hypermap_msgs::msg::HypermapMetaData>(topic_property_->getTopicStd(), 1, std::bind(&HypermapDisplay::receiveMapMeta, this, std::placeholders::_1));
}

QString HypermapDisplay::getDisplayName(const std::string &layerClass)
{
    if (layerClass == "OccupancyGridLayer")
        return "rviz/Map";
    else if (layerClass == "SemanticLayer")
        return "hypermap/SemanticMap";
    else
        return QString::fromStdString(layerClass) + QStringLiteral(" is not a valid layer class");
}

QString HypermapDisplay::getTopicName(const std::string &layerClass, const std::string &layerName)
{
    if (layerClass == "OccupancyGridLayer")
        return QString::fromStdString(current_map_meta_.node_name) + QStringLiteral("/") + QString::fromStdString(layerName) + QStringLiteral("_map");
    else if (layerClass == "SemanticLayer")
        return QString::fromStdString(current_map_meta_.node_name) + QStringLiteral("/") + QString::fromStdString(layerName) + QStringLiteral("_semmap");
    else
        return QString::fromStdString(layerClass) + QStringLiteral(" is not a valid layer class");
}

QString HypermapDisplay::getTopicType(const std::string &layerClass)
{
    if (layerClass == "OccupancyGridLayer")
        return rosidl_generator_traits::data_type<nav_msgs::msg::OccupancyGrid>();
    else if (layerClass == "SemanticLayer")
        return rosidl_generator_traits::data_type<hypermap_msgs::msg::SemanticMap>();
    else
        return QString::fromStdString(layerClass) + QStringLiteral(" is not a valid layer class");
}

void HypermapDisplay::updateLayerProps()
{
    removeAllDisplays();
    for (const hypermap_msgs::msg::LayerMetaData &layerMeta : current_map_meta_.layers)
    {
        Display *disp = createDisplay(getDisplayName(layerMeta.class_name));
        addDisplay(disp);
        disp->setObjectName(QString::fromStdString(layerMeta.name));
        disp->initialize(context_);
        disp->setTopic(getTopicName(layerMeta.class_name, layerMeta.name), getTopicType(layerMeta.class_name));
        disp->setEnabled(true);
    }
}

void HypermapDisplay::receiveMapMeta(const hypermap_msgs::msg::HypermapMetaData::ConstSharedPtr &msg)
{
    current_map_meta_ = *msg;
    //RCLCPP_INFO(node->get_logger(), "Map meta received");
    Q_EMIT mapReceived();
}

} // namespace hypermap

PLUGINLIB_EXPORT_CLASS(hypermap::HypermapDisplay, rviz_common::Display)