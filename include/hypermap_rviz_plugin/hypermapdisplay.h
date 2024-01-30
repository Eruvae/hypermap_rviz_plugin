#pragma once

#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/int_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <hypermap_msgs/msg/hypermap_meta_data.hpp>

namespace hypermap
{

class HypermapDisplay : public rviz_common::DisplayGroup
{
Q_OBJECT
public:
  HypermapDisplay();

  // Overrides from Display
  virtual void setTopic(const QString &topic, const QString &datatype);

Q_SIGNALS:
  void mapReceived();

protected Q_SLOTS:
  void updateTopic();
  void updateLayerProps();

protected:
  void receiveMapMeta(const hypermap_msgs::msg::HypermapMetaData::ConstSharedPtr &msg);

  hypermap_msgs::msg::HypermapMetaData current_map_meta_;
  rclcpp::Subscription<hypermap_msgs::msg::HypermapMetaData>::SharedPtr map_sub_;

  rviz_common::properties::RosTopicProperty *topic_property_;

private:
  QString getDisplayName(const std::string &layerClass);
  QString getTopicName(const std::string &layerClass, const std::string &layerName);
  QString getTopicType(const std::string &layerClass);
};

} // namespace hypermap