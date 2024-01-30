#pragma once

#include <map>
#include <string>

#include <rviz_common/display.hpp>
#include <rviz_common/properties/ros_topic_property.hpp>
#include <rviz_common/properties/bool_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/status_property.hpp>
#include <hypermap_msgs/msg/semantic_map.hpp>

namespace hypermap {

class SemanticMapDisplay : public rviz_common::Display
{
Q_OBJECT
public:
  SemanticMapDisplay();

  virtual void setTopic(const QString &topic, const QString &datatype);
  virtual void fixedFrameChanged();
  virtual void onEnable();
  virtual void onDisable();

Q_SIGNALS:
  void mapReceived();

protected Q_SLOTS:
  void updateTopic();
  void updateVisual();

protected:
  void receiveMap(const hypermap_msgs::msg::SemanticMap::ConstSharedPtr &msg);
  void updateTransform();
  void subscribe();
  void unsubscribe();
  void clearVisual();

  rviz_common::properties::RosTopicProperty *topic_property_;
  rviz_common::properties::BoolProperty *show_polygons_property_;
  rviz_common::properties::BoolProperty *show_labels_property_;
  rviz_common::properties::FloatProperty *char_height_property_;

  rviz_common::properties::Property *show_classes_property_;
  std::map<std::string, int> class_list_;

  hypermap_msgs::msg::SemanticMap current_map_;
  rclcpp::Subscription<hypermap_msgs::msg::SemanticMap>::SharedPtr map_sub_;
  bool loaded_;
};

} // namespace hypermap
