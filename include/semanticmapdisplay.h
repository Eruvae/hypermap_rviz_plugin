#ifndef SEMANTICMAPDISPLAY_H
#define SEMANTICMAPDISPLAY_H

#include "rviz/display.h"
#include "rviz/properties/ros_topic_property.h"
#include "hypermap_msgs/SemanticMap.h"

namespace hypermap {

class SemanticMapDisplay : public rviz::Display
{
Q_OBJECT
public:
  SemanticMapDisplay();

  virtual void setTopic(const QString &topic, const QString &datatype);
  virtual void fixedFrameChanged();

Q_SIGNALS:
  void mapReceived();

protected Q_SLOTS:
  void updateTopic();
  void updateVisual();

protected:
  void receiveMap(const hypermap_msgs::SemanticMap::ConstPtr& msg);
  void updateTransform();

  rviz::RosTopicProperty *topic_property_;
  rviz::BoolProperty *show_polygons_property_;
  rviz::BoolProperty *show_labels_property_;

  hypermap_msgs::SemanticMap current_map_;
  ros::Subscriber map_sub_;
};

}

#endif // SEMANTICMAPDISPLAY_H
