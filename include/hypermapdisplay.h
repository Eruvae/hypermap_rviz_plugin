#ifndef HYPERMAPDISPLAY_H
#define HYPERMAPDISPLAY_H

#include "rviz/display.h"
#include "rviz/display_group.h"
#include "rviz/default_plugin/image_display.h"
#include "rviz/default_plugin/map_display.h"
#include "hypermap_msgs/HypermapImage.h"

namespace hypermap {

class HypermapDisplay : public rviz::Display
{
Q_OBJECT
public:
  HypermapDisplay();

  // Overrides from Display
  //virtual void onInitialize();
  //virtual void fixedFrameChanged();
  //virtual void reset();
  virtual void setTopic(const QString &topic, const QString &datatype);

protected Q_SLOTS:
  void updateTopic();
  void updateLayerProps();
  //void updateMap();
  //* If this is true, will disable it's children when it's own bool value is false */

protected:
  void receiveMap(const hypermap_msgs::HypermapImage::ConstPtr& msg);

  hypermap_msgs::HypermapImage current_map_;
  ros::Subscriber map_sub_;

  rviz::RosTopicProperty *topic_property_;
  rviz::IntProperty *layerCnt_;
  rviz::BoolProperty **enableLayer_;
  rviz::BoolProperty *enable_bg_property_;

private:
  int oldLayerCnt;
};

}
#endif // HYPERMAPDISPLAY_H
