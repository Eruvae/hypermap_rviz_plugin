#include "hypermap_rviz_plugin/semanticmapdisplay.h"

#include <random>
#include <Ogre.h>
#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/frame_manager_iface.hpp>
//#include <rviz_common/view_manager.h>
//#include <rviz_common/tool_manager.h>
#include <geometry_msgs/msg/point32.hpp>

#include "hypermap_rviz_plugin/earcut.hpp"
#include "hypermap_rviz_plugin/glasbey.h"
#include "hypermap_rviz_plugin/movable_text.hpp"

namespace mapbox {
namespace util {

template <>
struct nth<0, geometry_msgs::msg::Point32> {
    inline static geometry_msgs::msg::Point32::_x_type get(const geometry_msgs::msg::Point32 &t) {
        return t.x;
    }
};

template <>
struct nth<1, geometry_msgs::msg::Point32> {
    inline static geometry_msgs::msg::Point32::_y_type get(const geometry_msgs::msg::Point32 &t) {
        return t.y;
    }
};

} // namespace util
} // namespace mapbox

namespace hypermap
{

SemanticMapDisplay::SemanticMapDisplay() : rviz_common::MessageFilterDisplay<hypermap_msgs::msg::SemanticMap>(), loaded_(false)
{
    show_polygons_property_ = new rviz_common::properties::BoolProperty("Show shapes", true, "Display shapes of semantic objects", this);
    show_labels_property_ = new rviz_common::properties::BoolProperty("Show labels", true, "Display names of semantic objects", this);
    char_height_property_ = new rviz_common::properties::FloatProperty("Char height", 0.3, "Char height for labels", this);
    show_classes_property_ = new rviz_common::properties::Property("Select classes", QVariant(), "Change which classes are shown", this);

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateVisual()));
    connect(show_polygons_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(show_labels_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(char_height_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
}

void SemanticMapDisplay::reset()
{
    rviz_common::MessageFilterDisplay<hypermap_msgs::msg::SemanticMap>::reset();
    clearVisual();
}

void SemanticMapDisplay::clearVisual()
{
    scene_node_->detachAllObjects();
    scene_node_->removeAndDestroyAllChildren();
}

void SemanticMapDisplay::updateVisual()
{
    clearVisual();

    if (!loaded_)
    {
        return;
    }

    for (const auto &obj : current_map_.objects)
    {
        auto c_it = class_list_.find(obj.name);
        if (c_it == class_list_.end())
        {
            rviz_common::properties::BoolProperty *prop = new rviz_common::properties::BoolProperty(QString::fromStdString(obj.name), true, "Show class", show_classes_property_);
            std::tie(c_it, std::ignore) = class_list_.insert(std::make_pair(obj.name, class_list_.size()));
            connect(prop, SIGNAL(changed()), this, SLOT(updateVisual()));
        }
        else
        {
            rviz_common::properties::BoolProperty *prop = (BoolProperty*) show_classes_property_->childAt(c_it->second);
            if (!prop->getBool())
                continue;
        }

        if (show_polygons_property_->getBool())
        {
            std::vector<std::vector<geometry_msgs::msg::Point32>> pg;
            pg.push_back(obj.shape.points);
            std::vector<uint32_t> indices = mapbox::earcut(pg);
            //RCLCPP_INFO(node->get_logger(), "Inds: %zu", indices.size());
            Ogre::ManualObject *mo = scene_manager_->createManualObject();
            //Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);
            //cind = (cind + 1) % 256;
            uint8_t cind = (2 + c_it->second) % 256;
            Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);

            mo->estimateVertexCount(obj.shape.points.size());
            mo->estimateIndexCount(indices.size());
            mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            bool shape_valid = true;
            for (const auto &point : obj.shape.points)
            {
                if (!std::isfinite(point.x) || !std::isfinite(point.y))
                {
                    //ROS_WARN_STREAM("Invalid shape detected");
                    shape_valid = false;
                    break;
                }
                mo->position(point.x, point.y, 0);
                mo->colour(col);
                //ROS_INFO_STREAM("Point added: " <<  point.x << ", " << point.y);
            }
            if (!shape_valid)
            {
                delete mo;
                continue;
            }

            for (uint32_t ind : indices)
            {
                mo->index(ind);
            }
            mo->end();
            //double mo_prio = 1.0 / mo->getBoundingRadius();
            //ROS_INFO_STREAM("Prio: " << mo_prio);
            //ushort mo_prio_sh = (ushort)(mo_prio * 1000);
            //mo->setRenderQueueGroupAndPriority(0, mo_prio_sh);
            scene_node_->attachObject(mo);
        }

        if (show_labels_property_->getBool())
        {
            if (!std::isfinite(obj.position.x) || !std::isfinite(obj.position.y))
            {
                //ROS_WARN_STREAM("Invalid position detected");
                continue;
            }
            //Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);
            //cind++;
            hypermap::MovableText *mo_txt = new hypermap::MovableText(obj.name, "Liberation Sans", char_height_property_->getFloat()/*, col*/);
            mo_txt->setTextAlignment(hypermap::MovableText::H_CENTER, hypermap::MovableText::V_CENTER);
            mo_txt->setLocalTranslation(Ogre::Vector3(obj.position.x, obj.position.y, 0));
            mo_txt->showOnTop();
            //mo_txt->setRenderQueueGroup(1);
            scene_node_->attachObject(mo_txt);
        }
    }
}

void SemanticMapDisplay::processMessage(hypermap_msgs::msg::SemanticMap::ConstSharedPtr msg)
{
    current_map_ = *msg;
    loaded_ = true;
    //setStatus(rviz_common::properties::StatusProperty::Ok, "Message", "Map received");
    rclcpp::Time msg_time(msg->header.stamp, RCL_ROS_TIME);
    if (!updateFrame(msg->header.frame_id, msg_time)) {
        setMissingTransformToFixedFrame(msg->header.frame_id);
        return;
    }
    setTransformOk();
    Q_EMIT mapReceived();
}

} // namespace hypermap

PLUGINLIB_EXPORT_CLASS(hypermap::SemanticMapDisplay, rviz_common::Display)