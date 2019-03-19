#include "semanticmapdisplay.h"

#include <random>
#include <Ogre.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
//#include <rviz/view_manager.h>
//#include <rviz/tool_manager.h>
#include "geometry_msgs/Point32.h"
#include "earcut.hpp"
#include "glasbey.h"

#include "movable_text.h"

namespace mapbox {
namespace util {

template <>
struct nth<0, geometry_msgs::Point32> {
    inline static geometry_msgs::Point32::_x_type get(const geometry_msgs::Point32 &t) {
        return t.x;
    }
};

template <>
struct nth<1, geometry_msgs::Point32> {
    inline static geometry_msgs::Point32::_y_type get(const geometry_msgs::Point32 &t) {
        return t.y;
    }
};

} // namespace util
} // namespace mapbox

#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(hypermap::SemanticMapDisplay, rviz::Display)

namespace hypermap
{

SemanticMapDisplay::SemanticMapDisplay() : rviz::Display()
{
    topic_property_ = new rviz::RosTopicProperty("Topic", "", ros::message_traits::datatype<hypermap_msgs::SemanticMap>(),
                                                 "hypermap_msgs::SemanticMap topic to subscribe to.", this, SLOT(updateTopic()));

    show_polygons_property_ = new rviz::BoolProperty("Show shapes", true, "Display shapes of semantic objects", this);
    show_labels_property_ = new rviz::BoolProperty("Show labels", true, "Display names of semantic objects", this);

    connect(this, SIGNAL(mapReceived()), this, SLOT(updateVisual()));
    connect(show_polygons_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
    connect(show_labels_property_, SIGNAL(changed()), this, SLOT(updateVisual()));
}

void SemanticMapDisplay::setTopic(const QString &topic, const QString &datatype)
{
    topic_property_->setString(topic);
}

void SemanticMapDisplay::updateTopic()
{
    map_sub_.shutdown();
    map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1, &SemanticMapDisplay::receiveMap, this);

    /*Ogre::ManualObject *mo = scene_manager_->createManualObject("test");

    mo->begin("BaseWhite", Ogre::RenderOperation::OT_TRIANGLE_FAN);
    mo->position(0,0,0);
    mo->position(1,0,0);
    mo->position(1,1,0);
    mo->position(0,1,0);
    mo->end();

    scene_node_->attachObject(mo);*/

    /*Ogre::Polygon *po = new Ogre::Polygon();

    po->insertVertex(Ogre::Vector3(0, 0, 0));
    po->insertVertex(Ogre::Vector3(0, 2, 0));
    po->insertVertex(Ogre::Vector3(2, 2, 0));
    po->insertVertex(Ogre::Vector3(2, 0, 0));

    scene_node_->addChild(po);*/
}

void SemanticMapDisplay::updateTransform()
{
    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(current_map_.header.frame_id, ros::Time(0), position, orientation))
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", current_map_.header.frame_id.c_str(), qPrintable(fixed_frame_));
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);
}

void SemanticMapDisplay::updateVisual()
{
    scene_node_->detachAllObjects();
    scene_node_->removeAndDestroyAllChildren();

    /*Ogre::ManualObject *mob = scene_manager_->createManualObject("test");

    mob->begin("BaseWhite", Ogre::RenderOperation::OT_TRIANGLE_FAN);
    mob->position(0,0,0);
    mob->position(1,0,0);
    mob->position(1,1,0);
    mob->position(0,1,0);
    mob->end();

    scene_node_->attachObject(mob);*/

    /*hypermap::MovableText *testTxt = new hypermap::MovableText("test text", "Liberation Sans", 0.3);
    testTxt->setTextAlignment(hypermap::MovableText::H_CENTER, hypermap::MovableText::V_CENTER);
    testTxt->setGlobalTranslation(Ogre::Vector3(3, 4, 5));
    scene_node_->attachObject(testTxt);*/

    updateTransform();

    //std::default_random_engine generator;
    //std::uniform_real_distribution<float> distribution(0.0,1.0);

    uint8_t cind = 2;

    for (const auto &obj : current_map_.objects)
    {
        if (show_polygons_property_->getBool())
        {
            std::vector<std::vector<geometry_msgs::Point32>> pg;
            pg.push_back(obj.shape.points);
            std::vector<uint32_t> indices = mapbox::earcut(pg);
            ROS_INFO_STREAM("Inds : " << indices.size());
            Ogre::ManualObject *mo = scene_manager_->createManualObject();
            Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);
            cind++;

            mo->estimateVertexCount(obj.shape.points.size());
            mo->estimateIndexCount(indices.size());
            mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
            for (const auto &point : obj.shape.points)
            {
                mo->position(point.x, point.y, 0);
                mo->colour(col);
                ROS_INFO_STREAM("Point added: " <<  point.x << point.y);
            }
            for (uint32_t ind : indices)
            {
                mo->index(ind);
            }
            mo->end();
            scene_node_->attachObject(mo);
        }

        if (show_labels_property_->getBool())
        {
            //Ogre::ColourValue col(glasbey[cind][0] / 255.0, glasbey[cind][1] / 255.0, glasbey[cind][2] / 255.0);
            //cind++;
            hypermap::MovableText *mo_txt = new hypermap::MovableText(obj.name, "Liberation Sans", 0.3/*, col*/);
            mo_txt->setTextAlignment(hypermap::MovableText::H_CENTER, hypermap::MovableText::V_CENTER);
            mo_txt->setLocalTranslation(Ogre::Vector3(obj.position.x, obj.position.y, 0));
            mo_txt->showOnTop();
            scene_node_->attachObject(mo_txt);
        }
    }
}

void SemanticMapDisplay::fixedFrameChanged()
{
    updateTransform();
}

void SemanticMapDisplay::receiveMap(const hypermap_msgs::SemanticMap::ConstPtr& msg)
{
    current_map_ = *msg;

    Q_EMIT mapReceived();
}

}
