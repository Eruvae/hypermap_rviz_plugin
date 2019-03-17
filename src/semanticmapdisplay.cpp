#include "semanticmapdisplay.h"

#include <random>
#include <Ogre.h>
#include <rviz/display_context.h>
#include <rviz/frame_manager.h>
#include "geometry_msgs/Point32.h"
#include "earcut.hpp"

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

void SemanticMapDisplay::receiveMap(const hypermap_msgs::SemanticMap::ConstPtr& msg)
{
    current_map_ = *msg;

    /*Ogre::ManualObject *mob = scene_manager_->createManualObject("test");

    mob->begin("BaseWhite", Ogre::RenderOperation::OT_TRIANGLE_FAN);
    mob->position(0,0,0);
    mob->position(1,0,0);
    mob->position(1,1,0);
    mob->position(0,1,0);
    mob->end();

    scene_node_->attachObject(mob);*/

    Ogre::Vector3 position;
    Ogre::Quaternion orientation;
    if(!context_->getFrameManager()->getTransform(msg->header, position, orientation))
    {
        ROS_DEBUG("Error transforming from frame '%s' to frame '%s'", msg->header.frame_id.c_str(), qPrintable( fixed_frame_ ));
    }

    scene_node_->setPosition(position);
    scene_node_->setOrientation(orientation);

    std::default_random_engine generator;
    std::uniform_real_distribution<float> distribution(0.0,1.0);

    for (const auto &obj : current_map_.objects)
    {
        std::vector<std::vector<geometry_msgs::Point32>> pg;
        pg.push_back(obj.shape.points);
        std::vector<uint32_t> indices = mapbox::earcut(pg);
        ROS_INFO_STREAM("Inds : " << indices.size());
        Ogre::ManualObject *mo = scene_manager_->createManualObject(obj.name);
        Ogre::ColourValue col(distribution(generator), distribution(generator), distribution(generator));
        mo->estimateVertexCount(obj.shape.points.size());
        mo->estimateIndexCount(indices.size());
        mo->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_TRIANGLE_LIST);
        for (size_t i = 0; i < obj.shape.points.size(); i++)
        {
            const auto &point = obj.shape.points[i];
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
}

}
