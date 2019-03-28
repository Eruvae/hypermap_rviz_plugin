#include "hypermapdisplay.h"

#include <pluginlib/class_list_macros.h>
#include <iostream>
#include <OgreImage.h>
#include <OgreManualObject.h>

PLUGINLIB_EXPORT_CLASS(hypermap::HypermapDisplay, rviz::Display)

namespace hypermap
{

HypermapDisplay::HypermapDisplay() : rviz::Display()
{
    topic_property_ = new rviz::RosTopicProperty("Topic", "", ros::message_traits::datatype<hypermap_msgs::HypermapMetaData>(),
                                                 "hypermap_msgs::HypermapMetaData topic to subscribe to.", this, SLOT(updateTopic()));
    layerCnt_ = new rviz::IntProperty("Layer count", 3, "Number of layers", this, SLOT(updateLayerProps()));
    oldLayerCnt = 3;
    layerCnt_->setMin(0);
    layerCnt_->setMax(10);
    enableLayer_ = new rviz::BoolProperty*[10];
    for (int i = 0; i < 3; i++)
    {
        QString label = "Layer " + QString::number(i);
        //std::cout << label.toStdString() << std::endl;
        enableLayer_[i] = new rviz::BoolProperty(label, true, "", layerCnt_);
    }
    enable_bg_property_ = new rviz::BoolProperty("Enable background", true, "Display background of map", this);
    //Display *disp = this->createDisplay("ImageDisplay");
    //this->addDisplay(disp);
}

void HypermapDisplay::setTopic(const QString &topic, const QString &datatype)
{
    topic_property_->setString(topic);
}

void HypermapDisplay::updateTopic()
{
    map_sub_.shutdown();
    map_sub_ = update_nh_.subscribe(topic_property_->getTopicStd(), 1, &HypermapDisplay::receiveMapMeta, this);
}

void HypermapDisplay::updateLayerProps()
{
    /*if (layerCnt_->getInt() > oldLayerCnt)
    {
        for (int i = oldLayerCnt; i < layerCnt_->getInt(); i++)
        {
            QString label = "Layer " + QString::number(i);
            //std::cout << label.toStdString() << std::endl;
            enableLayer_[i] = new rviz::BoolProperty(label, true, "", layerCnt_);
        }
    }
    else if (layerCnt_->getInt() > oldLayerCnt)
    {
        for (int i = layerCnt_->getInt(); i < oldLayerCnt; i++)
        {
            //enableLayer_[i]->
            //delete enableLayer_[i];
        }
    }
    oldLayerCnt = layerCnt_->getInt();*/
}

void HypermapDisplay::receiveMapMeta(const hypermap_msgs::HypermapMetaData::ConstPtr& msg)
{
    current_map_meta_ = *msg;
}

/*void HypermapDisplay::onInitialize()
{
    Display::onInitialize();
}

void HypermapDisplay::fixedFrameChanged()
{
    Display::fixedFrameChanged();
}

void HypermapDisplay::reset()
{
    Display::reset();
}

void HypermapDisplay::updateMap()
{

}*/

}
