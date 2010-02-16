/* 
 * Copyright (c) 2010, Nico Blodow (blodow@cs.tum.edu)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "positionstring_display.h"
#include "rviz/visualization_manager.h"
#include "rviz/properties/property.h"
#include "rviz/properties/property_manager.h"
#include "rviz/common.h"

#include <tf/transform_listener.h>

#include <boost/bind.hpp>

#include <OGRE/OgreSceneNode.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreBillboardSet.h>

#include <ogre_tools/movable_text.h>

namespace positionstring_rviz_plugin
{

PositionStringDisplay::PositionStringDisplay(const std::string & name, rviz::VisualizationManager * manager)
: Display(name, manager)
, color_(0.1f, 1.0f, 0.0f)
, override_color_(false)
, tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "PositionString " << count++;
  manual_object_ = scene_manager_->createManualObject(ss.str());
  manual_object_->setDynamic(true);
  scene_node_->attachObject(manual_object_);

  text_ = new ogre_tools::MovableText("positionstring", "Arial",0.1, Ogre::ColourValue::White);
  //text_ = new ogre_tools::MovableText("positionstring");
  scene_node_->attachObject(text_);

  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&PositionStringDisplay::incomingMessage, this, _1));
}

PositionStringDisplay::~PositionStringDisplay()
{
  unsubscribe();
  clear();

  scene_manager_->destroyManualObject(manual_object_);

  delete text_;
}

void PositionStringDisplay::clear()
{
  manual_object_->clear();
}

void PositionStringDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void PositionStringDisplay::setColor(const rviz::Color & color)
{
  color_ = color;

  propertyChanged(color_property_);
  processMessage(current_message_);
  causeRender();
}

void PositionStringDisplay::setOverrideColor(bool override)
{
  override_color_ = override;

  propertyChanged(override_color_property_);

  processMessage(current_message_);
  causeRender();
}

void PositionStringDisplay::subscribe()
{
  if (!isEnabled())
    return;

  if (!topic_.empty())
  {
    sub_.subscribe(update_nh_, topic_, 1);
  }
}

void PositionStringDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void PositionStringDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void PositionStringDisplay::onDisable()
{
  unsubscribe();
  clear();
  scene_node_->setVisible(false);
}

void PositionStringDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame(fixed_frame_);
  clear();
}

void PositionStringDisplay::update(float wall_dt, float ros_dt)
{
}

void PositionStringDisplay::processMessage(const ias_visualization_msgs::PositionString::ConstPtr& msg)
{
  clear();

  if (!msg)
  {
    return;
  }
  
  text_->setCaption (msg->text);

  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0.0f, 0.0f, 0.0f),
      btVector3(0.0f, 0.0f, 0.0f)), msg->header.stamp,
      msg->header.frame_id);

  try
  {
    vis_manager_->getTFClient()->transformPose(fixed_frame_, pose, pose);
  }
  catch (tf::TransformException & e)
  {
    ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
        msg->header.frame_id, fixed_frame_.c_str());
  }

  Ogre::Vector3 position = Ogre::Vector3(pose.getOrigin().x(),
      pose.getOrigin().y(), pose.getOrigin().z());
  rviz::robotToOgre(position);

  btScalar yaw, pitch, roll;
  pose.getBasis().getEulerZYX(yaw, pitch, roll);

  Ogre::Matrix3 orientation(rviz::ogreMatrixFromRobotEulers(yaw, pitch, roll));

  manual_object_->clear();

  Ogre::ColourValue color;

//  // If we render points, we don't care about the order
//  if (render_operation_ == collision_render_ops::CPoints)
//  {
//    typedef std::vector<ogre_tools::PointCloud::Point> V_Point;
//    V_Point points;
//    if (num_boxes > 0)
//    {
//      points.resize(num_boxes);
//      for (uint32_t i = 0; i < num_boxes; i++)
//      {
//        ogre_tools::PointCloud::Point & current_point = points[i];
//
//        current_point.x = msg->boxes[i].center.x;
//        current_point.y = msg->boxes[i].center.y;
//        current_point.z = msg->boxes[i].center.z;
//        color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
//        current_point.setColor(color.r, color.g, color.b);
//      }
//    }
//    cloud_->clear();
//
//    if (!points.empty())
//    {
//      cloud_->addPoints(&points.front(), points.size());
//    }
//  }
//  else
//  {
//    geometry_msgs::Point32 center, extents;
//    color = Ogre::ColourValue(color_.r_, color_.g_, color_.b_, alpha_);
//    if (num_boxes > 0)
//    {
//      for (uint32_t i = 0; i < num_boxes; i++)
//      {
//        manual_object_->estimateVertexCount(8);
//        manual_object_->begin("BaseWhiteNoLighting",
//            Ogre::RenderOperation::OT_LINE_STRIP);
//        center.x = msg->boxes[i].center.x;
//        center.y = msg->boxes[i].center.y;
//        center.z = msg->boxes[i].center.z;
//        extents.x = msg->boxes[i].extents.x;
//        extents.y = msg->boxes[i].extents.y;
//        extents.z = msg->boxes[i].extents.z;
//
//        manual_object_->position(center.x - extents.x, center.y - extents.y,
//            center.z - extents.z);
//        manual_object_->colour(color);
//        manual_object_->position(center.x - extents.x, center.y + extents.y,
//            center.z - extents.z);
//        manual_object_->colour(color);
//        manual_object_->position(center.x + extents.x, center.y + extents.y,
//            center.z - extents.z);
//        manual_object_->colour(color);
//        manual_object_->position(center.x + extents.x, center.y - extents.y,
//            center.z - extents.z);
//        manual_object_->colour(color);
//        manual_object_->position(center.x + extents.x, center.y - extents.y,
//            center.z + extents.z);
//        manual_object_->colour(color);
//        manual_object_->position(center.x + extents.x, center.y + extents.y,
//            center.z + extents.z);
//        manual_object_->colour(color);
//        manual_object_->position(center.x - extents.x, center.y + extents.y,
//            center.z + extents.z);
//        manual_object_->colour(color);
//        manual_object_->position(center.x - extents.x, center.y - extents.y,
//            center.z + extents.z);
//        manual_object_->colour(color);
//        manual_object_->end();
//      }
//    }
//  }
//
  scene_node_->setPosition(position);
  scene_node_->setOrientation(orientation);
}

void PositionStringDisplay::incomingMessage(const ias_visualization_msgs::PositionString::ConstPtr& message)
{
  current_message_ = message;
  processMessage(message);
}

void PositionStringDisplay::reset()
{
  clear();
}

void PositionStringDisplay::targetFrameChanged()
{
}

void PositionStringDisplay::createProperties()
{
  override_color_property_ = property_manager_->createProperty<rviz::BoolProperty> ("Override Color", property_prefix_,
                                                                                    boost::bind(&PositionStringDisplay::getOverrideColor, this),
                                                                                    boost::bind(&PositionStringDisplay::setOverrideColor, this, _1), parent_category_,
                                                                                    this);
  color_property_ = property_manager_->createProperty<rviz::ColorProperty> ("Color", property_prefix_, boost::bind(&PositionStringDisplay::getColor, this),
                                                                            boost::bind(&PositionStringDisplay::setColor, this, _1), parent_category_, this);
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> ("Topic", property_prefix_, boost::bind(&PositionStringDisplay::getTopic, this),
                                                                               boost::bind(&PositionStringDisplay::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ias_visualization_msgs::PositionString::__s_getDataType());
}

} // namespace mapping_rviz_plugin
