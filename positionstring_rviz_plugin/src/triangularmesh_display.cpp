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

#include "triangularmesh_display.h"
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

namespace positionstring_rviz_plugin
{

TriangularMeshDisplay::TriangularMeshDisplay(const std::string & name, rviz::VisualizationManager * manager)
  : Display(name, manager)
  , color_(0.1f, 1.0f, 0.0f)
  , tf_filter_(*manager->getTFClient(), "", 2, update_nh_)
  , count (0)
{
  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();

  static int count = 0;
  std::stringstream ss;
  ss << "TriangularMesh " << count++;
/////////////////////////////
//  text_ = new ogre_tools::MovableText("positionstring", "Arial", character_height_, Ogre::ColourValue::White);
//  text_->showOnTop (true);
//  scene_node_->attachObject(mesh_);
/////////////////////////////
  tf_filter_.connectInput(sub_);
  tf_filter_.registerCallback(boost::bind(&TriangularMeshDisplay::incomingMessage, this, _1));
}

TriangularMeshDisplay::~TriangularMeshDisplay()
{
  unsubscribe();
//////////////////////////////
//  delete text_;
//////////////////////////////
}


void TriangularMeshDisplay::setTopic(const std::string & topic)
{
  unsubscribe();
  topic_ = topic;
  subscribe();

  propertyChanged(topic_property_);

  causeRender();
}

void TriangularMeshDisplay::setColor(const rviz::Color & color)
{
  color_ = color;

  propertyChanged(color_property_);
  processMessage(current_message_);
  causeRender();
}

void TriangularMeshDisplay::subscribe()
{
  if (!isEnabled())
    return;

  if (!topic_.empty())
  {
    sub_.subscribe(update_nh_, topic_, 1);
  }
}

void TriangularMeshDisplay::unsubscribe()
{
  sub_.unsubscribe();
}

void TriangularMeshDisplay::onEnable()
{
  scene_node_->setVisible(true);
  subscribe();
}

void TriangularMeshDisplay::onDisable()
{
  unsubscribe();
  scene_node_->setVisible(false);
}

void TriangularMeshDisplay::fixedFrameChanged()
{
  tf_filter_.setTargetFrame(fixed_frame_);
}

void TriangularMeshDisplay::update(float wall_dt, float ros_dt)
{
}

void TriangularMeshDisplay::processMessage(const ias_table_msgs::TriangularMesh::ConstPtr& msg)
{
  if (!msg)
  {
    return;
  }
  if (count++ > 0)
  {
//    delete vbuf.get();
//    delete data;
//    delete ibuf;
    scene_manager_->destroyEntity(entity_);
    Ogre::MeshManager::getSingleton().unload ("triangularmesh");
    Ogre::MeshManager::getSingleton().remove ("triangularmesh");
  }

  mesh_ = Ogre::MeshManager::getSingleton().createManual ("triangularmesh", "General");
  submesh_ = mesh_->createSubMesh();

ROS_INFO ("a %i", __LINE__);
  // We first create a VertexData
  data = new Ogre::VertexData();
ROS_INFO ("a %i", __LINE__);
  // Then, we link it to our Mesh/SubMesh :
  mesh_->sharedVertexData = data;
  // submesh_->useSharedVertices = false; // This value is 'true' by default
ROS_INFO ("a %i", __LINE__);
  // submesh_->vertexData = data;
  // We have to provide the number of verteices we'll put into this Mesh/SubMesh
  data->vertexCount = msg->points.size();
ROS_INFO ("a %i", __LINE__);
  // Then we can create our VertexDeclaration
  Ogre::VertexDeclaration* decl = data->vertexDeclaration;
ROS_INFO ("a %i", __LINE__);
  decl->addElement (0,0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
ROS_INFO ("a %i", __LINE__);
  /** @todo: should add more elements, specifically normal vectors! */
  
  // create a vertex buffer
  vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    decl->getVertexSize(0),                     // This value is the size of a vertex in memory
    msg->points.size(),                         // The number of vertices you'll put into this buffer
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
  );
 
ROS_INFO ("a %i", __LINE__);
  double min_x=FLT_MAX,  min_y=FLT_MAX,  min_z=FLT_MAX;
  double max_x=-FLT_MAX, max_y=-FLT_MAX, max_z=-FLT_MAX;
  float* array = (float*) malloc (sizeof(float)*3*msg->points.size());
ROS_INFO ("a %i", __LINE__);
  for (unsigned int i = 0; i < msg->points.size(); i++)
  {
ROS_INFO ("a %i", __LINE__);
    array[i*3+0] = msg->points[i].x;
    array[i*3+1] = msg->points[i].y;
    array[i*3+2] = msg->points[i].z;
ROS_INFO ("a %i", __LINE__);
    if (msg->points[i].x < min_x) min_x = msg->points[i].x;
    if (msg->points[i].y < min_y) min_y = msg->points[i].y;
    if (msg->points[i].z < min_z) min_z = msg->points[i].z;
    if (msg->points[i].x > max_x) max_x = msg->points[i].x;
    if (msg->points[i].y > max_y) max_y = msg->points[i].y;
    if (msg->points[i].z > max_z) max_z = msg->points[i].z;
  }
ROS_INFO ("a %i", __LINE__);
  
  vbuf->writeData (0, vbuf->getSizeInBytes(), array, true); 
  free (array);
  
  Ogre::VertexBufferBinding* bind = data->vertexBufferBinding;
  bind->setBinding (0, vbuf);
  
  // create index buffer
  ibuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
    Ogre::HardwareIndexBuffer::IT_16BIT,        // You can use several different value types here
    msg->triangles.size()*3,                    // The number of indices you'll put in that buffer
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
  );
  
ROS_INFO ("a %i", __LINE__);
  submesh_->indexData->indexBuffer = ibuf;     // The pointer to the index buffer
  submesh_->indexData->indexCount = msg->triangles.size()*3*2; // The number of indices we'll use
  submesh_->indexData->indexStart = 0;

ROS_INFO ("a %i", __LINE__);
  unsigned short *index = (unsigned short*) malloc (sizeof(unsigned short)*3*msg->triangles.size()*2);
  for (unsigned int i = 0; i < msg->triangles.size(); i++)
  {
ROS_INFO ("a %i", __LINE__);
    index[i*6+0] = msg->triangles[i].i.data;
    index[i*6+1] = msg->triangles[i].j.data;
    index[i*6+2] = msg->triangles[i].k.data;
    index[i*6+3] = msg->triangles[i].i.data;
    index[i*6+4] = msg->triangles[i].k.data;
    index[i*6+5] = msg->triangles[i].j.data;
  }

ROS_INFO ("a %i", __LINE__);
  ibuf->writeData (0, ibuf->getSizeInBytes(), index, true); 
  free (index);
 
  mesh_->_setBounds (Ogre::AxisAlignedBox (min_x, min_y, min_z, max_x, max_y, max_z));
ROS_INFO ("a %i", __LINE__);
  mesh_->_setBoundingSphereRadius (std::max(max_x-min_x, std::max(max_y-min_y, max_z-min_z))/2.0f);

  mesh_->load ();
ROS_INFO ("a %i", __LINE__);

//  MaterialPtr material = MaterialManager::getSingleton().create(
//    "Test/ColourTest", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//  material->getTechnique(0)->getPass(0)->setVertexColourTracking(TVC_AMBIENT);

  std::stringstream ss;
  ss << "tm" << count;
  entity_ = scene_manager_->createEntity(ss.str(), "triangularmesh");

ROS_INFO ("a %i", __LINE__);
  //thisEntity->setMaterialName("Test/ColourTest");
//  scene_node_ = scene_manager_->getRootSceneNode()->createChildSceneNode();
//ROS_INFO ("a %i", __LINE__);
//  thisSceneNode->setPosition(0, 0, 0);
ROS_INFO ("a %i", __LINE__);
  scene_node_->attachObject(entity_);
ROS_INFO ("a %i", __LINE__);

/////////////////////////////////
//  text_->setCaption (msg->text);
//  text_->setColor (Ogre::ColourValue(color_.r_, color_.g_, color_.b_));
  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(1.0f, 0.0f, 0.0f, 0.0f),
      btVector3(0, 0, 0)), msg->header.stamp,
      msg->header.frame_id);

  try
  {
    vis_manager_->getTFClient()->transformPose(fixed_frame_, pose, pose);
  }
  catch (tf::TransformException & e)
  {
    ROS_ERROR("Error transforming from frame '%s' to frame '%s'",
        msg->header.frame_id.c_str(), fixed_frame_.c_str());
  }

  Ogre::Vector3 position = Ogre::Vector3(pose.getOrigin().x(),
      pose.getOrigin().y(), pose.getOrigin().z());

  rviz::robotToOgre(position);

  scene_node_->setPosition(position);

///////////////////////////////////
}

void TriangularMeshDisplay::incomingMessage(const ias_table_msgs::TriangularMesh::ConstPtr& message)
{
  current_message_ = message;
  processMessage(message);
}

void TriangularMeshDisplay::reset()
{
}

void TriangularMeshDisplay::targetFrameChanged()
{
}

void TriangularMeshDisplay::createProperties()
{
  ROS_INFO ("creating properties.");
  color_property_ = property_manager_->createProperty<rviz::ColorProperty> 
              ("Color", property_prefix_, boost::bind(&TriangularMeshDisplay::getColor, this),
              boost::bind(&TriangularMeshDisplay::setColor, this, _1), parent_category_, this);
  topic_property_ = property_manager_->createProperty<rviz::ROSTopicStringProperty> 
              ("Topic", property_prefix_, boost::bind(&TriangularMeshDisplay::getTopic, this),
              boost::bind(&TriangularMeshDisplay::setTopic, this, _1), parent_category_, this);
  rviz::ROSTopicStringPropertyPtr topic_prop = topic_property_.lock();
  topic_prop->setMessageType(ias_table_msgs::TriangularMesh::__s_getDataType());
}

} // namespace mapping_rviz_plugin

