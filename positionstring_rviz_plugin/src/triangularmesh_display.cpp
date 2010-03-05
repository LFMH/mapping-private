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

typedef struct {int a,b,c;} triangle;
////////////////////////////////////////////////////////////////////////////////
void write_vtk_file(std::string output, std::vector<triangle> triangles, std::vector<geometry_msgs::Point32> points, 
                                             int nr_tr)
{
  /* writing VTK file */
  
  FILE *f;
  
  f = fopen(output.c_str(),"w");
  
  fprintf (f, "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS %ld float\n",points.size());
  unsigned long i;
  
  for (i=0; i<points.size(); i++)
  {
    fprintf (f,"%f %f %f ", points[i].x, points[i].y,
               points[i].z);
    fprintf (f,"\n");
  }
  
  fprintf(f,"VERTICES %ld %ld\n", points.size(), 2*points.size());
  for (i=0; i<points.size(); i++)
    fprintf(f,"1 %ld\n", i);
    
  ROS_INFO("vector: %ld, nr: %d  ", triangles.size(), nr_tr);
  
  fprintf(f,"\nPOLYGONS %d %d\n", nr_tr, 4*nr_tr);
  for (int i=0; i<nr_tr; i++)
  {
    if ((unsigned long)triangles[i].a >= points.size() || triangles[i].a < 0 || isnan(triangles[i].a))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, points.size());
    else if ((unsigned long)triangles[i].b >= points.size() || triangles[i].b < 0 || isnan(triangles[i].b))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, points.size());
    else if ((unsigned long)triangles[i].c >= points.size() || triangles[i].c < 0 || isnan(triangles[i].c))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, points.size());
    else if (triangles[i].a == triangles[i].b || triangles[i].a == triangles[i].c || triangles[i].b == triangles[i].c)
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, points.size());
    else
      fprintf(f,"3 %d %d %d\n",triangles[i].a, triangles[i].c, triangles[i].b);
  }
}

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
//  std::vector<triangle> triangles;
//  for (unsigned int i = 0; i < msg->triangles.size(); i++)
//  {
//    triangle tr;
//    tr.a = msg->triangles[i].i;
//    tr.b = msg->triangles[i].j;
//    tr.c = msg->triangles[i].k;
//    triangles.push_back (tr);
//  }
//  write_vtk_file("output.vtk", triangles, msg->points, triangles.size());
  mesh_ = Ogre::MeshManager::getSingleton().createManual ("triangularmesh", "General");
  submesh_ = mesh_->createSubMesh();

  // We first create a VertexData
  data = new Ogre::VertexData();
  // Then, we link it to our Mesh/SubMesh :
  mesh_->sharedVertexData = data;
  // submesh_->useSharedVertices = false; // This value is 'true' by default
  // submesh_->vertexData = data;
  // We have to provide the number of verteices we'll put into this Mesh/SubMesh
  data->vertexCount = msg->points.size();
  // Then we can create our VertexDeclaration
  Ogre::VertexDeclaration* decl = data->vertexDeclaration;
  decl->addElement (0,0, Ogre::VET_FLOAT3, Ogre::VES_POSITION);
  /** @todo: should add more elements, specifically normal vectors! */
  
  // create a vertex buffer
  vbuf = Ogre::HardwareBufferManager::getSingleton().createVertexBuffer(
    decl->getVertexSize(0),                     // This value is the size of a vertex in memory
    msg->points.size(),                         // The number of vertices you'll put into this buffer
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
  );
 
  double min_x=FLT_MAX,  min_y=FLT_MAX,  min_z=FLT_MAX;
  double max_x=-FLT_MAX, max_y=-FLT_MAX, max_z=-FLT_MAX;
  float* array = (float*) malloc (sizeof(float)*3*msg->points.size());
  for (unsigned int i = 0; i < msg->points.size(); i++)
  {
    array[i*3+0] = msg->points[i].x;
    array[i*3+1] = msg->points[i].y;
    array[i*3+2] = msg->points[i].z;
    if (msg->points[i].x < min_x) min_x = msg->points[i].x;
    if (msg->points[i].y < min_y) min_y = msg->points[i].y;
    if (msg->points[i].z < min_z) min_z = msg->points[i].z;
    if (msg->points[i].x > max_x) max_x = msg->points[i].x;
    if (msg->points[i].y > max_y) max_y = msg->points[i].y;
    if (msg->points[i].z > max_z) max_z = msg->points[i].z;
  }
ROS_INFO ("a %i: %f, %f, %f, %f, %f, %f", __LINE__, min_x, min_y, min_z, max_x, max_y, max_z);
  
  vbuf->writeData (0, vbuf->getSizeInBytes(), array, true); 
  free (array);
  
  Ogre::VertexBufferBinding* bind = data->vertexBufferBinding;
  bind->setBinding (0, vbuf);
  
  bool back_face_duplicate = true;
  int stride = 3;  
  if (back_face_duplicate)
    stride = 6;

  // create index buffer
  ibuf = Ogre::HardwareBufferManager::getSingleton().createIndexBuffer(
    Ogre::HardwareIndexBuffer::IT_32BIT,        // You can use several different value types here
    msg->triangles.size()*stride,                    // The number of indices you'll put in that buffer
    Ogre::HardwareBuffer::HBU_STATIC_WRITE_ONLY // Properties
  );
  
  submesh_->indexData->indexBuffer = ibuf;     // The pointer to the index buffer

  submesh_->indexData->indexCount = msg->triangles.size()*stride; // The number of indices we'll use
  submesh_->indexData->indexStart = 0;

  unsigned int *index;
  index = (unsigned int*) malloc (sizeof(unsigned int)*stride*msg->triangles.size());
  for (unsigned int i = 0; i < msg->triangles.size(); i++)
  {
    index[i*stride+0] = msg->triangles[i].i;
    index[i*stride+1] = msg->triangles[i].j;
    index[i*stride+2] = msg->triangles[i].k;
    if (back_face_duplicate)
    {
      index[i*stride+3] = msg->triangles[i].i;
      index[i*stride+4] = msg->triangles[i].k;
      index[i*stride+5] = msg->triangles[i].j;
    }
  }

  ibuf->writeData (0, ibuf->getSizeInBytes(), index, true); 
  free (index);
 
  mesh_->_setBounds (Ogre::AxisAlignedBox (min_x, min_y, min_z, max_x, max_y, max_z));
  mesh_->_setBoundingSphereRadius (std::max(max_x-min_x, std::max(max_y-min_y, max_z-min_z))/2.0f);

  mesh_->load ();

//  MaterialPtr material = MaterialManager::getSingleton().create(
//    "Test/ColourTest", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
//  material->getTechnique(0)->getPass(0)->setVertexColourTracking(TVC_AMBIENT);

  std::stringstream ss;
  ss << "tm" << count;
  entity_ = scene_manager_->createEntity(ss.str(), "triangularmesh");
  scene_node_->attachObject(entity_);

  // finally, make sure we get everything in RVIZ's coordinate system
  tf::Stamped<tf::Pose> pose(btTransform(btQuaternion(0.0f, 0.0f, 0.0f, 1.0f),
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
  
  Ogre::Quaternion orientation = Ogre::Quaternion (pose.getRotation().w(), pose.getRotation().x(), pose.getRotation().y(), pose.getRotation().z()); 

  rviz::robotToOgre(position);
  rviz::robotToOgre(orientation);

  scene_node_->setOrientation (orientation);
  scene_node_->setPosition (position);

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

