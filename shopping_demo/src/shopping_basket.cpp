/*
 * Copyright (c) 2011, Lucian Cosmin Goron <goron@cs.tum.edu>
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






// ROS core
#include <ros/ros.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include "pcl/common/common.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
//#include <ias_table_msgs/TableCluster.h>

#include "pcl/segmentation/extract_clusters.h"
#include "pcl/kdtree/kdtree.h"
#include "pcl/kdtree/kdtree_flann.h"

using namespace std;




////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv)
{

  int queue_size = 10;

  std::string input_topic, output_topic, to_frame_;


  ros::init (argc, argv, "shopping_basket");

  ros::NodeHandle nh("~");
  ros::Publisher pub;



  //nh.param("input_topic", input_topic, std::string("input"));
  //nh.param("output_topic", output_topic, std::string("basket"));

  //nh.param("to_frame", to_frame_, std::string("/map"));



  visualization_msgs::Marker mesh;

  mesh.header.frame_id = "shopping_basket_link";
  mesh.header.stamp = ros::Time::now();
  mesh.ns = "";
  mesh.id = 0;

  mesh.type = visualization_msgs::Marker::MESH_RESOURCE;
  mesh.action = visualization_msgs::Marker::ADD;

  //mesh.mesh_resource = "file:///home/goron/ros/tum-stacks/mapping-private/shopping_demo/meshes/basket/models/skpfile.dae";
  //mesh.mesh_resource = "package://shopping_demo/meshes/basket/models/skpfile.dae";
  //mesh.mesh_resource = "package://shopping_demo/meshes/basket/models/easter.skp";
  //mesh.mesh_resource = "package://ias_pancakes/Media/models/pancake.dae";
  //mesh.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
  mesh.mesh_resource = "package://shopping_demo/meshes/models/basket_handle_center.stl";
  
  //  mesh.mesh_use_embedded_materials = true;

  mesh.pose.position.x = 0.0;
  mesh.pose.position.y = 0.0;
  mesh.pose.position.z = 0.0;
  mesh.pose.orientation.x = 0.0;
  mesh.pose.orientation.y = 0.0;
  mesh.pose.orientation.z = 0.0;
  mesh.pose.orientation.w = 1.0;
  mesh.scale.x = 0.05;
  mesh.scale.y = 0.05;
  mesh.scale.z = 0.05;
  mesh.color.a = 1.0;
  mesh.color.r = 1.0;
  mesh.color.g = 0.0;
  mesh.color.b = 0.0;
  mesh.lifetime = ros::Duration();

  pub = nh.advertise<visualization_msgs::Marker>( "basket", 1, true);

  while (ros::ok())
  {
    ros::spinOnce();
    sleep (1);

    pub.publish( mesh );

  } 

  //ros::spin ();

  return (0);
}
