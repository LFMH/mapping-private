/*
 * Copyright (c) 2010 Nico Blodow <blodow -=- cs.tum.edu>
 *
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
 *
 *
 */

/** 
@file
@brief mesh_to_pointcloud subscribes to a mesh and republishes the points on a
point cloud topic
*/

#include <ctime>
#include <ros/node_handle.h>
#include <triangle_mesh/TriangleMesh.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>

class MeshToPointcloud
{
  protected:
    ros::NodeHandle nh_;
    std::string input_mesh_topic_;
    std::string output_cloud_topic_;

    ros::Subscriber mesh_sub_;
    ros::Publisher cloud_pub_;

  public:
    MeshToPointcloud (ros::NodeHandle &anode) : nh_(anode)
    {
      nh_.param ("input_mesh_topic", input_mesh_topic_, std::string("mesh"));
      nh_.param ("output_cloud_topic", output_cloud_topic_, std::string("mesh_cloud"));
      
      mesh_sub_ = nh_.subscribe (input_mesh_topic_, 1, &MeshToPointcloud::mesh_cb, this);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud> (output_cloud_topic_, 1);
    }
    
    void
      mesh_cb (const triangle_mesh::TriangleMesh::ConstPtr& mesh)
    {
      sensor_msgs::PointCloud p;
      p.header = mesh->header;
      p.points = mesh->points;
      cloud_pub_.publish (p);
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "mesh_to_pointcloud");

  ros::NodeHandle nh("~");
  MeshToPointcloud n (nh);
  ros::spin ();

  return (0);
}


