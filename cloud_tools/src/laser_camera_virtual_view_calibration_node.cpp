/* 
 * Copyright (c) 2010, 
 * Ioana <ioana_the_first@yahoo.com>
 * Zoltan-Csaba Marton <marton@cs.tum.edu>
 * Dejan Pangercic <pangercic@cs.tum.edu>
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

/** 
@file

@brief laser_camera_virtual_view_calibration takes in a triangular mesh (from 
triangulated PointCloud message) from a vtk file
and generates the scene image for a given focal length and a view point. The generated 
image is then used to carry out tilting laser-camera calibration as a normal
stereo calibration.
@par Execute:
 - ./laser_camera_virtual_view_calibration configuration.yaml

@par Example configuration file:
- vtk_file: path_to_file/test.vtk
- ppm_file: output.ppm
- position: [2.18616,0.602594,0.224822]
- focal_point: [0.66648,-3.03167,2.5434]
- view_up: [0.339361,0.400962,0.850919]
- height: 480
- width: 640
- display_win: 1
*/


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <float.h>
#include <vector>

#include "yaml-cpp/yaml.h"
#include <fstream>

#include <GL/glut.h>
#include<X11/X.h>
#include<X11/Xlib.h>
#include<GL/glx.h>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <triangle_mesh/TriangleMesh.h>
#include <cloud_tools/laser_camera_virtual_view_calibration.h>

#include <boost/thread.hpp>


class LaserCameraVirtualViewCalibration
{
public:
  ros::NodeHandle nh_;
  //node parameters
  std::string input_mesh_topic_, input_image_topic_;
  std::string laser_image_name_, camera_image_name_;
  bool camera_image_saved_;
  //prepended with ros::Time::now()
  std::stringstream ros_stamp_;
  int width_, height_, display_win_;
  point_3D position_, focal_point_,view_up_;
  std::vector<point_3D> points_;
  std::vector<triangle> triangles_;
  int nr_pct_;
  int nr_tr_;
  int argc_;
  char ** argv_;
  int file_name_counter_;
  //subscribers, publishers
  ros::Subscriber mesh_sub_;
  image_transport::Subscriber image_sub_;
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  
  sensor_msgs::PointCloud cloud_in_;
  triangle_mesh::TriangleMesh mesh_; 
 
  boost::thread spin_thread_;
  //public:
  LaserCameraVirtualViewCalibration (ros::NodeHandle &anode) : nh_(anode), it_(nh_)
  {
    nh_.param ("input_mesh_topic", input_mesh_topic_, std::string("/depth_image_triangulation_node/cloud_triangulated"));
    nh_.param ("input_image_topic", input_image_topic_, std::string("/image_topic_2"));
    nh_.param ("laser_image_name", laser_image_name_, std::string("laser_image"));  
    nh_.param ("camera_image_name", camera_image_name_, std::string("camera_image"));  
    nh_.param ("width", width_, 800);  
    nh_.param ("height", height_, 600);  
    nh_.param ("display_win", display_win_, 1);  
    XmlRpc::XmlRpcValue p, fp, vu;
    p.setSize(3), fp.setSize(3), vu.setSize(3);
    p[0] = -2.56693,    p[2] = 1.25371,    p[2] = 3.49953;
    fp[0] = 1.84166,    fp[2] = 0.447215,    fp[2] = 0.258622;
    vu[0] = 0.582762,    vu[2] = -0.068426,    vu[2] = 0.809757;
    nh_.param("position", p, p);
    nh_.param("focal_point", fp, fp);
    nh_.param("view_point", vu, vu);
    position_.x = p[0], position_.y = p[1], position_.z = p[2]; 
    focal_point_.x = fp[0], focal_point_.y = fp[1], focal_point_.z = fp[2]; 
    view_up_.x = vu[0], view_up_.y = vu[1], view_up_.z = vu[2]; 
    mesh_sub_ = nh_.subscribe (input_mesh_topic_, 1, &LaserCameraVirtualViewCalibration::mesh_cb, this);
    image_sub_ = it_.subscribe(input_image_topic_, 1, &LaserCameraVirtualViewCalibration::image_cb, this);
    file_name_counter_ = 0;
    camera_image_saved_ = false;
  }

  /**
   * \brief image callback
   * \param msg_ptr input image message
   */
  void image_cb(const sensor_msgs::ImageConstPtr& msg_ptr)
  {
    if (!camera_image_saved_)
    {
      IplImage *cv_image = NULL;
      try
      {
        cv_image = bridge_.imgMsgToCv(msg_ptr, "passthrough");
      }
      catch (sensor_msgs::CvBridgeException error)
      {
        ROS_ERROR("[LCVVC: ] error in imgMsgToCv");
      }
      ros_stamp_ << ros::Time::now();
      std::string final_image_name = ros_stamp_.str() + "_" + camera_image_name_ + ".png";
      ROS_INFO("Saving image to %s", final_image_name.c_str());
      cvSaveImage(final_image_name.c_str(), cv_image);
      camera_image_saved_ = true;
    }
  }

  /**
   * \brief mesh callback 
   * \param mesh input mesh message
   */
  void mesh_cb (const triangle_mesh::TriangleMeshConstPtr& mesh)
  {
    if (camera_image_saved_)
    {
      if (mesh->points.size() != mesh->intensities.size())
        ROS_WARN("Unusual!!! TriangleMesh's points and intensities channels differ in sizes");
      //read in points
      nr_pct_ = mesh->points.size();
      points_.resize (mesh->points.size());
      for (unsigned int i = 0; i < mesh->points.size(); i++)
      {
        points_[i].x = mesh->points[i].x;
        points_[i].y = mesh->points[i].y;
        points_[i].z = mesh->points[i].z;
      }
      
      //read in triangles
      nr_tr_ = mesh->triangles.size();
      triangles_.resize (mesh->triangles.size());
      for (unsigned int i = 0; i < mesh->triangles.size(); i++)
      {
        triangles_[i].a = mesh->triangles[i].i;
        triangles_[i].b = mesh->triangles[i].j;
        triangles_[i].c = mesh->triangles[i].k;
      }
      
      //read in intensities
      for (unsigned int i = 0; i < mesh->intensities.size(); i++)
      {
        points_[i].i = mesh->intensities[i];
      }
      std::string final_laser_image_name = ros_stamp_.str() + "_" + laser_image_name_ + ".ppm";
      
      lc_main (argc_, argv_, final_laser_image_name, position_, focal_point_, 
               view_up_, width_, height_, display_win_, points_, nr_pct_, triangles_,  nr_tr_);
      
      file_name_counter_++;
      points_.resize(0);
      triangles_.resize(0);
      camera_image_saved_ = false;
    }
  }
}; //end LaserCameraVirtualViewCalibration



int main (int argc, char* argv[])
{
  ros::init (argc, argv, "laser_camera_virtual_view_calibration_node");
  ros::NodeHandle nh("~");
  LaserCameraVirtualViewCalibration n (nh);
  n.argc_ = argc;
  n.argv_ = argv;
  ros::spin ();

  return (0);
}
