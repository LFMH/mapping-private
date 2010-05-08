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
#include <cloud_tools/misc.h>
#include <cloud_tools/laser_camera_virtual_view_calibration.h>


using namespace cloud_tools;


class LaserCameraVirtualViewCalibration
{
public:
  ros::NodeHandle nh_;
  std::string input_mesh_topic_;
  std::string output_image_topic_;
  
  ros::Subscriber mesh_sub_;
  
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;
  
  sensor_msgs::PointCloud cloud_in_;
  triangle_mesh::TriangleMesh mesh_; 
  int argc_;
  char ** argv_;
  //public:
  LaserCameraVirtualViewCalibration (ros::NodeHandle &anode) : nh_(anode), it_(nh_)
  {
    nh_.param ("input_mesh_topic", input_mesh_topic_, std::string("mesh"));
    nh_.param ("output_image_topic", output_image_topic_, std::string("scene_image"));  
//     nh_.param ("mouse_down", mouse_down_, false);  
//     nh_.param ("displayWin", displayWin, 1);
//     nh_.param ("width", width, 640);  
//     nh_.param ("height", height, 480);  
//     mouse_down = mouse_down_;
    mesh_sub_ = nh_.subscribe (input_mesh_topic_, 1, &LaserCameraVirtualViewCalibration::mesh_cb, this);
    image_pub_ = it_.advertise(output_image_topic_, 1);

  }

  /**
   * \brief mesh callback 
   * \param mesh input mesh message
   */
  void mesh_cb (const triangle_mesh::TriangleMeshConstPtr& mesh)
  {
  }

  bool spin ()
  {
        //double interval = rate_ * 1e+6;
    lc_main(argc_, argv_);
    ros::Rate loop_rate(1);
    while (nh_.ok ())
    {     
      loop_rate.sleep();
      //ros::spinOnce ();
    }
    return (true);
  }
};



int main (int argc, char* argv[])
{
  //argc_ = argc;
  //argv_ = argv;
  ros::init (argc, argv, "laser_camera_virtual_view_calibration_node");
  ros::NodeHandle nh("~");
  LaserCameraVirtualViewCalibration n (nh);
  n.argc_ = argc;
  n.argv_ = argv;
  n.spin();
  //ros::spin ();

  return (0);
}
