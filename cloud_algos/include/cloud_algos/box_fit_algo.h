/* 
 * Copyright (c) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>, 
 Zoltan-Csaba Marton <marton@cs.tum.edu>, Nico Blodow <blodow@cs.tum.edu>
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

#ifndef CLOUD_ALGOS_BOX_ESTIMATION_H
#define CLOUD_ALGOS_BOX_ESTIMATION_H
#include <cloud_algos/cloud_algos.h>
#include <mapping_msgs/PolygonalMap.h>
#include <position_string_rviz_plugin/PositionStringList.h>
#include <triangle_mesh/TriangleMesh.h>
#include <point_cloud_mapping/geometry/point.h>

namespace cloud_algos
{

class BoxEstimation : public CloudAlgo
{
 public:
  BoxEstimation () { };
  typedef triangle_mesh::TriangleMesh OutputType;
  typedef sensor_msgs::PointCloud InputType;

  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  static std::string default_output_topic ()
    {return std::string ("mesh_box");};

  static std::string default_node_name () 
    {return std::string ("box_estimation_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>);
  boost::shared_ptr<const OutputType> output ();

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief function for actual model fitting
   * \param cloud de-noisified input point cloud message
   * \param coeff box to-be-filled-in coefficients(15 elements):
   * box center: cx, cy, cz, 
   * box dimensions: dx, dy, dz, 
   * box eigen axes: e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z  
   */
  void find_model (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief makes triangular mesh out of box coefficients
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void triangulate_box (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief publish box as marker for rvis visualisation
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void publish_marker (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);
 
 private: 
  boost::shared_ptr<OutputType> mesh_;
  std::vector<int> inliers_;
  std::vector<int> outliers_;

  ros::NodeHandle nh_;

 protected: 
  //model rviz publisher
  ros::Publisher box_pub_;
  //box coefficients: cx, cy, cz, dx, dy, dz, e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z  
  std::vector<double> coeff_;
  geometry_msgs::Point32 box_centroid_;
  //publish box as marker
  std::string output_box_topic_;
  //point color
  float r_, g_, b_;
  //lock point cloud
  boost::mutex lock;
};

}
#endif


