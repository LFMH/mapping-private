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

#ifndef CLOUD_ALGOS_CYLINDER_ESTIMATION_H
#define CLOUD_ALGOS_CYLINDER_ESTIMATION_H
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point32.h>

#include <ros/ros.h>
#include <ros/node_handle.h>

#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/lmeds.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/rransac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/rmsac.h>
#include <point_cloud_mapping/sample_consensus/mlesac.h>
#include <point_cloud_mapping/sample_consensus/sac_model.h>
#include <point_cloud_mapping/sample_consensus/sac_model_cylinder.h>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>

#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/distances.h>
//#include <tools/transform.h>
#include <angles/angles.h>

//for mesh output
#include <triangle_mesh_msgs/TriangleMesh.h>
#include <tf/tf.h>

// Eigen
#include <Eigen/Array>
#include <Eigen/Geometry>

#include <cloud_algos/cloud_algos.h>

using namespace sample_consensus;

namespace cloud_algos
{

class CylinderEstimation : public CloudAlgo
{
public:
  // should the result marker be published or only rotated
  bool publish_marker_; // TODO: get it from parameter server...

  CylinderEstimation () : CloudAlgo ()
  {
    publish_marker_ = true;
  }

  typedef triangle_mesh_msgs::TriangleMesh OutputType;
  typedef sensor_msgs::PointCloud InputType;

  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  static std::string default_output_topic ()
    {return std::string ("mesh_cylinder");};

  static std::string default_node_name () 
    {return std::string ("cylinder_estimation_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>);
  boost::shared_ptr<const OutputType> output ();

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief estimates point cloud normals
   * \param cloud input point cloud
   */
  //TODO - how to make use of the commented out signature
  //void estimatePointNormals (boost::shared_ptr<const sensor_msgs::PointCloud> cloud);
  void estimatePointNormals (sensor_msgs::PointCloud cloud);

  
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief cylinder SAC-model fitting function
   * \param cloud input point cloud
   * \param coeff cylinder coefficients to be calculated [point_on_axis - 3, axis_direction - 3, cylinder radius - 1]
   */
  //TODO - how to make use of the commented out signature
  //  void find_model(boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);
    void find_model(sensor_msgs::PointCloud cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief change the axis to be defined by the top-most and bottom-most points
   * instead of an arbitrary point and a direction
   * \param points 3D points of cylinder candidate  
   * \param inliers inlying points calculated by sac_->getInliers()
   * \param coeff cylinder coefficients to be modified
   */
  void changeAxisToMinMax (const std::vector<geometry_msgs::Point32> &points, std::vector<int> &inliers, std::vector<double> &coeff);

  
  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief triangulates cylinder model into TriangleMesh message
   * \param cylinder coefficients as calculated in changeAxisToMinMax function
   */  
  void triangulate_cylinder(std::vector<double> &coeff);


  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief publishes model (cylinder) marker (to rviz)
   * \param cylinder coefficients as calculated in changeAxisToMinMax function
   */  
  void publish_model_rviz (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Sets the internal box as marker for rvis visualisation.
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void computeMarker (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Returns the internal box as marker for rvis visualisation
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  visualization_msgs::Marker getMarker () { return marker_; }

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Returns the computed model coefficients
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  std::vector<double> getCoeff () { return coeff_; }

  // Get inlier and outlier points
  virtual boost::shared_ptr<sensor_msgs::PointCloud> getInliers ();
  virtual boost::shared_ptr<sensor_msgs::PointCloud> getOutliers ();
  virtual boost::shared_ptr<sensor_msgs::PointCloud> getThresholdedInliers (double eps_angle);

  boost::shared_ptr<sensor_msgs::PointCloud> points_;
  boost::shared_ptr<sensor_msgs::PointCloud> cylinder_points_;
  boost::shared_ptr<sensor_msgs::PointCloud> outlier_points_;
  SACModel *model_;
  SAC *sac_;
  std::string output_outliers_topic_;

  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }
protected:
  ros::NodeHandle nh_;
  ros::Publisher cloud_pub_;
  ros::Publisher outlier_pub_;
  //model rviz publisher
  ros::Publisher marker_pub_;
  visualization_msgs::Marker marker_;
  //normals' indices in PointCloud.channels
  int nx_, ny_, nz_;
  //needed to append channels for normals if PointCloud.channels.size() =! 0
  int channels_size_;
  //KD Tree parameter
  int k_;
  //model's coefficients, i.e. geometrical description
  std::vector<double> coeff_;
    geometry_msgs::Point32 axis_;
  float rotate_inliers_;
  //triangular mesh
  boost::shared_ptr<triangle_mesh_msgs::TriangleMesh> mesh_;
};

}//namespace cloud_algos
#endif


