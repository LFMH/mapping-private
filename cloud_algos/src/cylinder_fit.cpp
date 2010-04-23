/*
 * Copyright (c) 2020 Dejan Pangercic <pangercic -=- cs.tum.edu>
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
 * $Id: cylinder_fit.cpp 21050 2009-08-07 21:24:30Z jfaustwg $
 *
 */

/** \author Radu Bogdan Rusu, Dejan Pangercic */

#include <gtest/gtest.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point32.h>

#include <ros/ros.h>

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
#include <tools/transform.h>
#include <angles/angles.h>

//for mesh output
#include <triangle_mesh/TriangleMesh.h>
#include <tf/tf.h>
using namespace sample_consensus;

class CylinderFit
{
 public:
  sensor_msgs::PointCloud points_;
  sensor_msgs::PointCloud cylinder_points_;
  sensor_msgs::PointCloud outlier_points_;
  SACModel *model_;
  SAC *sac_;
  std::string input_cloud_topic_, output_outliers_topic_;
  ///////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Contructor
   */
  CylinderFit(ros::NodeHandle &n) :
    n_(n)
  {
    n_.param("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
    n_.param("output_outliers_topic", output_outliers_topic_, std::string("cylinder"));
    n_.param("output_mesh_topic", output_mesh_topic_, std::string("cylinder_mesh"));
    cylinder_pub_ = n_.advertise<sensor_msgs::PointCloud>(output_outliers_topic_ ,1);
    vis_pub = n_.advertise<visualization_msgs::Marker>( "visualization_marker", 0 );
    mesh_pub_ = n_.advertise<triangle_mesh::TriangleMesh> (output_mesh_topic_, 1);
    //use either next 2 or the 3rd line
    clusters_sub_ = n_.subscribe (input_cloud_topic_, 1, &CylinderFit::cloud_cb, this);
    model_ = new SACModelCylinder ();
    sac_ = new RANSAC (model_, 0.01);
    nx_ = ny_ = nz_ = -1;
    channels_size_ = 0;
    k_ = 20;
    axis_.x = 1;
    axis_.y = axis_.z = 0;
    rotate_inliers_ = 0.0;
 }
  
  ///////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Destructor
   */
  ~CylinderFit()
  {
    delete model_;
    delete sac_;
  }


  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief cloud_cb
   */
  void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
  {
    ROS_INFO ("PointCloud message received on %s with %d points.", input_cloud_topic_.c_str (), (int)cloud->points.size ());
    points_ = *cloud;

    
    for (unsigned long i = 0; i < points_.channels.size(); i++)
    {
      if(points_.channels[i].name == "nx")
        nx_ = i;
      if(points_.channels[i].name == "ny")
        ny_ = i;
      if(points_.channels[i].name == "nz")
        nz_ = i;
    }
    channels_size_ = (int)points_.channels.size();
    //no normals, estimate them
    if (nx_ == -1 || ny_ == -1 || nz_ == -1) 
    {
      points_.channels.resize (channels_size_ + 3);
      points_.channels[channels_size_ + 0].name = "nx";
      points_.channels[channels_size_ + 1].name = "ny";
      points_.channels[channels_size_ + 2].name = "nz";
      
      for (unsigned int d = channels_size_; d < (channels_size_ + 3); d++)
        points_.channels[d].values.resize (points_.points.size ());
      estimatePointNormals(points_);
    }

    find_model(points_, coeff);
    triangulate_cylinder(coeff);
    mesh_pub_.publish(mesh_);
    publish_model_rviz(coeff);
  }

  
    ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief estimates point cloud normals
   */
  void estimatePointNormals (sensor_msgs::PointCloud &cloud)
  {
    ros::Time ts = ros::Time::now ();
    ROS_INFO ("+ estimatePointNormals, %ld", cloud.points.size ());
    cloud_kdtree::KdTree *kdtree = new cloud_kdtree::KdTreeANN (cloud);
    std::vector<std::vector<int> > points_k_indices;
    ROS_INFO ("1 estimatePointNormals");
    // Allocate enough space for point indices
    points_k_indices.resize (cloud.points.size ());
    ROS_INFO ("2 estimatePointNormals %i", k_);
    for (int i = 0; i < (int)cloud.points.size (); i++)
        points_k_indices[i].resize (k_);
    // Get the nerest neighbors for all the point indices in the bounds
    ROS_INFO ("3 estimatePointNormals");
    std::vector<float> distances;
    for (int i = 0; i < (int)cloud.points.size (); i++)
      kdtree->nearestKSearch (i, k_, points_k_indices[i], distances);
    
    ROS_INFO ("4 estimatePointNormals");
    // Figure out the viewpoint value in the point cloud frame
    geometry_msgs::PointStamped viewpoint_laser, viewpoint_cloud;

    ////////////////////////////////////////////////////////////////////////////
    //TODO: uncomment when getting point clouds from the robot
    ////////////////////////////////////////////////////////////////////////////
    //viewpoint_laser.header.frame_id = "laser_tilt_mount_link";
    
    // Set the viewpoint in the laser coordinate system to 0,0,0
    ROS_INFO ("5 estimatePointNormals");
    viewpoint_laser.point.x = viewpoint_laser.point.y = viewpoint_laser.point.z = 0.0;
    
    ////////////////////////////////////////////////////////////////////////////
    //TODO: uncomment when getting point clouds from the robot
    ////////////////////////////////////////////////////////////////////////////
//     try
//     {
//       tf_.transformPoint (cloud.header.frame_id, viewpoint_laser, viewpoint_cloud);
//     }
//     catch (tf::TransformException)
//     {
//       viewpoint_cloud.point.x = viewpoint_cloud.point.y = viewpoint_cloud.point.z = 0.0;
//     }
    
    
    ROS_INFO ("5 estimatePointNormals");
    //#pragma omp parallel for schedule(dynamic)
    for (int i = 0; i < (int)cloud.points.size (); i++)
    {
      // Compute the point normals (nx, ny, nz), surface curvature estimates (c)
      Eigen::Vector4d plane_parameters;
      double curvature;
      cloud_geometry::nearest::computePointNormal (cloud, points_k_indices[i], plane_parameters, curvature);
      
      cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud.points[i], viewpoint_cloud);
      
      cloud.channels[channels_size_ + 0].values[i] = plane_parameters (0);
      cloud.channels[channels_size_ + 1].values[i] = plane_parameters (1);
      cloud.channels[channels_size_ + 2].values[i] = plane_parameters (2);
    }
    // Delete the kd-tree
    delete kdtree;
    ROS_INFO("estimatePointNormals completed in %f seconds",(ros::Time::now () - ts).toSec ());
  }
  

 ////////////////////////////////////////////////////////////////////////////////
  /**
   * actual model fitting happens here
   */
  void find_model(sensor_msgs::PointCloud &cloud, std::vector<double> &coeff)
  {
    // cylinder coefficients:
    // point_on_axis - 3
    // axis_direction - 3
    // cylinder radius - 1
    if(cloud.points.size() != 0)
    {
      model_->setDataSet (&cloud);
      
      bool result = sac_->computeModel ();
      if(result)
      {
        std::vector<int> inliers = sac_->getInliers ();
        ROS_INFO("inliers size %ld", inliers.size());
        cloud_geometry::getPointCloud (cloud, inliers, cylinder_points_);
        
        
        sac_->computeCoefficients (coeff);
        ROS_INFO("Cylinder coefficients: %f %f %f %f %f %f %f", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
        
        // // std::vector<double> coeff_ref;
        sac_->refineCoefficients (coeff);


        // transform coefficients to get the limits of the cylinder
        changeAxisToMinMax(cylinder_points_.points, inliers, coeff);
        ROS_INFO("Cylinder coefficients in min-max mode: %g %g %g %g %g %g %g", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);

        cloud_geometry::getPointCloudOutside(cloud, inliers, outlier_points_);
        cylinder_pub_.publish (outlier_points_);
        ROS_INFO ("Publishing data on topic %s with nr of points %ld", n_.resolveName (output_outliers_topic_).c_str (), 
                  outlier_points_.points.size());
      }
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
   /**
   * \brief change the axis to be defined by the top-most and bottom-most points
   * instead of an arbitrary point and a direction
   */
  void changeAxisToMinMax (const std::vector<geometry_msgs::Point32> &points, std::vector<int> &inliers, std::vector<double> &coeff)
  {
    // P0 (coeff[0], coeff[1], coeff[2]) - a point on the axis
    // direction (coeff[3], coeff[4], coeff[5]) - the direction vector of the axis
    // P (points[i].x, points[i].y, points[i].z) - a point on the cylinder
    btVector3 p0(coeff[0], coeff[1], coeff[2]);
    btVector3 direction(coeff[3], coeff[4], coeff[5]);
    //direction.normalize();
    double min_t = DBL_MAX, max_t = -DBL_MAX;
    for (size_t i = 0; i < inliers.size (); i++)
    {
      // dot product of (P-P0) and direction to get the distance of P's projection from P0 ("t" in the parametric equation of the axis)
      double t = (points[i].x-p0.x())*direction.normalize().x() + (points[i].y-p0.y())*direction.normalize().y() + (points[i].z-p0.z())*direction.normalize().z();
      if (t < min_t) min_t = t;
      if (t > max_t) max_t = t;
    }
    ROS_INFO("min, max t %lf - %lf = %f ", min_t, max_t, max_t - min_t);
    // update coefficients with the two extreme points on the axis line
    coeff[0] = p0.x() + min_t * direction.x ();
    coeff[1] = p0.y() + min_t * direction.y ();
    coeff[2] = p0.z() + min_t * direction.z ();
    coeff[3] = p0.x() + max_t * direction.x ();
    coeff[4] = p0.y() + max_t * direction.y ();
    coeff[5] = p0.z() + max_t * direction.z ();
  }


  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief triangulates cylinder model
   */  
  void triangulate_cylinder(std::vector<double> &coeff)
  {
    mesh_.points.resize(0);
    mesh_.triangles.resize(0);
    mesh_.header = points_.header;
    geometry_msgs::Point32 p1, p2, p3, p4;
    triangle_mesh::Triangle triangle1, triangle2;
    double radius = coeff[6];
    int angle_step = 10;
    for (int angle = 0; angle < 360; angle = angle + angle_step)
    {
      p1.x = sin(angles::from_degrees(angle)) * radius + coeff[0];
      p1.y = cos(angles::from_degrees(angle)) * radius + coeff[1];
      // std::cerr << "p1 " << p1.x << p1.y << std::endl;
      p1.z = coeff[2];
      p2.x = p1.x;
      p2.y = p1.y;
      p2.z = p1.z + (coeff[5] - coeff[2]);
      p3.x = sin(angles::from_degrees(angle + angle_step)) * radius + coeff[0];
      p3.y = cos(angles::from_degrees(angle + angle_step)) * radius + coeff[1];
      //      std::cerr << "p3 " << p3.x << p3.y << std::endl;
      p3.z = coeff[2];
      p4.x = p3.x;
      p4.y = p3.y;
      p4.z = p3.z + (coeff[5] - coeff[2]);
      mesh_.points.push_back(p1);
      mesh_.points.push_back(p2);
      mesh_.points.push_back(p3);
      mesh_.points.push_back(p4);
      triangle1.i = mesh_.points.size() - 4;
      triangle1.j = mesh_.points.size() - 3;
      triangle1.k = mesh_.points.size() - 2;
      triangle2.i = mesh_.points.size() - 2;
      triangle2.j = mesh_.points.size() - 3;
      triangle2.k = mesh_.points.size() - 1;
      mesh_.triangles.push_back(triangle1);
      mesh_.triangles.push_back(triangle2);
    }
  }


 ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief publishes model marker (to rviz)
   */  
  void publish_model_rviz (std::vector<double> &coeff)
  {
    //axis angle to quaternion conversion
    btVector3 axis(coeff[3]-coeff[0], coeff[4]-coeff[1], coeff[5]-coeff[2]);
    btVector3 marker_axis(0, 0, 1);
    
    btQuaternion qt(marker_axis.cross(axis), marker_axis.angle(axis));
    ROS_INFO("qt x, y, z, w:  %f, %f, %f, %f", qt.x(), qt.y(), qt.z(), qt.w());
    
    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = coeff[0]+axis.x()/2;
    marker.pose.position.y = coeff[1]+axis.y()/2;
    marker.pose.position.z = coeff[2]+axis.z()/2;
    marker.pose.orientation.x = qt.x();
    marker.pose.orientation.y = qt.y();
    marker.pose.orientation.z = qt.z();
    marker.pose.orientation.w = qt.w();
    marker.scale.x = 2*coeff[6];
    marker.scale.y = 2*coeff[6];
    marker.scale.z = axis.length ();
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    ROS_INFO("Publishing on /visualization_marker topic");
    vis_pub.publish( marker );
  }
 ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief spin
   */
  bool spin ()
  {
    ros::Duration delay(5.0);
    while (n_.ok())
    { 
      ros::spinOnce();
      delay.sleep();
    }
    return true;
  }

  protected:
  ros::NodeHandle n_;
  ros::Publisher cloud_pub_;
  ros::Publisher cylinder_pub_;
  ros::Subscriber clusters_sub_;
  //model rviz publisher
  ros::Publisher vis_pub;
  //normals' indices in PointCloud.channels
  int nx_, ny_, nz_;
  //needed to append channels for normals if PointCloud.channels.size() =! 0
  int channels_size_;
  //KD Tree parameter
  int k_;
  //model's coefficients, i.e. geometrical description
  std::vector<double> coeff;
  
  //set to != 0 if you want inlying points to be rotated.
  //rotation is around cloud's frame_id frame
  geometry_msgs::Point32 axis_;
  float rotate_inliers_;
  //mesh publisher
  ros::Publisher mesh_pub_;
  //publish mesh on topic
  std::string output_mesh_topic_;
  //triangular mesh
  triangle_mesh::TriangleMesh mesh_;
};

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_fit");
  ros::NodeHandle n("~");
  CylinderFit c(n);
  c.spin();
  return 0;
}
/* ]--- */
