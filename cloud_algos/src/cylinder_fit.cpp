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

using namespace sample_consensus;

class CylinderFit
{
 public:
  sensor_msgs::PointCloud points_;
  sensor_msgs::PointCloud cylinder_points_;
  SACModel *model_;
  SAC *sac_;
  std::string cloud_topic_, cylinder_topic_;
  ///////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Contructor
   */
  CylinderFit(ros::NodeHandle &n) :
    n_(n)
  {
    cloud_topic_ = "/cloud_pcd";
    cylinder_topic_ = "cylinder";
    cylinder_pub_ = n_.advertise<sensor_msgs::PointCloud>(cylinder_topic_ ,1);
    //use either next 2 or the 3rd line
    //create_point_cloud();
    //cloud_pub_ = n_.advertise<sensor_msgs::PointCloud>(cloud_topic_, 1);
    clusters_sub_ = n_.subscribe (cloud_topic_, 1, &CylinderFit::cloud_cb, this);
    model_ = new SACModelCylinder ();
    sac_ = new RANSAC (model_, 0.05);
    nx_ = ny_ = nz_ = -1;
    channels_size_ = 0;
    k_ = 20;
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
    ROS_INFO ("PointCloud message received on %s with %d points.", cloud_topic_.c_str (), (int)cloud->points.size ());
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
      estimatePointNormals(points_);
    }
    find_model(points_);
  }

  
    ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief estimates point cloud normals
   */
  void estimatePointNormals (sensor_msgs::PointCloud &cloud)
  {
    ROS_INFO ("+ estimatePointNormals, %i", cloud.points.size ());
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
    ROS_INFO ("- estimatePointNormals");
  }
  

 ////////////////////////////////////////////////////////////////////////////////
  /**
   * actual model fitting happens here
   */
void find_model(sensor_msgs::PointCloud &cloud)
  {
    if(cloud.points.size() != 0)
    {
      model_->setDataSet (&cloud);
      //EXPECT_EQ ((int)model_->getCloud ()->points.size (), 20);
      
      bool result = sac_->computeModel ();
      //EXPECT_EQ (result, true);
      if(result)
      {
        std::vector<int> inliers = sac_->getInliers ();
        ROS_INFO("inliers size %ld", inliers.size());
        cloud_geometry::getPointCloud (cloud, inliers, cylinder_points_);
        //EXPECT_EQ ((int)inliers.size (), 20);
        
        std::vector<double> coeff;
        sac_->computeCoefficients (coeff);
        //EXPECT_EQ ((int)coeff.size (), 7);
        //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
        //EXPECT_NEAR (coeff[0], -0.5, 1e-1);
        //EXPECT_NEAR (coeff[1], 1.7, 1e-1);
        //EXPECT_NEAR (coeff[6], 0.5, 1e-1);
        
        std::vector<double> coeff_ref;
        sac_->refineCoefficients (coeff_ref);
        //EXPECT_EQ ((int)coeff_ref.size (), 7);
        //EXPECT_NEAR (coeff_ref[6], 0.5, 1e-1);
        
        //int nr_points_left = sac_->removeInliers ();
        //EXPECT_EQ (nr_points_left, 0);
        //cloud_pub_.publish (points_);
        //ROS_INFO ("Publishing data on topic %s.", n_.resolveName (cloud_topic_).c_str ());
        cylinder_pub_.publish (cylinder_points_);
        ROS_INFO ("Publishing data on topic %s with nr of points %ld", n_.resolveName (cylinder_topic_).c_str (), 
                  cylinder_points_.points.size());
      }
    }
  }


 ////////////////////////////////////////////////////////////////////////////////
  /**
   * spin
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

    ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief creates a mini circle pointcloud
   */
  void create_point_cloud()
  {
    points_.header.frame_id = "/base_link";
    points_.points.resize (20);
    
    points_.set_channels_size (3);
    points_.channels[0].name = "nx";
    points_.channels[1].name = "ny";
    points_.channels[2].name = "nz";
    points_.channels[0].values.resize (points_.points.size ());
    points_.channels[1].values.resize (points_.points.size ());
    points_.channels[2].values.resize (points_.points.size ());
    
    points_.points[0].x = -0.499902; points_.points[0].y = 2.199701; points_.points[0].z = 0.000008;
    points_.points[1].x = -0.875397; points_.points[1].y = 2.030177; points_.points[1].z = 0.050104;
    points_.points[2].x = -0.995875; points_.points[2].y = 1.635973; points_.points[2].z = 0.099846;
    points_.points[3].x = -0.779523; points_.points[3].y = 1.285527; points_.points[3].z = 0.149961;
    points_.points[4].x = -0.373285; points_.points[4].y = 1.216488; points_.points[4].z = 0.199959;
    points_.points[5].x = -0.052893; points_.points[5].y = 1.475973; points_.points[5].z = 0.250101;
    points_.points[6].x = -0.036558; points_.points[6].y = 1.887591; points_.points[6].z = 0.299839;
    points_.points[7].x = -0.335048; points_.points[7].y = 2.171994; points_.points[7].z = 0.350001;
    points_.points[8].x = -0.745456; points_.points[8].y = 2.135528; points_.points[8].z = 0.400072;
    points_.points[9].x = -0.989282; points_.points[9].y = 1.803311; points_.points[9].z = 0.449983;
    points_.points[10].x = -0.900651; points_.points[10].y = 1.400701; points_.points[10].z = 0.500126;
    points_.points[11].x = -0.539658; points_.points[11].y = 1.201468; points_.points[11].z = 0.550079;
    points_.points[12].x = -0.151875; points_.points[12].y = 1.340951; points_.points[12].z = 0.599983;
    points_.points[13].x = -0.000724; points_.points[13].y = 1.724373; points_.points[13].z = 0.649882;
    points_.points[14].x = -0.188573; points_.points[14].y = 2.090983; points_.points[14].z = 0.699854;
    points_.points[15].x = -0.587925; points_.points[15].y = 2.192257; points_.points[15].z = 0.749956;
    points_.points[16].x = -0.927724; points_.points[16].y = 1.958846; points_.points[16].z = 0.800008;
    points_.points[17].x = -0.976888; points_.points[17].y = 1.549655; points_.points[17].z = 0.849970;
    points_.points[18].x = -0.702003; points_.points[18].y = 1.242707; points_.points[18].z = 0.899954;
    points_.points[19].x = -0.289916; points_.points[19].y = 1.246296; points_.points[19].z = 0.950075;
    
    points_.channels[0].values[0] = 0.000098;  points_.channels[1].values[0] = 1.000098;  points_.channels[2].values[0] = 0.000008;
    points_.channels[0].values[1] = -0.750891; points_.channels[1].values[1] = 0.660413;  points_.channels[2].values[1] = 0.000104;
    points_.channels[0].values[2] = -0.991765; points_.channels[1].values[2] = -0.127949; points_.channels[2].values[2] = -0.000154;
    points_.channels[0].values[3] = -0.558918; points_.channels[1].values[3] = -0.829439; points_.channels[2].values[3] = -0.000039;
    points_.channels[0].values[4] = 0.253627;  points_.channels[1].values[4] = -0.967447; points_.channels[2].values[4] = -0.000041;
    points_.channels[0].values[5] = 0.894105;  points_.channels[1].values[5] = -0.447965; points_.channels[2].values[5] = 0.000101;
    points_.channels[0].values[6] = 0.926852;  points_.channels[1].values[6] = 0.375543;  points_.channels[2].values[6] = -0.000161;
    points_.channels[0].values[7] = 0.329948;  points_.channels[1].values[7] = 0.943941;  points_.channels[2].values[7] = 0.000001;
    points_.channels[0].values[8] = -0.490966; points_.channels[1].values[8] = 0.871203;  points_.channels[2].values[8] = 0.000072;
    points_.channels[0].values[9] = -0.978507; points_.channels[1].values[9] = 0.206425;  points_.channels[2].values[9] = -0.000017;
    points_.channels[0].values[10] = -0.801227; points_.channels[1].values[10] = -0.598534; points_.channels[2].values[10] = 0.000126;
    points_.channels[0].values[11] = -0.079447; points_.channels[1].values[11] = -0.996697; points_.channels[2].values[11] = 0.000079;
    points_.channels[0].values[12] = 0.696154;  points_.channels[1].values[12] = -0.717889; points_.channels[2].values[12] = -0.000017;
    points_.channels[0].values[13] = 0.998685;  points_.channels[1].values[13] = 0.048502;  points_.channels[2].values[13] = -0.000118;
    points_.channels[0].values[14] = 0.622933;  points_.channels[1].values[14] = 0.782133;  points_.channels[2].values[14] = -0.000146;
    points_.channels[0].values[15] = -0.175948; points_.channels[1].values[15] = 0.984480;  points_.channels[2].values[15] = -0.000044;
    points_.channels[0].values[16] = -0.855476; points_.channels[1].values[16] = 0.517824;  points_.channels[2].values[16] = 0.000008;
    points_.channels[0].values[17] = -0.953769; points_.channels[1].values[17] = -0.300571; points_.channels[2].values[17] = -0.000030;
    points_.channels[0].values[18] = -0.404035; points_.channels[1].values[18] = -0.914700; points_.channels[2].values[18] = -0.000046;
    points_.channels[0].values[19] = 0.420154;  points_.channels[1].values[19] = -0.907445; points_.channels[2].values[19] = 0.000075;
  }

  protected:
  ros::NodeHandle n_;
  ros::Publisher cloud_pub_;
  ros::Publisher cylinder_pub_;
  ros::Subscriber clusters_sub_;
  //normals' indices in PointCloud.channels
  int nx_, ny_, nz_;
  //needed to append channels for normals if PointCloud.channels.size() =! 0
  int channels_size_;
  //KD Tree parameter
  int k_;
};

// TEST (RANSAC, SACModelCylinder)
// {
//   sensor_msgs::PointCloud points;
//   points.points.resize (20);

//   points.set_channels_size (3);
//   points.channels[0].name = "nx";
//   points.channels[1].name = "ny";
//   points.channels[2].name = "nz";
//   points.channels[0].values.resize (points.points.size ());
//   points.channels[1].values.resize (points.points.size ());
//   points.channels[2].values.resize (points.points.size ());

//   points.points[0].x = -0.499902; points.points[0].y = 2.199701; points.points[0].z = 0.000008;
//   points.points[1].x = -0.875397; points.points[1].y = 2.030177; points.points[1].z = 0.050104;
//   points.points[2].x = -0.995875; points.points[2].y = 1.635973; points.points[2].z = 0.099846;
//   points.points[3].x = -0.779523; points.points[3].y = 1.285527; points.points[3].z = 0.149961;
//   points.points[4].x = -0.373285; points.points[4].y = 1.216488; points.points[4].z = 0.199959;
//   points.points[5].x = -0.052893; points.points[5].y = 1.475973; points.points[5].z = 0.250101;
//   points.points[6].x = -0.036558; points.points[6].y = 1.887591; points.points[6].z = 0.299839;
//   points.points[7].x = -0.335048; points.points[7].y = 2.171994; points.points[7].z = 0.350001;
//   points.points[8].x = -0.745456; points.points[8].y = 2.135528; points.points[8].z = 0.400072;
//   points.points[9].x = -0.989282; points.points[9].y = 1.803311; points.points[9].z = 0.449983;
//   points.points[10].x = -0.900651; points.points[10].y = 1.400701; points.points[10].z = 0.500126;
//   points.points[11].x = -0.539658; points.points[11].y = 1.201468; points.points[11].z = 0.550079;
//   points.points[12].x = -0.151875; points.points[12].y = 1.340951; points.points[12].z = 0.599983;
//   points.points[13].x = -0.000724; points.points[13].y = 1.724373; points.points[13].z = 0.649882;
//   points.points[14].x = -0.188573; points.points[14].y = 2.090983; points.points[14].z = 0.699854;
//   points.points[15].x = -0.587925; points.points[15].y = 2.192257; points.points[15].z = 0.749956;
//   points.points[16].x = -0.927724; points.points[16].y = 1.958846; points.points[16].z = 0.800008;
//   points.points[17].x = -0.976888; points.points[17].y = 1.549655; points.points[17].z = 0.849970;
//   points.points[18].x = -0.702003; points.points[18].y = 1.242707; points.points[18].z = 0.899954;
//   points.points[19].x = -0.289916; points.points[19].y = 1.246296; points.points[19].z = 0.950075;

//   points.channels[0].values[0] = 0.000098;  points.channels[1].values[0] = 1.000098;  points.channels[2].values[0] = 0.000008;
//   points.channels[0].values[1] = -0.750891; points.channels[1].values[1] = 0.660413;  points.channels[2].values[1] = 0.000104;
//   points.channels[0].values[2] = -0.991765; points.channels[1].values[2] = -0.127949; points.channels[2].values[2] = -0.000154;
//   points.channels[0].values[3] = -0.558918; points.channels[1].values[3] = -0.829439; points.channels[2].values[3] = -0.000039;
//   points.channels[0].values[4] = 0.253627;  points.channels[1].values[4] = -0.967447; points.channels[2].values[4] = -0.000041;
//   points.channels[0].values[5] = 0.894105;  points.channels[1].values[5] = -0.447965; points.channels[2].values[5] = 0.000101;
//   points.channels[0].values[6] = 0.926852;  points.channels[1].values[6] = 0.375543;  points.channels[2].values[6] = -0.000161;
//   points.channels[0].values[7] = 0.329948;  points.channels[1].values[7] = 0.943941;  points.channels[2].values[7] = 0.000001;
//   points.channels[0].values[8] = -0.490966; points.channels[1].values[8] = 0.871203;  points.channels[2].values[8] = 0.000072;
//   points.channels[0].values[9] = -0.978507; points.channels[1].values[9] = 0.206425;  points.channels[2].values[9] = -0.000017;
//   points.channels[0].values[10] = -0.801227; points.channels[1].values[10] = -0.598534; points.channels[2].values[10] = 0.000126;
//   points.channels[0].values[11] = -0.079447; points.channels[1].values[11] = -0.996697; points.channels[2].values[11] = 0.000079;
//   points.channels[0].values[12] = 0.696154;  points.channels[1].values[12] = -0.717889; points.channels[2].values[12] = -0.000017;
//   points.channels[0].values[13] = 0.998685;  points.channels[1].values[13] = 0.048502;  points.channels[2].values[13] = -0.000118;
//   points.channels[0].values[14] = 0.622933;  points.channels[1].values[14] = 0.782133;  points.channels[2].values[14] = -0.000146;
//   points.channels[0].values[15] = -0.175948; points.channels[1].values[15] = 0.984480;  points.channels[2].values[15] = -0.000044;
//   points.channels[0].values[16] = -0.855476; points.channels[1].values[16] = 0.517824;  points.channels[2].values[16] = 0.000008;
//   points.channels[0].values[17] = -0.953769; points.channels[1].values[17] = -0.300571; points.channels[2].values[17] = -0.000030;
//   points.channels[0].values[18] = -0.404035; points.channels[1].values[18] = -0.914700; points.channels[2].values[18] = -0.000046;
//   points.channels[0].values[19] = 0.420154;  points.channels[1].values[19] = -0.907445; points.channels[2].values[19] = 0.000075;

//   SACModel *model = new SACModelCylinder ();
//   SAC *sac        = new RANSAC (model, 0.2);
//   model->setDataSet (&points);
//   EXPECT_EQ ((int)model->getCloud ()->points.size (), 20);

//   bool result = sac->computeModel ();
//   EXPECT_EQ (result, true);

//   std::vector<int> inliers = sac->getInliers ();
//   EXPECT_EQ ((int)inliers.size (), 20);

//   std::vector<double> coeff;
//   sac->computeCoefficients (coeff);
//   EXPECT_EQ ((int)coeff.size (), 7);
//   //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
//   EXPECT_NEAR (coeff[0], -0.5, 1e-1);
//   EXPECT_NEAR (coeff[1], 1.7, 1e-1);
//   EXPECT_NEAR (coeff[6], 0.5, 1e-1);

//   std::vector<double> coeff_ref;
//   sac->refineCoefficients (coeff_ref);
//   EXPECT_EQ ((int)coeff_ref.size (), 7);
//   EXPECT_NEAR (coeff_ref[6], 0.5, 1e-1);

//   int nr_points_left = sac->removeInliers ();
//   EXPECT_EQ (nr_points_left, 0);

//   delete model;
//   delete sac;
// }

// TEST (MSAC, SACModelCylinder)
// {
//   sensor_msgs::PointCloud points;
//   points.points.resize (20);

//   points.set_channels_size (3);
//   points.channels[0].name = "nx";
//   points.channels[1].name = "ny";
//   points.channels[2].name = "nz";
//   points.channels[0].values.resize (points.points.size ());
//   points.channels[1].values.resize (points.points.size ());
//   points.channels[2].values.resize (points.points.size ());

//   points.points[0].x = -0.499902; points.points[0].y = 2.199701; points.points[0].z = 0.000008;
//   points.points[1].x = -0.875397; points.points[1].y = 2.030177; points.points[1].z = 0.050104;
//   points.points[2].x = -0.995875; points.points[2].y = 1.635973; points.points[2].z = 0.099846;
//   points.points[3].x = -0.779523; points.points[3].y = 1.285527; points.points[3].z = 0.149961;
//   points.points[4].x = -0.373285; points.points[4].y = 1.216488; points.points[4].z = 0.199959;
//   points.points[5].x = -0.052893; points.points[5].y = 1.475973; points.points[5].z = 0.250101;
//   points.points[6].x = -0.036558; points.points[6].y = 1.887591; points.points[6].z = 0.299839;
//   points.points[7].x = -0.335048; points.points[7].y = 2.171994; points.points[7].z = 0.350001;
//   points.points[8].x = -0.745456; points.points[8].y = 2.135528; points.points[8].z = 0.400072;
//   points.points[9].x = -0.989282; points.points[9].y = 1.803311; points.points[9].z = 0.449983;
//   points.points[10].x = -0.900651; points.points[10].y = 1.400701; points.points[10].z = 0.500126;
//   points.points[11].x = -0.539658; points.points[11].y = 1.201468; points.points[11].z = 0.550079;
//   points.points[12].x = -0.151875; points.points[12].y = 1.340951; points.points[12].z = 0.599983;
//   points.points[13].x = -0.000724; points.points[13].y = 1.724373; points.points[13].z = 0.649882;
//   points.points[14].x = -0.188573; points.points[14].y = 2.090983; points.points[14].z = 0.699854;
//   points.points[15].x = -0.587925; points.points[15].y = 2.192257; points.points[15].z = 0.749956;
//   points.points[16].x = -0.927724; points.points[16].y = 1.958846; points.points[16].z = 0.800008;
//   points.points[17].x = -0.976888; points.points[17].y = 1.549655; points.points[17].z = 0.849970;
//   points.points[18].x = -0.702003; points.points[18].y = 1.242707; points.points[18].z = 0.899954;
//   points.points[19].x = -0.289916; points.points[19].y = 1.246296; points.points[19].z = 0.950075;

//   points.channels[0].values[0] = 0.000098;  points.channels[1].values[0] = 1.000098;  points.channels[2].values[0] = 0.000008;
//   points.channels[0].values[1] = -0.750891; points.channels[1].values[1] = 0.660413;  points.channels[2].values[1] = 0.000104;
//   points.channels[0].values[2] = -0.991765; points.channels[1].values[2] = -0.127949; points.channels[2].values[2] = -0.000154;
//   points.channels[0].values[3] = -0.558918; points.channels[1].values[3] = -0.829439; points.channels[2].values[3] = -0.000039;
//   points.channels[0].values[4] = 0.253627;  points.channels[1].values[4] = -0.967447; points.channels[2].values[4] = -0.000041;
//   points.channels[0].values[5] = 0.894105;  points.channels[1].values[5] = -0.447965; points.channels[2].values[5] = 0.000101;
//   points.channels[0].values[6] = 0.926852;  points.channels[1].values[6] = 0.375543;  points.channels[2].values[6] = -0.000161;
//   points.channels[0].values[7] = 0.329948;  points.channels[1].values[7] = 0.943941;  points.channels[2].values[7] = 0.000001;
//   points.channels[0].values[8] = -0.490966; points.channels[1].values[8] = 0.871203;  points.channels[2].values[8] = 0.000072;
//   points.channels[0].values[9] = -0.978507; points.channels[1].values[9] = 0.206425;  points.channels[2].values[9] = -0.000017;
//   points.channels[0].values[10] = -0.801227; points.channels[1].values[10] = -0.598534; points.channels[2].values[10] = 0.000126;
//   points.channels[0].values[11] = -0.079447; points.channels[1].values[11] = -0.996697; points.channels[2].values[11] = 0.000079;
//   points.channels[0].values[12] = 0.696154;  points.channels[1].values[12] = -0.717889; points.channels[2].values[12] = -0.000017;
//   points.channels[0].values[13] = 0.998685;  points.channels[1].values[13] = 0.048502;  points.channels[2].values[13] = -0.000118;
//   points.channels[0].values[14] = 0.622933;  points.channels[1].values[14] = 0.782133;  points.channels[2].values[14] = -0.000146;
//   points.channels[0].values[15] = -0.175948; points.channels[1].values[15] = 0.984480;  points.channels[2].values[15] = -0.000044;
//   points.channels[0].values[16] = -0.855476; points.channels[1].values[16] = 0.517824;  points.channels[2].values[16] = 0.000008;
//   points.channels[0].values[17] = -0.953769; points.channels[1].values[17] = -0.300571; points.channels[2].values[17] = -0.000030;
//   points.channels[0].values[18] = -0.404035; points.channels[1].values[18] = -0.914700; points.channels[2].values[18] = -0.000046;
//   points.channels[0].values[19] = 0.420154;  points.channels[1].values[19] = -0.907445; points.channels[2].values[19] = 0.000075;

//   SACModel *model = new SACModelCylinder ();
//   SAC *sac        = new MSAC (model, 0.2);
//   model->setDataSet (&points);
//   EXPECT_EQ ((int)model->getCloud ()->points.size (), 20);

//   bool result = sac->computeModel ();
//   EXPECT_EQ (result, true);

//   std::vector<int> inliers = sac->getInliers ();
//   EXPECT_EQ ((int)inliers.size (), 20);

//   std::vector<double> coeff;
//   sac->computeCoefficients (coeff);
//   EXPECT_EQ ((int)coeff.size (), 7);
//   //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
//   EXPECT_NEAR (coeff[0], -0.5, 1e-1);
//   EXPECT_NEAR (coeff[1], 1.7, 1e-1);
//   EXPECT_NEAR (coeff[6], 0.5, 1e-1);

//   std::vector<double> coeff_ref;
//   sac->refineCoefficients (coeff_ref);
//   EXPECT_EQ ((int)coeff_ref.size (), 7);
//   EXPECT_NEAR (coeff_ref[6], 0.5, 1e-1);

//   int nr_points_left = sac->removeInliers ();
//   EXPECT_EQ (nr_points_left, 0);

//   delete model;
//   delete sac;
// }

// TEST (MLESAC, SACModelCylinder)
// {
//   sensor_msgs::PointCloud points;
//   points.points.resize (20);

//   points.set_channels_size (3);
//   points.channels[0].name = "nx";
//   points.channels[1].name = "ny";
//   points.channels[2].name = "nz";
//   points.channels[0].values.resize (points.points.size ());
//   points.channels[1].values.resize (points.points.size ());
//   points.channels[2].values.resize (points.points.size ());

//   points.points[0].x = -0.499902; points.points[0].y = 2.199701; points.points[0].z = 0.000008;
//   points.points[1].x = -0.875397; points.points[1].y = 2.030177; points.points[1].z = 0.050104;
//   points.points[2].x = -0.995875; points.points[2].y = 1.635973; points.points[2].z = 0.099846;
//   points.points[3].x = -0.779523; points.points[3].y = 1.285527; points.points[3].z = 0.149961;
//   points.points[4].x = -0.373285; points.points[4].y = 1.216488; points.points[4].z = 0.199959;
//   points.points[5].x = -0.052893; points.points[5].y = 1.475973; points.points[5].z = 0.250101;
//   points.points[6].x = -0.036558; points.points[6].y = 1.887591; points.points[6].z = 0.299839;
//   points.points[7].x = -0.335048; points.points[7].y = 2.171994; points.points[7].z = 0.350001;
//   points.points[8].x = -0.745456; points.points[8].y = 2.135528; points.points[8].z = 0.400072;
//   points.points[9].x = -0.989282; points.points[9].y = 1.803311; points.points[9].z = 0.449983;
//   points.points[10].x = -0.900651; points.points[10].y = 1.400701; points.points[10].z = 0.500126;
//   points.points[11].x = -0.539658; points.points[11].y = 1.201468; points.points[11].z = 0.550079;
//   points.points[12].x = -0.151875; points.points[12].y = 1.340951; points.points[12].z = 0.599983;
//   points.points[13].x = -0.000724; points.points[13].y = 1.724373; points.points[13].z = 0.649882;
//   points.points[14].x = -0.188573; points.points[14].y = 2.090983; points.points[14].z = 0.699854;
//   points.points[15].x = -0.587925; points.points[15].y = 2.192257; points.points[15].z = 0.749956;
//   points.points[16].x = -0.927724; points.points[16].y = 1.958846; points.points[16].z = 0.800008;
//   points.points[17].x = -0.976888; points.points[17].y = 1.549655; points.points[17].z = 0.849970;
//   points.points[18].x = -0.702003; points.points[18].y = 1.242707; points.points[18].z = 0.899954;
//   points.points[19].x = -0.289916; points.points[19].y = 1.246296; points.points[19].z = 0.950075;

//   points.channels[0].values[0] = 0.000098;  points.channels[1].values[0] = 1.000098;  points.channels[2].values[0] = 0.000008;
//   points.channels[0].values[1] = -0.750891; points.channels[1].values[1] = 0.660413;  points.channels[2].values[1] = 0.000104;
//   points.channels[0].values[2] = -0.991765; points.channels[1].values[2] = -0.127949; points.channels[2].values[2] = -0.000154;
//   points.channels[0].values[3] = -0.558918; points.channels[1].values[3] = -0.829439; points.channels[2].values[3] = -0.000039;
//   points.channels[0].values[4] = 0.253627;  points.channels[1].values[4] = -0.967447; points.channels[2].values[4] = -0.000041;
//   points.channels[0].values[5] = 0.894105;  points.channels[1].values[5] = -0.447965; points.channels[2].values[5] = 0.000101;
//   points.channels[0].values[6] = 0.926852;  points.channels[1].values[6] = 0.375543;  points.channels[2].values[6] = -0.000161;
//   points.channels[0].values[7] = 0.329948;  points.channels[1].values[7] = 0.943941;  points.channels[2].values[7] = 0.000001;
//   points.channels[0].values[8] = -0.490966; points.channels[1].values[8] = 0.871203;  points.channels[2].values[8] = 0.000072;
//   points.channels[0].values[9] = -0.978507; points.channels[1].values[9] = 0.206425;  points.channels[2].values[9] = -0.000017;
//   points.channels[0].values[10] = -0.801227; points.channels[1].values[10] = -0.598534; points.channels[2].values[10] = 0.000126;
//   points.channels[0].values[11] = -0.079447; points.channels[1].values[11] = -0.996697; points.channels[2].values[11] = 0.000079;
//   points.channels[0].values[12] = 0.696154;  points.channels[1].values[12] = -0.717889; points.channels[2].values[12] = -0.000017;
//   points.channels[0].values[13] = 0.998685;  points.channels[1].values[13] = 0.048502;  points.channels[2].values[13] = -0.000118;
//   points.channels[0].values[14] = 0.622933;  points.channels[1].values[14] = 0.782133;  points.channels[2].values[14] = -0.000146;
//   points.channels[0].values[15] = -0.175948; points.channels[1].values[15] = 0.984480;  points.channels[2].values[15] = -0.000044;
//   points.channels[0].values[16] = -0.855476; points.channels[1].values[16] = 0.517824;  points.channels[2].values[16] = 0.000008;
//   points.channels[0].values[17] = -0.953769; points.channels[1].values[17] = -0.300571; points.channels[2].values[17] = -0.000030;
//   points.channels[0].values[18] = -0.404035; points.channels[1].values[18] = -0.914700; points.channels[2].values[18] = -0.000046;
//   points.channels[0].values[19] = 0.420154;  points.channels[1].values[19] = -0.907445; points.channels[2].values[19] = 0.000075;

//   SACModel *model = new SACModelCylinder ();
//   SAC *sac        = new MLESAC (model, 0.2);
//   model->setDataSet (&points);
//   EXPECT_EQ ((int)model->getCloud ()->points.size (), 20);

//   bool result = sac->computeModel ();
//   EXPECT_EQ (result, true);

//   std::vector<int> inliers = sac->getInliers ();
//   EXPECT_EQ ((int)inliers.size (), 20);

//   std::vector<double> coeff;
//   sac->computeCoefficients (coeff);
//   EXPECT_EQ ((int)coeff.size (), 7);
//   //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
//   EXPECT_NEAR (coeff[0], -0.5, 1e-1);
//   EXPECT_NEAR (coeff[1], 1.7, 1e-1);
//   EXPECT_NEAR (coeff[6], 0.5, 1e-1);

//   std::vector<double> coeff_ref;
//   sac->refineCoefficients (coeff_ref);
//   EXPECT_EQ ((int)coeff_ref.size (), 7);
//   EXPECT_NEAR (coeff_ref[6], 0.5, 1e-1);

//   int nr_points_left = sac->removeInliers ();
//   EXPECT_EQ (nr_points_left, 0);

//   delete model;
//   delete sac;
// }

// TEST (RRANSAC, SACModelCylinder)
// {
//   sensor_msgs::PointCloud points;
//   points.points.resize (20);

//   points.set_channels_size (3);
//   points.channels[0].name = "nx";
//   points.channels[1].name = "ny";
//   points.channels[2].name = "nz";
//   points.channels[0].values.resize (points.points.size ());
//   points.channels[1].values.resize (points.points.size ());
//   points.channels[2].values.resize (points.points.size ());

//   points.points[0].x = -0.499902; points.points[0].y = 2.199701; points.points[0].z = 0.000008;
//   points.points[1].x = -0.875397; points.points[1].y = 2.030177; points.points[1].z = 0.050104;
//   points.points[2].x = -0.995875; points.points[2].y = 1.635973; points.points[2].z = 0.099846;
//   points.points[3].x = -0.779523; points.points[3].y = 1.285527; points.points[3].z = 0.149961;
//   points.points[4].x = -0.373285; points.points[4].y = 1.216488; points.points[4].z = 0.199959;
//   points.points[5].x = -0.052893; points.points[5].y = 1.475973; points.points[5].z = 0.250101;
//   points.points[6].x = -0.036558; points.points[6].y = 1.887591; points.points[6].z = 0.299839;
//   points.points[7].x = -0.335048; points.points[7].y = 2.171994; points.points[7].z = 0.350001;
//   points.points[8].x = -0.745456; points.points[8].y = 2.135528; points.points[8].z = 0.400072;
//   points.points[9].x = -0.989282; points.points[9].y = 1.803311; points.points[9].z = 0.449983;
//   points.points[10].x = -0.900651; points.points[10].y = 1.400701; points.points[10].z = 0.500126;
//   points.points[11].x = -0.539658; points.points[11].y = 1.201468; points.points[11].z = 0.550079;
//   points.points[12].x = -0.151875; points.points[12].y = 1.340951; points.points[12].z = 0.599983;
//   points.points[13].x = -0.000724; points.points[13].y = 1.724373; points.points[13].z = 0.649882;
//   points.points[14].x = -0.188573; points.points[14].y = 2.090983; points.points[14].z = 0.699854;
//   points.points[15].x = -0.587925; points.points[15].y = 2.192257; points.points[15].z = 0.749956;
//   points.points[16].x = -0.927724; points.points[16].y = 1.958846; points.points[16].z = 0.800008;
//   points.points[17].x = -0.976888; points.points[17].y = 1.549655; points.points[17].z = 0.849970;
//   points.points[18].x = -0.702003; points.points[18].y = 1.242707; points.points[18].z = 0.899954;
//   points.points[19].x = -0.289916; points.points[19].y = 1.246296; points.points[19].z = 0.950075;

//   points.channels[0].values[0] = 0.000098;  points.channels[1].values[0] = 1.000098;  points.channels[2].values[0] = 0.000008;
//   points.channels[0].values[1] = -0.750891; points.channels[1].values[1] = 0.660413;  points.channels[2].values[1] = 0.000104;
//   points.channels[0].values[2] = -0.991765; points.channels[1].values[2] = -0.127949; points.channels[2].values[2] = -0.000154;
//   points.channels[0].values[3] = -0.558918; points.channels[1].values[3] = -0.829439; points.channels[2].values[3] = -0.000039;
//   points.channels[0].values[4] = 0.253627;  points.channels[1].values[4] = -0.967447; points.channels[2].values[4] = -0.000041;
//   points.channels[0].values[5] = 0.894105;  points.channels[1].values[5] = -0.447965; points.channels[2].values[5] = 0.000101;
//   points.channels[0].values[6] = 0.926852;  points.channels[1].values[6] = 0.375543;  points.channels[2].values[6] = -0.000161;
//   points.channels[0].values[7] = 0.329948;  points.channels[1].values[7] = 0.943941;  points.channels[2].values[7] = 0.000001;
//   points.channels[0].values[8] = -0.490966; points.channels[1].values[8] = 0.871203;  points.channels[2].values[8] = 0.000072;
//   points.channels[0].values[9] = -0.978507; points.channels[1].values[9] = 0.206425;  points.channels[2].values[9] = -0.000017;
//   points.channels[0].values[10] = -0.801227; points.channels[1].values[10] = -0.598534; points.channels[2].values[10] = 0.000126;
//   points.channels[0].values[11] = -0.079447; points.channels[1].values[11] = -0.996697; points.channels[2].values[11] = 0.000079;
//   points.channels[0].values[12] = 0.696154;  points.channels[1].values[12] = -0.717889; points.channels[2].values[12] = -0.000017;
//   points.channels[0].values[13] = 0.998685;  points.channels[1].values[13] = 0.048502;  points.channels[2].values[13] = -0.000118;
//   points.channels[0].values[14] = 0.622933;  points.channels[1].values[14] = 0.782133;  points.channels[2].values[14] = -0.000146;
//   points.channels[0].values[15] = -0.175948; points.channels[1].values[15] = 0.984480;  points.channels[2].values[15] = -0.000044;
//   points.channels[0].values[16] = -0.855476; points.channels[1].values[16] = 0.517824;  points.channels[2].values[16] = 0.000008;
//   points.channels[0].values[17] = -0.953769; points.channels[1].values[17] = -0.300571; points.channels[2].values[17] = -0.000030;
//   points.channels[0].values[18] = -0.404035; points.channels[1].values[18] = -0.914700; points.channels[2].values[18] = -0.000046;
//   points.channels[0].values[19] = 0.420154;  points.channels[1].values[19] = -0.907445; points.channels[2].values[19] = 0.000075;

//   SACModel *model = new SACModelCylinder ();
//   SAC *sac        = new RRANSAC (model, 0.2);
//   reinterpret_cast<RRANSAC*>(sac)->setFractionNrPretest (50);
//   model->setDataSet (&points);
//   EXPECT_EQ ((int)model->getCloud ()->points.size (), 20);

//   bool result = sac->computeModel ();
//   EXPECT_EQ (result, true);

//   std::vector<int> inliers = sac->getInliers ();
//   EXPECT_EQ ((int)inliers.size (), 20);

//   std::vector<double> coeff;
//   sac->computeCoefficients (coeff);
//   EXPECT_EQ ((int)coeff.size (), 7);
//   //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
//   EXPECT_NEAR (coeff[0], -0.5, 1e-1);
//   EXPECT_NEAR (coeff[1], 1.7, 1e-1);
//   EXPECT_NEAR (coeff[6], 0.5, 1e-1);

//   std::vector<double> coeff_ref;
//   sac->refineCoefficients (coeff_ref);
//   EXPECT_EQ ((int)coeff_ref.size (), 7);
//   EXPECT_NEAR (coeff_ref[6], 0.5, 1e-1);

//   int nr_points_left = sac->removeInliers ();
//   EXPECT_EQ (nr_points_left, 0);

//   delete model;
//   delete sac;
// }

// TEST (RMSAC, SACModelCylinder)
// {
//   sensor_msgs::PointCloud points;
//   points.points.resize (20);

//   points.set_channels_size (3);
//   points.channels[0].name = "nx";
//   points.channels[1].name = "ny";
//   points.channels[2].name = "nz";
//   points.channels[0].values.resize (points.points.size ());
//   points.channels[1].values.resize (points.points.size ());
//   points.channels[2].values.resize (points.points.size ());

//   points.points[0].x = -0.499902; points.points[0].y = 2.199701; points.points[0].z = 0.000008;
//   points.points[1].x = -0.875397; points.points[1].y = 2.030177; points.points[1].z = 0.050104;
//   points.points[2].x = -0.995875; points.points[2].y = 1.635973; points.points[2].z = 0.099846;
//   points.points[3].x = -0.779523; points.points[3].y = 1.285527; points.points[3].z = 0.149961;
//   points.points[4].x = -0.373285; points.points[4].y = 1.216488; points.points[4].z = 0.199959;
//   points.points[5].x = -0.052893; points.points[5].y = 1.475973; points.points[5].z = 0.250101;
//   points.points[6].x = -0.036558; points.points[6].y = 1.887591; points.points[6].z = 0.299839;
//   points.points[7].x = -0.335048; points.points[7].y = 2.171994; points.points[7].z = 0.350001;
//   points.points[8].x = -0.745456; points.points[8].y = 2.135528; points.points[8].z = 0.400072;
//   points.points[9].x = -0.989282; points.points[9].y = 1.803311; points.points[9].z = 0.449983;
//   points.points[10].x = -0.900651; points.points[10].y = 1.400701; points.points[10].z = 0.500126;
//   points.points[11].x = -0.539658; points.points[11].y = 1.201468; points.points[11].z = 0.550079;
//   points.points[12].x = -0.151875; points.points[12].y = 1.340951; points.points[12].z = 0.599983;
//   points.points[13].x = -0.000724; points.points[13].y = 1.724373; points.points[13].z = 0.649882;
//   points.points[14].x = -0.188573; points.points[14].y = 2.090983; points.points[14].z = 0.699854;
//   points.points[15].x = -0.587925; points.points[15].y = 2.192257; points.points[15].z = 0.749956;
//   points.points[16].x = -0.927724; points.points[16].y = 1.958846; points.points[16].z = 0.800008;
//   points.points[17].x = -0.976888; points.points[17].y = 1.549655; points.points[17].z = 0.849970;
//   points.points[18].x = -0.702003; points.points[18].y = 1.242707; points.points[18].z = 0.899954;
//   points.points[19].x = -0.289916; points.points[19].y = 1.246296; points.points[19].z = 0.950075;

//   points.channels[0].values[0] = 0.000098;  points.channels[1].values[0] = 1.000098;  points.channels[2].values[0] = 0.000008;
//   points.channels[0].values[1] = -0.750891; points.channels[1].values[1] = 0.660413;  points.channels[2].values[1] = 0.000104;
//   points.channels[0].values[2] = -0.991765; points.channels[1].values[2] = -0.127949; points.channels[2].values[2] = -0.000154;
//   points.channels[0].values[3] = -0.558918; points.channels[1].values[3] = -0.829439; points.channels[2].values[3] = -0.000039;
//   points.channels[0].values[4] = 0.253627;  points.channels[1].values[4] = -0.967447; points.channels[2].values[4] = -0.000041;
//   points.channels[0].values[5] = 0.894105;  points.channels[1].values[5] = -0.447965; points.channels[2].values[5] = 0.000101;
//   points.channels[0].values[6] = 0.926852;  points.channels[1].values[6] = 0.375543;  points.channels[2].values[6] = -0.000161;
//   points.channels[0].values[7] = 0.329948;  points.channels[1].values[7] = 0.943941;  points.channels[2].values[7] = 0.000001;
//   points.channels[0].values[8] = -0.490966; points.channels[1].values[8] = 0.871203;  points.channels[2].values[8] = 0.000072;
//   points.channels[0].values[9] = -0.978507; points.channels[1].values[9] = 0.206425;  points.channels[2].values[9] = -0.000017;
//   points.channels[0].values[10] = -0.801227; points.channels[1].values[10] = -0.598534; points.channels[2].values[10] = 0.000126;
//   points.channels[0].values[11] = -0.079447; points.channels[1].values[11] = -0.996697; points.channels[2].values[11] = 0.000079;
//   points.channels[0].values[12] = 0.696154;  points.channels[1].values[12] = -0.717889; points.channels[2].values[12] = -0.000017;
//   points.channels[0].values[13] = 0.998685;  points.channels[1].values[13] = 0.048502;  points.channels[2].values[13] = -0.000118;
//   points.channels[0].values[14] = 0.622933;  points.channels[1].values[14] = 0.782133;  points.channels[2].values[14] = -0.000146;
//   points.channels[0].values[15] = -0.175948; points.channels[1].values[15] = 0.984480;  points.channels[2].values[15] = -0.000044;
//   points.channels[0].values[16] = -0.855476; points.channels[1].values[16] = 0.517824;  points.channels[2].values[16] = 0.000008;
//   points.channels[0].values[17] = -0.953769; points.channels[1].values[17] = -0.300571; points.channels[2].values[17] = -0.000030;
//   points.channels[0].values[18] = -0.404035; points.channels[1].values[18] = -0.914700; points.channels[2].values[18] = -0.000046;
//   points.channels[0].values[19] = 0.420154;  points.channels[1].values[19] = -0.907445; points.channels[2].values[19] = 0.000075;

//   SACModel *model = new SACModelCylinder ();
//   SAC *sac        = new RMSAC (model, 0.2);
//   reinterpret_cast<RMSAC*>(sac)->setFractionNrPretest (50);
//   model->setDataSet (&points);
//   EXPECT_EQ ((int)model->getCloud ()->points.size (), 20);

//   bool result = sac->computeModel ();
//   EXPECT_EQ (result, true);

//   std::vector<int> inliers = sac->getInliers ();
//   EXPECT_EQ ((int)inliers.size (), 20);

//   std::vector<double> coeff;
//   sac->computeCoefficients (coeff);
//   EXPECT_EQ ((int)coeff.size (), 7);
//   //printf ("Cylinder coefficients: %f %f %f %f %f %f %f\n", coeff[0], coeff[1], coeff[2], coeff[3], coeff[4], coeff[5], coeff[6]);
//   EXPECT_NEAR (coeff[0], -0.5, 1e-1);
//   EXPECT_NEAR (coeff[1], 1.7, 1e-1);
//   EXPECT_NEAR (coeff[6], 0.5, 1e-1);

//   std::vector<double> coeff_ref;
//   sac->refineCoefficients (coeff_ref);
//   EXPECT_EQ ((int)coeff_ref.size (), 7);
//   EXPECT_NEAR (coeff_ref[6], 0.5, 1e-1);

//   int nr_points_left = sac->removeInliers ();
//   EXPECT_EQ (nr_points_left, 0);

//   delete model;
//   delete sac;
// }

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init(argc, argv, "cylinder_fit");
  ros::NodeHandle n;
  CylinderFit c(n);
  c.spin();
  return 0;
}
/* ]--- */
