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



//#include "pcl_ros/publisher.h"
//#include "pcl_ros/subscriber.h"

//#include <tf/transform_listener.h>


// ros dependencies
#include "ros/ros.h"

//// terminal tools dependecies
//#include "terminal_tools/parse.h"

// pcl dependencies
#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
//#include "pcl_ros/transforms.h"

#include <pcl/features/normal_3d.h>
//#include "pcl/surface/convex_hull.h"

//#include "pcl/filters/voxel_grid.h"
//#include "pcl/filters/passthrough.h"
#include "pcl/filters/extract_indices.h"
//#include "pcl/filters/project_inliers.h"
//#include "pcl/filters/statistical_outlier_removal.h"

#include "pcl/sample_consensus/method_types.h"
//#include "pcl/sample_consensus/impl/ransac.hpp"

#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"
//#include "pcl/segmentation/extract_polygonal_prism_data.h"

//// pcl visualization dependencies
//#include "pcl_visualization/pcl_visualizer.h"

//// pcl ias sample consensus dependencies
//#include "pcl_ias_sample_consensus/pcl_sac_model_orientation.h"

//// dos pcl dependencies
//#include "dos_pcl/segmentation/door_detection_by_color_and_fixture.h"
//#include "dos_pcl_ros/segmentation/door_detection_by_color_and_fixture.h"




using namespace std;



////////////////////////////////////////////////////////////////////////////////
float getRGB (float r, float g, float b)
{
  int res = (int (r * 255) << 16) | (int (g * 255) << 8) | int (b * 255);
  double rgb = *(float*)(&res);
  return (rgb);
}



////////////////////////////////////////////////////////////////////////////////
static unsigned stepRGBA = 100;
inline double* randRGB (double min=0.2, double max=2.8)
{
  double sum;
  double* rgb = new double[3];
  do
  {
    sum = 0;
    rgb[0] = (rand ()%stepRGBA) / (double)stepRGBA;
    while ((rgb[1] = (rand ()%stepRGBA) / (double)stepRGBA) == rgb[0]);
    while (((rgb[2] = (rand ()%stepRGBA) / (double)stepRGBA) == rgb[0]) && (rgb[2] == rgb[1]));
    sum = rgb[0] + rgb[1] + rgb[2];
  } while (sum <= min || sum >= max);
  return rgb;
}



////////////////////////////////////////////////////////////////////////////////
bool concatenateToPointCloud (const sensor_msgs::PointCloud2 &cloud1, sensor_msgs::PointCloud2 &cloud2)
{
  if (cloud2.fields.size () != cloud1.fields.size ())
    return (false);

  for (size_t i = 0; i < cloud2.fields.size (); ++i)
    if (cloud2.fields[i].name != cloud1.fields[i].name)
      return (false);

  // Copy <cloud1> into <cloud2>
  size_t nrpts = cloud2.data.size ();
  cloud2.data.resize (nrpts + cloud1.data.size ());
  memcpy (&cloud2.data [nrpts], &cloud1.data [0], cloud1.data.size ());

  // Update <width> and <height>
  cloud2.width  = cloud2.width * cloud2.height + cloud1.width * cloud1.height;
  cloud2.height = 1;

  if (!cloud2.is_dense || !cloud1.is_dense)
    cloud2.is_dense = false;
  else
    cloud2.is_dense = true;

  return (true);
}



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
class ClusterObjectsOnRacks
{
  protected:
    ros::NodeHandle nh_;

  public:
    int queue_size_;
    string output_cloud_topic_, input_cloud_topic_, to_frame_;

    ros::Subscriber sub_1_;
    ros::Subscriber sub_2_;
    ros::Subscriber sub_3_;
    ros::Subscriber sub_4_;
    ros::Publisher pub_;
    //tf::TransformListener tf_;

    sensor_msgs::PointCloud2 output_cloud_;
    ////////////////////////////////////////////////////////////////////////////////
  //  ClusterObjectsOnRacks (ros::NodeHandle &n) : nh_(n), input_cloud_topic_ ("/input"), output_cloud_topic_ ("/output"), queue_size_ (100)
    ClusterObjectsOnRacks (ros::NodeHandle &n) : nh_(n), queue_size_ (100) 
    {
      // Maximum number of outgoing messages to be queued for delivery to subscribers = 1
      nh_.param("input_cloud_topic", input_cloud_topic_, std::string("input"));
      nh_.param("output_cloud_topic", output_cloud_topic_, std::string("output"));
      //output_cloud_topic_ = input_cloud_topic_ + "_transformed";
      nh_.param("to_frame", to_frame_, std::string("/map"));





      //sub_ = nh_.subscribe (input_cloud_topic_, queue_size_, &ClusterObjectsOnRacks::Clustering, this);
      //ROS_INFO ("[ClusterObjectsOnRacks:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());


      // TODO Concatenate all urdf racks topics and work only on that one

      // TODO Search topics for urdf rack and do everything in a for statement

      sub_1_ = nh_.subscribe ("/urdf_cloud_filter/objects_on_top_rack", queue_size_, &ClusterObjectsOnRacks::Clustering, this);
      ROS_INFO ("[ClusterObjectsOnRacks:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());
   
      sub_2_ = nh_.subscribe ("/urdf_cloud_filter/objects_on_middle_rack", queue_size_, &ClusterObjectsOnRacks::Clustering, this);
      ROS_INFO ("[ClusterObjectsOnRacks:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());

      sub_3_ = nh_.subscribe ("/urdf_cloud_filter/objects_on_bottom_rack", queue_size_, &ClusterObjectsOnRacks::Clustering, this);
      ROS_INFO ("[ClusterObjectsOnRacks:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());

      sub_4_ = nh_.subscribe ("/urdf_cloud_filter/objects_on_skirting_rack", queue_size_, &ClusterObjectsOnRacks::Clustering, this);
      ROS_INFO ("[ClusterObjectsOnRacks:] Listening for incoming data on topic %s", nh_.resolveName (input_cloud_topic_).c_str ());



      pub_ = nh_.advertise<sensor_msgs::PointCloud2>(output_cloud_topic_, queue_size_, true);
      ROS_INFO ("[ClusterObjectsOnRacks:] Will be publishing data on topic %s.", nh_.resolveName (output_cloud_topic_).c_str ());
    }


//    typedef sensor_msgs::PointCloud2 PointCloud2;
//    typedef PointCloud2::Ptr PointCloud2Ptr;
//    typedef PointCloud2::ConstPtr PointCloud2ConstPtr;


//pcl::PointCloud<pcl::PointXYZ>::Ptr

    ////////////////////////////////////////////////////////////////////////////////
    void Clustering (const sensor_msgs::PointCloud2::ConstPtr &input)
    {

      // Declare the working cloud
      //pcl::PointCloud<pcl::PointXYZ> cloud;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ> ());

      // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
      pcl::fromROSMsg (*input, *cloud);





      // // Point cloud of normals
      // pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal> ());
      // // Build kd-tree structure for normals
      // pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

      // // Create object for normal estimation
      // pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
      // // Provide pointer to the search method
      // ne.setSearchMethod (tree);
      // // Set for which point cloud to compute the normals
      // ne.setInputCloud (cloud);
      // // Set number of k nearest neighbors to use
      // ne.setKSearch (50);
      // // Estimate the normals
      // ne.compute (*normals);

      // ROS_INFO ("[ClusterObjectsOnRacks:] Working cloud has %d points", (int) cloud->points.size());
      // ROS_INFO ("[ClusterObjectsOnRacks:] With %d normals of course", (int) normals->points.size());


      // // Create the segmentation object
      // pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sfn;

      // // Declare the variable which are needed
      // Eigen::Vector3f axis = Eigen::Vector3f (1.0, 0.0, 0.0); 
      // pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
      // pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());

      // // Segmentation's Parameters
      // double epsilon_angle = 0.25; /// [radians]
      // double plane_threshold = 0.050; /// [meters]
      // int minimum_plane_inliers = 10000; /// [points]
      // int maximum_plane_iterations = 1000; /// [iterations]
      // double normal_distance_weight = 0.05; /// [percentage] 

      // // Set all the parameters for segmenting planes
      // sfn.setMethodType (pcl::SAC_RANSAC);
      // sfn.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      // sfn.setModelType (pcl::SACMODEL_PLANE);

      // sfn.setAxis (axis);
      // sfn.setInputCloud (cloud);
      // sfn.setInputNormals (normals);
      // sfn.setEpsAngle (epsilon_angle);
      // sfn.setOptimizeCoefficients (true);
      // sfn.setDistanceThreshold (plane_threshold);
      // sfn.setMaxIterations (maximum_plane_iterations);
      // sfn.setNormalDistanceWeight (normal_distance_weight);

      // // Obtain the plane inliers and coefficients
      // sfn.segment (*plane_inliers, *plane_coefficients);





      /*

      // Create the segmentation object
      //pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sfn;
      pcl::SACSegmentation<pcl::PointXYZ> sfn;

      // Declare the variable which are needed
      //Eigen::Vector3f axis = Eigen::Vector3f (1.0, 0.0, 0.0); 
      pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ());
      pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ());

      // Segmentation's Parameters
      //double epsilon_angle = 0.25; /// [radians]
      double plane_threshold = 0.050; /// [meters]
      int minimum_plane_inliers = 10000; /// [points]
      int maximum_plane_iterations = 1000; /// [iterations]
      //double normal_distance_weight = 0.05; /// [percentage] 

      // Set all the parameters for segmenting planes
      sfn.setMethodType (pcl::SAC_RANSAC);
      //sfn.setModelType (pcl::SACMODEL_NORMAL_PLANE);
      sfn.setModelType (pcl::SACMODEL_PLANE);

      //sfn.setAxis (axis);
      sfn.setInputCloud (cloud);
      //sfn.setInputNormals (normals);
      //sfn.setEpsAngle (epsilon_angle);
      sfn.setOptimizeCoefficients (true);
      sfn.setDistanceThreshold (plane_threshold);
      sfn.setMaxIterations (maximum_plane_iterations);
      //sfn.setNormalDistanceWeight (normal_distance_weight);

      // Obtain the plane inliers and coefficients
      sfn.segment (*plane_inliers, *plane_coefficients);



      */


      // if ( minimum_plane_inliers < (int) plane_inliers->indices.size () )
      // {

      //   ROS_INFO ("[ClusterObjectsOnRacks:] Plane has %5d inliers with parameters A = %f B = %f C = %f and D = %f found in maximum %d iterations", (int) plane_inliers->indices.size (), 
      //       plane_coefficients->values [0], plane_coefficients->values [1], plane_coefficients->values [2], plane_coefficients->values [3], maximum_plane_iterations);

      //   // Point cloud of plane inliers
      //   pcl::PointCloud<pcl::PointXYZ>::Ptr plane_inliers_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

      //   // Extract the circular inliers from the input cloud
      //   pcl::ExtractIndices<pcl::PointXYZ> extraction_of_plane_inliers;
      //   // Set point cloud from where to extract
      //   extraction_of_plane_inliers.setInputCloud (cloud);
      //   // Set which indices to extract
      //   extraction_of_plane_inliers.setIndices (plane_inliers);
      //   // Return the points which represent the inliers
      //   extraction_of_plane_inliers.setNegative (false);
      //   // Call the extraction function
      //   extraction_of_plane_inliers.filter (*plane_inliers_cloud);
      //   // Return the remaining points of inliers
      //   extraction_of_plane_inliers.setNegative (true);
      //   // Call the extraction function
      //   extraction_of_plane_inliers.filter (*cloud);

      //   ROS_INFO ("[ClusterObjectsOnRacks:] Remaning working cloud has %d points", (int) cloud->points.size());
      //   ROS_INFO ("[ClusterObjectsOnRacks:] And the cloud with the plane's inliers has %d points", (int) plane_inliers_cloud->points.size());

      // }







      // Clustering's Parameters
      int minimum_size_of_plane_cluster = 250; /// [points]
      double plane_inliers_clustering_tolerance = 0.010; /// [meters]

      // Vector of clusters from inliers
      std::vector<pcl::PointIndices> plane_clusters;
      // Build kd-tree structure for clusters
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr plane_clusters_tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

      // Instantiate cluster extraction object
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> clustering_of_plane_inliers;
      // Set as input the cloud of circle inliers
      clustering_of_plane_inliers.setInputCloud (cloud);
      // Radius of the connnectivity threshold
      clustering_of_plane_inliers.setClusterTolerance (plane_inliers_clustering_tolerance);
      // Minimum size of clusters
      clustering_of_plane_inliers.setMinClusterSize (minimum_size_of_plane_cluster);
      // Provide pointer to the search method
      clustering_of_plane_inliers.setSearchMethod (plane_clusters_tree);
      // Call the extraction function
      clustering_of_plane_inliers.extract (plane_clusters);

      ROS_INFO ("[ClusterObjectsOnRacks:] There are %d objects on the rack", (int) plane_clusters.size());










      // Point clouds which represent the clusters of the plane inliers
      std::vector<pcl::PointCloud<pcl::PointXYZ>::ConstPtr> plane_clusters_clouds;

      for (int c = 0; c < (int) plane_clusters.size(); c++)
      {
        // Local variables
        pcl::PointIndices::Ptr pointer_of_plane_cluster (new pcl::PointIndices (plane_clusters.at(c)));
        pcl::PointCloud<pcl::PointXYZ>::Ptr cluster (new pcl::PointCloud<pcl::PointXYZ>);

        // Extract the circular inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZ> extraction_of_plane_clusters;
        // Set point cloud from where to extract
        
        extraction_of_plane_clusters.setInputCloud (cloud);
        // Set which indices to extract
        extraction_of_plane_clusters.setIndices (pointer_of_plane_cluster);
        // Return the points which represent the inliers
        extraction_of_plane_clusters.setNegative (false);
        // Call the extraction function
        extraction_of_plane_clusters.filter (*cluster);

        // Save cluster
        plane_clusters_clouds.push_back (cluster);
      }

      ROS_INFO ("[ClusterObjectsOnRacks:] There are %d objects on the rack", (int) plane_clusters_clouds.size());



      // Point clouds which represent the clusters of the plane inliers
      std::vector<sensor_msgs::PointCloud2> plane_clusters_msgs;

      sensor_msgs::PointCloud2 cluster_msg;

      // Construct the PointXYZRGB cloud for rviz visualization
      for (int c = 0; c < (int) plane_clusters_clouds.size(); c++)
      {

        //double R = ((double) rand() / (RAND_MAX + 1));
        //double G = ((double) rand() / (RAND_MAX + 1));
        //double B = ((double) rand() / (RAND_MAX + 1));

        double* drgb = randRGB();

        pcl::PointCloud<pcl::PointXYZRGB> cloud_a;
        cloud_a.points.resize (plane_clusters_clouds.at(c)->size());
        for (unsigned int i = 0; i < plane_clusters_clouds.at(c)->size(); i++)
        {
          cloud_a.points[i].x = plane_clusters_clouds.at(c)->points[i].x;
          cloud_a.points[i].y = plane_clusters_clouds.at(c)->points[i].y;
          cloud_a.points[i].z = plane_clusters_clouds.at(c)->points[i].z;

          //cloud_a.points[i].rgb = getRGB (1.0, 0.0, 0.0);
          //cerr << c << " :: " <<  R << " " << G << " " << B << endl ; 
          //cloud_a.points[i].rgb = getRGB (R, G, B);
          
          cloud_a.points[i].rgb = getRGB (drgb[0], drgb[1], drgb[2]);

        }

        cloud_a.header.frame_id = to_frame_;
        cloud_a.header.stamp = ros::Time::now();
        //cerr << " STAMP: " << cloud_a.header.stamp << endl ;
        cloud_a.header.seq = c;

        ROS_INFO (" [] %d ", (int) cloud_a.size());
        pcl::toROSMsg (cloud_a, cluster_msg);

        ROS_INFO (" [] %d ", (int) cluster_msg.width);
        ROS_INFO (" [] %d ", (int) cluster_msg.height);
        ROS_INFO (" [] %d ", (int) cluster_msg.data.size());

        cluster_msg.header.frame_id = to_frame_;

        plane_clusters_msgs.push_back (cluster_msg);

        //pub_.publish (cluster_msg);

        //pcl::PointCloud<pcl::PointXYZ>::Ptr _output_cloud_ (new pcl::PointCloud<pcl::PointXYZ> ());
        //pcl::fromROSMsg (output_cloud_, *_output_cloud_);
        //pcl::toROSMsg (output_cloud_, *_output_cloud_);

/*
        sensor_msgs::PointCloud2 pivot_output_cloud_;
        if (c == 0)
          pivot_output_cloud_ = cluster_msg;
        else
          pivot_output_cloud_ = output_cloud_;
        pcl::concatenatePointCloud (cluster_msg, pivot_output_cloud_, output_cloud_);
        //pcl::concatenatePointCloud (cluster_msg, cluster_msg, output_cloud_);
        cerr << " at c :  " << c << " is " << output_cloud_.data.size () << " points " << endl ;
*/

/*
        if (c == 0)
          output_cloud_ = cluster_msg;
        concatenateToPointCloud (cluster_msg, output_cloud_);
        cerr << " at c :  " << c << " is " << output_cloud_.data.size () << " points " << endl ;
*/

      }


      // Concatenate sensor_msgs of clusters into one
      for (int c = 1; c < (int) plane_clusters_msgs.size(); c++)
      {
        if (c == 1)
          pcl::concatenatePointCloud (plane_clusters_msgs.at(c-1), plane_clusters_msgs.at(c), output_cloud_);
        else
          pcl::concatenatePointCloud (output_cloud_, plane_clusters_msgs.at(c), output_cloud_);
      }

      // Publish the concatenated sensor_msgs 
      pub_.publish (output_cloud_);

    }
};



////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
int main (int argc, char** argv)
{
  ros::init (argc, argv, "cluster_objects_on_racks");

  ros::NodeHandle n("~");

  ClusterObjectsOnRacks coor(n);

  ros::spin ();

  return (0);
}
