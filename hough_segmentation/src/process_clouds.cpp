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



// ros dependencies
#include "ros/ros.h"

// terminal tools dependecies
#include "terminal_tools/parse.h"

// pcl dependencies
#include "pcl/io/pcd_io.h"
#include "pcl/features/normal_3d.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/sac_model_circle.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

// pcl visualization dependencies
#include "pcl_visualization/pcl_visualizer.h"



typedef pcl::PointXYZINormal PointT;



bool project = true; 

bool verbose = false;
int size_of_points = 3;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of the method. Processing point clouds. Projection in 2D. Computing of normals in 2D. Refinement of normals in 2D.
 */
int main (int argc, char** argv)
{
 
  // --------------------------------------------------------------- //
  // ------------------ Check and parse arguments ------------------ //
  // --------------------------------------------------------------- //

  // Argument check and info about
  if (argc < 2)
  {
    ROS_INFO (" ");
    ROS_INFO ("Syntax is: %s <input>.pcd <options>", argv[0]);
    ROS_INFO ("  where <options> are:");
    ROS_INFO ("    -project B                   = Yes/No to projecting points in xOy plane");
    ROS_INFO (" ");
    ROS_INFO ("    -verbose B                   = Yes/No to printing of useful messages");
    ROS_INFO ("    -size_of_points D            = Size of points for 3D viewer");
    ROS_INFO (" ");
    return (-1);
  }

  // Take only the first .pcd file into account
  std::vector<int> pFileIndicesPCD = terminal_tools::parse_file_extension_argument (argc, argv, ".pcd");
  if (pFileIndicesPCD.size () == 0)
  {
    ROS_INFO ("No .pcd file given as input!");
    return (-1);
  }

  terminal_tools::parse_argument (argc, argv,   "-project", project);

  terminal_tools::parse_argument (argc, argv, "-verbose", verbose);
  terminal_tools::parse_argument (argc, argv, "-size_of_points", size_of_points);

  // ----------------------------------------------------- //
  // ------------------ Initializations ------------------ //
  // ----------------------------------------------------- //

  // Initialize random number generator
  srand (time(0));

  // Initialize ros time
  ros::Time::init();

  // Declare the timer
  terminal_tools::TicToc tt;

  // Starting timer
  tt.tic ();

  if ( verbose )
  {
    // Displaying when the timer starts
    ROS_WARN ("Timer started !");
  }

  // ----------------------------------------------------------- //
  // ------------------ Load point cloud data ------------------ //
  // ----------------------------------------------------------- //

  // Input point cloud data
  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT> ());

  // Load point cloud data
  if (pcl::io::loadPCDFile (argv [pFileIndicesPCD [0]], *input_cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv [pFileIndicesPCD [0]]);
    return (-1);
  }

  if ( verbose )
  {
    ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*input_cloud).c_str ());
  }

  // Working point cloud data 
  pcl::PointCloud<PointT>::Ptr working_cloud (new pcl::PointCloud<PointT> ());

  // Update working point cloud
  *working_cloud = *input_cloud;

  // ---------------------------------------------------------------- //
  // ------------------ Visualize point cloud data ------------------ //
  // ---------------------------------------------------------------- //

  // Open a 3D viewer
  pcl_visualization::PCLVisualizer viewer ("3D VIEWER");
  // Set the background of viewer
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  // Add system coordiante to viewer
  viewer.addCoordinateSystem (1.0f);
  // Parse the camera settings and update the internal camera
  viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render
  viewer.updateCamera ();

  // Add the point cloud data
  viewer.addPointCloud (*working_cloud, "INPUT");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, "INPUT"); 
  // And wait until Q key is pressed
  viewer.spin ();

  // -------------------------------------------------------------- //
  // ------------------ Filtering of Point Cloud ------------------ //
  // -------------------------------------------------------------- //

  // Filtered point cloud
  pcl::PointCloud<PointT>::Ptr filtered_cloud (new pcl::PointCloud<PointT> ());

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<PointT> sor;
  // Set which point cloud to filter
  sor.setInputCloud (working_cloud);
  // Set number of points for mean distance estimation
  sor.setMeanK (25);
  // Set the standard deviation multiplier threshold
  sor.setStddevMulThresh (1.0);
  // Call the filtering method
  sor.filter (*working_cloud);

  if ( verbose )
  {
    ROS_INFO ("Statistical Outlier Removal ! Before: %d points | After: %d points | Filtered: %d points",
              (int) input_cloud->points.size (),  (int) working_cloud->points.size (), (int) input_cloud->points.size () - (int) working_cloud->points.size ());
  }

  // Add the point cloud data
  viewer.addPointCloud (*working_cloud, "FILTERED");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, "FILTERED"); 
  // And wait until Q key is pressed
  viewer.spin ();

  // ---------------------------------------------------------------- //
  // ------------------ Projecting of Point Clouds ------------------ //
  // ---------------------------------------------------------------- //












  if ( project )
    for (int idx = 0; idx < (int) working_cloud->points.size (); idx++)
      working_cloud->points[idx].z = 0.0;  




  // Add the point cloud data
  viewer.addPointCloud (*working_cloud, "PROJECTED");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, "PROJECTED"); 
  // And wait until Q key is pressed
  viewer.spin ();






  // ------------------------------------------------------------------ //
  // ------------------ Estimating Normals of Points ------------------ //
  // ------------------------------------------------------------------ //

  // Point cloud of normals
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> ());
  // Build kd-tree structure for normals
  pcl::KdTreeFLANN<PointT>::Ptr normals_tree (new pcl::KdTreeFLANN<PointT> ());

  // Create object for normal estimation
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  // Provide pointer to the search method
  ne.setSearchMethod (normals_tree);
  // Set for which point cloud to compute the normals
  ne.setInputCloud (working_cloud);
  // Set number of k nearest neighbors to use
  ne.setKSearch (50);
  // Estimate the normals
  ne.compute (*normals_cloud);

  if ( verbose )
  {
    ROS_INFO ("Normal Estimation ! Returned: %d normals", (int) normals_cloud->points.size ());
  }










  int level = 1;
  double scale = 0.025;
  viewer.addPointCloudNormals (*working_cloud, *normals_cloud, level, scale, "NORMALS");
  // And wait until Q key is pressed 
  viewer.spin ();

  // Color the cloud with red
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "NORMALS"); 
  // And wait until Q key is pressed 
  viewer.spin ();









 //// Get position of dot in path of file 
  //std::string file = argv [pFileIndicesPCD[0]];
  //size_t dot = file.find (".");

  //// Create file name for saving
  //std::string circle_space_filename = argv [pFileIndicesPCD [0]];
  //circle_space_filename.insert (dot, "-circles");

  //// Save these points to disk
  //pcl::io::savePCDFile (circle_space_filename, *those_points);

  //if ( verbose )
  //{
    //// Show the floor's number of points
    //ROS_INFO ("The space which is represented by the circles models has %d points and was saved !", (int) those_points->points.size ());
  //}





















  if ( verbose )
  {
    // Displaying the overall time
    ROS_WARN ("Finished in %5.3g [s] !", tt.toc ());
  }

  // And wait until Q key is pressed
  viewer.spin ();

return (0);
}
