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



// Method's Parameters
double   line_threshold = 0.010; /// [meters]
double circle_threshold = 0.010; /// [meters]
double voting_threshold =  0.25; /// [percentage]
double minimum_radius = 0.010; /// [meters]
double maximum_radius = 0.100; /// [meters]
int minimum_line_inliers   = 10; /// [points]
int minimum_circle_inliers = 50; /// [points]
int maximum_line_iterations   = 100; /// [iterations]
int maximum_circle_iterations = 100; /// [iterations]
double   line_inliers_clustering_tolerance = 0.010; /// [meters]
double circle_inliers_clustering_tolerance = 0.010; /// [meters]

// Visualization's Parameters
int point_size = 3;
bool   line_step = false;
bool circle_step = false;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Computes line's coefficients with regards to its inliers
 * \param inliers_cloud The point cloud of the line's inliers
 * \param coefficients The line's parameters where the first triplete is a point on the line and the second triplete is the direction
 */
void adjustLine (pcl::PointCloud<pcl::PointXYZ>::Ptr &inliers_cloud, pcl::ModelCoefficients &coefficients)
{
  // Vector of line
  double line[2];
  line[0] = coefficients.values [3];
  line[1] = coefficients.values [4];

  // Normalize the vector of line
  double norm = sqrt (line[0]*line[0] + line[1]*line[1]);
  line[0] = line[0] / norm;
  line[1] = line[1] / norm;

  // First point of line
  double P1[2];
  P1[0] = coefficients.values [0];
  P1[1] = coefficients.values [1];

  // Second point of line
  double P2[2];
  P2[0] = coefficients.values [3] + coefficients.values [0];
  P2[1] = coefficients.values [4] + coefficients.values [1];

  // Get limits of inliers leghtwise
  double minimum_lengthwise =  DBL_MAX;
  double maximum_lengthwise = -DBL_MAX;
  for (int idx = 0; idx < (int)inliers_cloud->points.size (); idx++)
  {
    double P[2];
    P[0] = inliers_cloud->points.at(idx).x - P1[0];
    P[1] = inliers_cloud->points.at(idx).y - P1[1];

    double distance_lengthwise = (line[0]*P[0]) + (line[1]*P[1]);
    if (minimum_lengthwise > distance_lengthwise) minimum_lengthwise = distance_lengthwise;
    if (maximum_lengthwise < distance_lengthwise) maximum_lengthwise = distance_lengthwise;
  }

  // New model's coefficients
  P2[0] = P1[0] + line[0]*maximum_lengthwise;
  P2[1] = P1[1] + line[1]*maximum_lengthwise;

  P1[0] = P1[0] + line[0]*minimum_lengthwise;
  P1[1] = P1[1] + line[1]*minimum_lengthwise;

  // Save the new coefficients
  coefficients.values [0] = P1[0];
  coefficients.values [1] = P1[1];
  coefficients.values [2] = 0.0;

  coefficients.values [3] = P2[0] - P1[0];
  coefficients.values [4] = P2[1] - P1[1];
  coefficients.values [5] = 0.0;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of the method. Segmentation of point cloud data based on Hough-voting of RANSAC-fitted models
 */
int main (int argc, char** argv)
{

  // Initialize random number generator
  srand (time(0));

  // Declare the timer
  terminal_tools::TicToc tt;

  // Starting timer
  tt.tic ();

  ROS_WARN ("Timer started !");
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");



  // --------------------------------------------------------------- //
  // ------------------ Check and parse arguments ------------------ //
  // --------------------------------------------------------------- //

  // Argument check and info about
  if (argc < 2)
  {
    std::cout << std::endl;
    ROS_INFO ("Syntax is: %s <input>.pcd <options>", argv[0]);
    ROS_INFO ("where <options> are: -line_threshold X                       = threshold for line inlier selection");
    ROS_INFO ("                     -circle_threshold X                     = threshold for circle inlier selection");
    ROS_INFO ("                     -voting_threshold X                     = threshold for Hough-based model voting");
    ROS_INFO ("                     -minimum_radius X                       = ");
    ROS_INFO ("                     -maximum_radius X                       = ");
    ROS_INFO ("                     -minimum_line_inliers D                 = ");
    ROS_INFO ("                     -minimum_circle_inliers D               = ");
    ROS_INFO ("                     -maximum_line_iterations D              = ");
    ROS_INFO ("                     -maximum_circle_iterations D            = ");
    ROS_INFO ("                     -line_inliers_clustering_tolerance X    = ");
    ROS_INFO ("                     -circle_inliers_clustering_tolerance X  = ");
    ROS_INFO ("                                                               ");
    ROS_INFO ("                     -point_size B                           = ");
    ROS_INFO ("                     -line_step B                            = wait or not wait");
    ROS_INFO ("                     -circle_step B                          = wait or not wait");
    std::cout << std::endl;
    return (-1);
  }

  // Take only the first .pcd file into account
  std::vector<int> pFileIndicesPCD = terminal_tools::parse_file_extension_argument (argc, argv, ".pcd");
  if (pFileIndicesPCD.size () == 0)
  {
    ROS_INFO ("No .pcd file given as input!");
    return (-1);
  }

  // Parsing the arguments of the method
  terminal_tools::parse_argument (argc, argv,   "-line_threshold",   line_threshold);
  terminal_tools::parse_argument (argc, argv, "-circle_threshold", circle_threshold);
  terminal_tools::parse_argument (argc, argv, "-voting_threshold", voting_threshold);
  terminal_tools::parse_argument (argc, argv, "-minimum_radius", minimum_radius);
  terminal_tools::parse_argument (argc, argv, "-maximum_radius", maximum_radius);
  terminal_tools::parse_argument (argc, argv, "-minimum_line_inliers",   minimum_line_inliers);
  terminal_tools::parse_argument (argc, argv, "-minimum_circle_inliers", minimum_circle_inliers);
  terminal_tools::parse_argument (argc, argv, "-maximum_line_iterations",   maximum_line_iterations);
  terminal_tools::parse_argument (argc, argv, "-maximum_circle_iterations", maximum_circle_iterations);
  terminal_tools::parse_argument (argc, argv,   "-line_inliers_clustering_tolerance",   line_inliers_clustering_tolerance);
  terminal_tools::parse_argument (argc, argv, "-circle_inliers_clustering_tolerance", circle_inliers_clustering_tolerance);

  // Parsing the arguments for visualization
  terminal_tools::parse_argument (argc, argv,  "-point_size",  point_size);
  terminal_tools::parse_argument (argc, argv,   "-line_step",   line_step);
  terminal_tools::parse_argument (argc, argv, "-circle_step", circle_step);



  // ---------------------------------------------------------------- //
  // ------------------ Load the point cloud dataa ------------------ //
  // ---------------------------------------------------------------- //

  // Input point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // Load point cloud data
  if (pcl::io::loadPCDFile (argv[pFileIndicesPCD[0]], *input_cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv[pFileIndicesPCD[0]]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*input_cloud).c_str ());



  // ------------------------------------------------------------- //
  // ------------------ Filter point cloud data ------------------ //
  // ------------------------------------------------------------- //

  // Filtered point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
  // Auxiliary filtered point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr auxiliary_filtered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // Set which point cloud to filter
  sor.setInputCloud (input_cloud);
  // Set number of points for mean distance estimation
  sor.setMeanK (25);
  // Set the standard deviation multiplier threshold
  sor.setStddevMulThresh (1.0);
  // Call the filtering method
  sor.filter (*filtered_cloud);
  // Save filtered cloud to auxiliary
  *auxiliary_filtered_cloud = *filtered_cloud;

  ROS_INFO ("Statistical Outlier Removal ! before: %d points | after: %d points | filtered: %d points", input_cloud->points.size (),  filtered_cloud->points.size (), input_cloud->points.size () - filtered_cloud->points.size ());



  // ------------------------------------------------------------------- //
  // ------------------ Estiamte 3D normals of points ------------------ //
  // ------------------------------------------------------------------- //
 
  // Point cloud of normals
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> ());
  // Build kd-tree structure for normals
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr normals_tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

  // Create object for normal estimation
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  // Provide pointer to the search method
  ne.setSearchMethod (normals_tree);
  // Set for which point cloud to compute the normals
  ne.setInputCloud (filtered_cloud);
  // Set number of k nearest neighbors to use
  ne.setKSearch (50);
  // Estimate the normals
  ne.compute (*normals_cloud);



  // -------------------------------------------------------------- //
  // ------------------ Computation of 2D circles ----------------- //
  // -------------------------------------------------------------- //

  // Open a 3D viewer
  pcl_visualization::PCLVisualizer circle_viewer ("3D CIRCLE VIEWER");
  // Set the background of viewer
  circle_viewer.setBackgroundColor (0.0, 0.0, 0.0);
  // Add system coordiante to viewer
  circle_viewer.addCoordinateSystem (1.0f);
  // Parse the camera settings and update the internal camera
  circle_viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render
  circle_viewer.updateCamera ();

  // TODO add color to point clouds and to certain points from point clouds using pcl visualization

  // Add the point cloud data
  circle_viewer.addPointCloud (*input_cloud, "INPUT");
  // Color the cloud in white
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "INPUT");
  // Set the size of points for cloud
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "INPUT"); 
  // And wait until Q key is pressed
  circle_viewer.spin ();

  // Remove the point cloud data
  circle_viewer.removePointCloud ("INPUT");
  // And wait until Q key is pressed
  circle_viewer.spin ();

  // Add the filtered point cloud data in the same viewer
  circle_viewer.addPointCloud (*filtered_cloud, "FILTERED");
  // Color the filtered points in blue
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "FILTERED");
  // Set the size of points for cloud
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "FILTERED");
  // And wait until Q key is pressed
  circle_viewer.spin ();

  /*

  int level = 1;
  double scale = 0.025;
  // Add normals of points in the same viewer
  circle_viewer.addPointCloudNormals (*filtered_cloud, *normals_cloud, level, scale, "NORMALS");
  // Color the filtered points in blue
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "NORMALS");
  // And wait until Q key is pressed
  circle_viewer.spin ();

  */

  // ------------------------ //
  // Start fitting 2D circles //
  // ------------------------ //

  int circle_fit = 0;

  bool stop_circles = false;

  // Space of parameters for fitted circle models
  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_parameters_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  do
  {
    // Inliers of circle model
    pcl::PointIndices::Ptr circle_inliers (new pcl::PointIndices ());
    // Coefficients of cirlce model
    pcl::ModelCoefficients circle_coefficients;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> circle_segmentation;
    // Optimize coefficients
    circle_segmentation.setOptimizeCoefficients (false);
    // Set type of method
    circle_segmentation.setMethodType (pcl::SAC_RANSAC);
    // Set type of model
    circle_segmentation.setModelType (pcl::SACMODEL_CIRCLE2D);
    // Set number of maximum iterations
    circle_segmentation.setMaxIterations (maximum_circle_iterations);
    // Set threshold of model
    circle_segmentation.setDistanceThreshold (circle_threshold);
    // Set minimum and maximum radii
    circle_segmentation.setRadiusLimits (minimum_radius, maximum_radius);
    // Give as input the filtered point cloud
    circle_segmentation.setInputCloud (filtered_cloud);
    // Call the segmenting method
    circle_segmentation.segment (*circle_inliers, circle_coefficients);

    // ---------------------------- //
    // Start the extraction process //
    // ---------------------------- //

    // Point cloud of circle inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_inliers_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    // Extract the circular inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> circle_extraction;
    // Set point cloud from where to extract
    circle_extraction.setInputCloud (filtered_cloud);
    // Set which indices to extract
    circle_extraction.setIndices (circle_inliers);

    // Return the points which represent the inliers
    circle_extraction.setNegative (false);
    // Call the extraction function
    circle_extraction.filter (*circle_inliers_cloud);

    ///*

    // Return the remaining points of inliers
    circle_extraction.setNegative (true);
    // Call the extraction function
    circle_extraction.filter (*filtered_cloud);

    //*/

    ///*

    ROS_INFO ("Circle has %d inliers", circle_inliers_cloud->points.size());
    ROS_INFO ("%d points remain after extraction", filtered_cloud->points.size ());

    //*/

    // ---------------------------- //
    // Start the clustering process //
    // ---------------------------- //

    // Vector of clusters from inliers
    std::vector<pcl::PointIndices> circle_clusters;
    // Build kd-tree structure for clusters
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr circle_clusters_tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

    // Instantiate cluster extraction object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> circle_clustering;
    // Set as input the cloud of circle inliers
    circle_clustering.setInputCloud (circle_inliers_cloud);
    // Radius of the connnectivity threshold
    circle_clustering.setClusterTolerance (circle_inliers_clustering_tolerance);
    // Provide pointer to the search method
    circle_clustering.setSearchMethod (circle_clusters_tree);
    // Call the extraction function
    circle_clustering.extract (circle_clusters);

    int maximum_circle_clusters_size = -1;
    int maximum_circle_clusters_index = -1;
    for (int c = 0; c < (int) circle_clusters.size(); c++)
    {
      if (maximum_circle_clusters_size < (int) circle_clusters.at(c).indices.size())
      {
        maximum_circle_clusters_size = circle_clusters.at(c).indices.size();
        maximum_circle_clusters_index = c;
      }
    }

    ///*

    ROS_WARN (" has %d clusters where", circle_clusters.size());
    for (int c = 0; c < (int) circle_clusters.size(); c++)
      ROS_WARN ("       cluster %d has %d points", c, (int) circle_clusters.at(c).indices.size());
    ROS_WARN (" and biggest cluster is %d with %d points", maximum_circle_clusters_index, maximum_circle_clusters_size);

    //*/

    // -------------------------------------------------------------------------------------------------------------------------------

    // The biggest cluster of the circle inliers
    pcl::PointIndices::Ptr biggest_cluster_of_circle_inliers (new pcl::PointIndices ());
    *biggest_cluster_of_circle_inliers = circle_clusters.at(0);

    // Point cloud of the biggest cluster of the circle inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr biggest_cluster_of_circle_inliers_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    ROS_WARN ("   before:");
    ROS_WARN ("   biggest_cluster_of_circle_inliers_cloud has %d inliers", biggest_cluster_of_circle_inliers_cloud->points.size ());
    ROS_WARN ("   circle_inliers_cloud has %d inliers", circle_inliers_cloud->points.size ());
    ROS_INFO ("   %d points remain after extraction", filtered_cloud->points.size ());

    // Extract the circular inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> circle_clusters_extraction;
    // Set point cloud from where to extract
    circle_clusters_extraction.setInputCloud (circle_inliers_cloud);
    // Set which indices to extract
    circle_clusters_extraction.setIndices (biggest_cluster_of_circle_inliers);

    // Return the points which represent the inliers
    circle_clusters_extraction.setNegative (false);
    // Call the extraction function
    circle_clusters_extraction.filter (*biggest_cluster_of_circle_inliers_cloud);

    // Return the remaining points of inliers
    circle_clusters_extraction.setNegative (true);
    // Call the extraction function
    circle_clusters_extraction.filter (*circle_inliers_cloud);

    // Set point cloud from where to extract
    circle_extraction.setInputCloud (filtered_cloud);
    // Set which indices to extract
    circle_extraction.setIndices (biggest_cluster_of_circle_inliers);

    // Return the remaining points of inliers
    circle_extraction.setNegative (true);
    // Call the extraction function
    circle_extraction.filter (*filtered_cloud);

    ROS_WARN ("   after:");
    ROS_WARN ("   biggest_cluster_of_circle_inliers_cloud has %d inliers", biggest_cluster_of_circle_inliers_cloud->points.size ());
    ROS_WARN ("   circle_inliers_cloud has %d inliers left", circle_inliers_cloud->points.size ());
    ROS_INFO ("   %d points remain after extraction", filtered_cloud->points.size ());

    // -------------------------------------------------------------------------------------------------------------------------------

    // Check if the method found inliers for the current circle model
    if ((int) circle_inliers->indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a circular model for the given dataset.");
      ROS_ERROR (" %d points remain unfitted.", filtered_cloud->points.size ());

      // Points which remain unfitted
      pcl::PointCloud<pcl::PointXYZ>::Ptr circle_unfitted_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      // Save these points to disk
      pcl::io::savePCDFile ("data/circle-unfitted-cloud.pcd", *circle_unfitted_cloud);

      /*

      // Open a 3D viewer
      pcl_visualization::PCLVisualizer v ("CIRCLE_UNFITTED_CLOUD");
      // Set the background of viewer
      v.setBackgroundColor (0.0, 0.0, 0.0);
      // Add system coordiante to viewer
      v.addCoordinateSystem (1.0f);
      // Add the point cloud data
      v.addPointCloud (*filtered_cloud, "CIRCLE_UNFITTED_CLOUD");

      // Parse the camera settings and update the internal camera
      v.getCameraParameters (argc, argv);
      // Update camera parameters and render.
      v.updateCamera ();
      // And wait until Q key is pressed
      v.spin ();

      */

      return (-1);
    }

    // Check if the fitted circle has enough inliers in order to be accepted
    if ((int) circle_inliers->indices.size () < minimum_circle_inliers)
    {
      ROS_ERROR ("NOT ACCEPTED ! Circle [%2d] has %3d inliers with C = (%6.3f,%6.3f) and R = %5.3f in [%5.3f, %5.3f] found in maximum %d iterations",
                 circle_fit, circle_inliers->indices.size (), circle_coefficients.values [0], circle_coefficients.values [1], circle_coefficients.values [2], minimum_radius, maximum_radius, maximum_circle_iterations);

      // No need for fitting circles anymore
      stop_circles = true;
    }
    else
    {
      ROS_INFO ("ACCEPTED ! Circle [%2d] has %3d inliers with C = (%6.3f,%6.3f) and R = %5.3f in [%5.3f, %5.3f] found in maximum %d iterations",
                circle_fit, circle_inliers->indices.size (), circle_coefficients.values [0], circle_coefficients.values [1], circle_coefficients.values [2], minimum_radius, maximum_radius, maximum_circle_iterations);

      // Build the space of parameters for circles //

      // A vote consists of the actual circle parameters
      pcl::PointXYZ circle_vot;
      circle_vot.x = circle_coefficients.values [0];
      circle_vot.y = circle_coefficients.values [1];
      circle_vot.z = circle_coefficients.values [2];

      // Cast one vot for the current circle
      circle_parameters_cloud->points.push_back (circle_vot);
    }

    // --------------------------- //
    // Start visualization process //
    // --------------------------- //

    // Create ID for circle model
    std::stringstream circle_id;
    circle_id << "CIRCLE_" << circle_fit;

    // Create ID for circle inliers
    std::stringstream circle_inliers_id;
    circle_inliers_id << "CIRCLE_INLIERS_" << circle_fit ;

    // Add circle model to point cloud
    circle_viewer.addCircle (circle_coefficients, circle_id.str ());

    // Add the point cloud data
    circle_viewer.addPointCloud (*circle_inliers_cloud, circle_inliers_id.str ());

    // Set the size of points for cloud
    circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, circle_inliers_id.str ()); 

    // Wait or not wait
    if ( circle_step )
    {
      // And wait until Q key is pressed
      circle_viewer.spin ();
    }

    // number of fitted circles
    circle_fit++;

    // --------------------------------------------------- //
    // Check for continuing with the fitting of 2D circles //
    // --------------------------------------------------- //

    // Print the number of points left for model fitting
    if ( (int) filtered_cloud->points.size () < minimum_circle_inliers )
      ROS_ERROR (" %d < %d | Stop !", filtered_cloud->points.size (), minimum_circle_inliers);
    else
      if ( (int) filtered_cloud->points.size () > minimum_circle_inliers )
        ROS_WARN (" %d > %d | Continue... ", filtered_cloud->points.size (), minimum_circle_inliers);
      else
        ROS_WARN (" %d = %d | Continue... ", filtered_cloud->points.size (), minimum_circle_inliers);

  } while ((int) filtered_cloud->points.size () > minimum_circle_inliers && stop_circles == false);

  // And wait until Q key is pressed
  circle_viewer.spin ();

  // Save these points to disk
  pcl::io::savePCDFile ("data/circle-parameters-cloud.pcd", *circle_parameters_cloud);

  // Add the point cloud data
  circle_viewer.addPointCloud (*circle_parameters_cloud, "CIRCLE_PARAMETER");

  // Set the size of points for cloud
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size * 2, "CIRCLE_PARAMETER"); 

  // Save these points to disk
  pcl::io::savePCDFile ("data/circle-rest-cloud.pcd", *filtered_cloud);

  // Done with 2D circle models
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Done with 2D circle models in %5.3g [s]", tt.toc ());



  // ---------------------------------------------------------------------- //
  // ------------------ Recover filtered point cloud data ----------------- //
  // ---------------------------------------------------------------------- //

  filtered_cloud = auxiliary_filtered_cloud;



  // ------------------------------------------------------------ //
  // ------------------ Computation of 2D lines ----------------- //
  // ------------------------------------------------------------ //

  // Open a 3D viewer
  pcl_visualization::PCLVisualizer line_viewer ("3D LINE VIEWER");
  // Set the background of viewer
  line_viewer.setBackgroundColor (0.0, 0.0, 0.0);
  // Add system coordiante to viewer
  line_viewer.addCoordinateSystem (1.0f);
  // Parse the camera settings and update the internal camera
  line_viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render
  line_viewer.updateCamera ();

  // TODO add color to point clouds and to certain points from point clouds using pcl visualization

  // Add the point cloud data
  line_viewer.addPointCloud (*input_cloud, "INPUT");
  // Color the cloud in white
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "INPUT");
  // Set the size of points for cloud
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "INPUT"); 
  // And wait until Q key is pressed
  line_viewer.spin ();

  // Remove the point cloud data
  line_viewer.removePointCloud ("INPUT");
  // And wait until Q key is pressed
  line_viewer.spin ();

  // Add the filtered point cloud data in the same viewer
  line_viewer.addPointCloud (*filtered_cloud, "FILTERED");
  // Color the filtered points in blue
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "FILTERED");
  // Set the size of points for cloud
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "FILTERED");
  // And wait until Q key is pressed
  line_viewer.spin ();

  /*

  int level = 1;
  double scale = 0.025;
  // Add normals of points in the same viewer
  line_viewer.addPointCloudNormals (*filtered_cloud, *normals_cloud, level, scale, "NORMALS");
  // Color the filtered points in blue
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "NORMALS");
  // And wait until Q key is pressed
  line_viewer.spin ();

  */

  // ---------------------- //
  // Start fitting 2D lines //
  // ---------------------- //

  int line_fit = 0;

  bool stop_lines = false;

  // Space of parameters for fitted circle models
  pcl::PointCloud<pcl::PointXYZ>::Ptr line_parameters_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  do
  {
    // Inliers of line model
    pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());
    // Coefficients of cirlce model
    pcl::ModelCoefficients line_coefficients;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> line_segmentation;
    // Optimize coefficients
    line_segmentation.setOptimizeCoefficients (false);
    // Set type of method
    line_segmentation.setMethodType (pcl::SAC_RANSAC);
    // Set type of model
    line_segmentation.setModelType (pcl::SACMODEL_LINE);
    // Set number of maximum iterations
    line_segmentation.setMaxIterations (maximum_line_iterations);
    // Set threshold of model
    line_segmentation.setDistanceThreshold (line_threshold);
    // Give as input the filtered point cloud
    line_segmentation.setInputCloud (filtered_cloud);
    // Call the segmenting method
    line_segmentation.segment (*line_inliers, line_coefficients);

    // Check if the method found inliers for the current circle model
    if ((int) line_inliers->indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a liniar model for the given dataset.");
      ROS_ERROR (" %d points remain unfitted.", filtered_cloud->points.size ());

      // Points which remain unfitted
      pcl::PointCloud<pcl::PointXYZ>::Ptr line_unfitted_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      // Save these points to disk
      pcl::io::savePCDFile ("data/line-unfitted-cloud.pcd", *line_unfitted_cloud);

      /*

      // Open a 3D viewer
      pcl_visualization::PCLVisualizer v ("LINE_UNFITTED_CLOUD");
      // Set the background of viewer
      v.setBackgroundColor (0.0, 0.0, 0.0);
      // Add system coordiante to viewer
      v.addCoordinateSystem (1.0f);
      // Add the point cloud data
      v.addPointCloud (*filtered_cloud, "LINE_UNFITTED_CLOUD");

      // Parse the camera settings and update the internal camera
      v.getCameraParameters (argc, argv);
      // Update camera parameters and render.
      v.updateCamera ();
      // And wait until Q key is pressed
      v.spin ();

      */

      return (-1);
    }

    // Check if the fitted lines has enough inliers in order to be accepted
    if ((int) line_inliers->indices.size () < minimum_line_inliers)
    {
      ROS_ERROR ("NOT ACCEPTED ! Line [%2d] has %3d inliers with Point = (%6.3f,%6.3f,%6.3f) and Direction = (%6.3f,%6.3f,%6.3f) found in maximum %d iterations", line_fit, line_inliers->indices.size (),
                 line_coefficients.values [0], line_coefficients.values [1], line_coefficients.values [2], line_coefficients.values [3], line_coefficients.values [4], line_coefficients.values [5], maximum_line_iterations);

      // No need for fitting lines anymore
      stop_lines = true;
    }
    else
    {

      ROS_INFO ("ACCEPTED ! Line [%2d] has %3d inliers with Point = (%6.3f,%6.3f,%6.3f) and Direction = (%6.3f,%6.3f,%6.3f) found in maximum %d iterations", line_fit, line_inliers->indices.size (),
                line_coefficients.values [0], line_coefficients.values [1], line_coefficients.values [2], line_coefficients.values [3], line_coefficients.values [4], line_coefficients.values [5], maximum_line_iterations);

      // Build the space of parameters for lines //
 
      // Computing the line's parameters
      double m = line_coefficients.values [4] / line_coefficients.values [3];
      double b = line_coefficients.values [1] - m * line_coefficients.values [0];

      // A vote consists of the actual line parameters
      pcl::PointXYZ line_vot;
      line_vot.x = m;
      line_vot.y = b;

      // Cast one vot for the current line
      line_parameters_cloud->points.push_back (line_vot);
    }

    // ---------------------------- //
    // Start the extraction process //
    // ---------------------------- //

    // Point cloud of line inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr line_inliers_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    // Extract the circular inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> line_extraction;
    // Set point cloud from where to extract
    line_extraction.setInputCloud (filtered_cloud);
    // Set which indices to extract
    line_extraction.setIndices (line_inliers);

    // Return the points which represent the inliers
    line_extraction.setNegative (false);
    // Call the extraction function
    line_extraction.filter (*line_inliers_cloud);

    // Return the remaining points of inliers
    line_extraction.setNegative (true);
    // Call the extraction function
    line_extraction.filter (*filtered_cloud);

    /*

    ROS_INFO ("Line has %d inliers", line_inliers_cloud->points.size());
    ROS_INFO ("%d points remain after extraction", filtered_cloud->points.size ());

    */

    // --------------------------- //
    // Start visualization process //
    // --------------------------- //

    // Create ID for line model
    std::stringstream line_id;
    line_id << "LINE_" << line_fit;

    // Create ID for line inliers
    std::stringstream line_inliers_id;
    line_inliers_id << "LINE_INLIERS_" << line_fit ;

    // Compute line lengthwise with regard to its inliers
    adjustLine (line_inliers_cloud, line_coefficients);
    // Add line model to point cloud
    line_viewer.addLine (line_coefficients, line_id.str ());
    // Add the point cloud data
    line_viewer.addPointCloud (*line_inliers_cloud, line_inliers_id.str ());
    // Set the size of points for cloud
    line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, line_inliers_id.str ()); 

    // Wait or not wait
    if ( line_step )
    {
      // And wait until Q key is pressed
      line_viewer.spin ();
    }

    // TODO Integrate the clustering results into the validation of model 

    // ---------------------------- //
    // Start the clustering process //
    // ---------------------------- //

    // Vector of clusters from inliers
    std::vector<pcl::PointIndices> line_clusters;
    // Build kd-tree structure for clusters
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr line_clusters_tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

    // Instantiate cluster extraction object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> line_clustering;
    // Set as input the cloud of line inliers
    line_clustering.setInputCloud (line_inliers_cloud);
    // Radius of the connnectivity threshold
    line_clustering.setClusterTolerance (line_inliers_clustering_tolerance);
    // Provide pointer to the search method
    line_clustering.setSearchMethod (line_clusters_tree);
    // Call the extraction function
    line_clustering.extract (line_clusters);

    ///*

    ROS_WARN (" has %d clusters where", line_clusters.size() );
    for (int c = 0; c < (int) line_clusters.size(); c++)
      ROS_WARN ("       cluster %d has %d points", c, line_clusters.at(c).indices.size() );

    //*/

    // Wait or not wait
    if ( line_step )
    {
      // And wait until Q key is pressed
      line_viewer.spin ();
    }

    // number of fitted lines
    line_fit++;

    // ------------------------------------------------- //
    // Check for continuing with the fitting of 2D lines //
    // ------------------------------------------------- //

    // Print the number of points left for model fitting
    if ( (int) filtered_cloud->points.size () < minimum_line_inliers )
      ROS_ERROR (" %d < %d | Stop !", filtered_cloud->points.size (), minimum_line_inliers);
    else
      if ( (int) filtered_cloud->points.size () > minimum_line_inliers )
        ROS_WARN (" %d > %d | Continue... ", filtered_cloud->points.size (), minimum_line_inliers);
      else
        ROS_WARN (" %d = %d | Continue... ", filtered_cloud->points.size (), minimum_line_inliers);

  } while ((int) filtered_cloud->points.size () > minimum_line_inliers && stop_lines == false);

  // And wait until Q key is pressed
  line_viewer.spin ();

  // Save these points to disk
  pcl::io::savePCDFile ("data/line-parameters-cloud.pcd", *line_parameters_cloud);

  // Add the point cloud data
  line_viewer.addPointCloud (*line_parameters_cloud, "LINE_PARAMETER");

  // Set the size of points for cloud
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size * 2, "LINE_PARAMETER"); 

  // Save these points to disk
  pcl::io::savePCDFile ("data/line-rest-cloud.pcd", *filtered_cloud);

  // Done with 2D line models
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Done with 2D line models in %5.3g [s]", tt.toc ());



  // TODO Computing the best votes and segmenting the point cloud data



  // Displaying the overall time
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Finished in %5.3g [s]", tt.toc ());



  // And wait until Q key is pressed
  circle_viewer.spin ();

  // And wait until Q key is pressed
  line_viewer.spin ();

  return (0);
}

