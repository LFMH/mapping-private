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

// ------------------------------------------------------------------------- //
// -------------------- Specify all needed dependencies -------------------- //
// ------------------------------------------------------------------------- //

#include "ros/ros.h"

#include "terminal_tools/parse.h"

#include "pcl/io/pcd_io.h"
#include "pcl/features/rsd.h"
#include "pcl/features/normal_3d.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/sac_model_circle.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

#include "pcl_visualization/pcl_visualizer.h"

#include "pcl_cloud_algos/pcl_cloud_algos_point_types.h"

// --------------------------------------------------------------- //
// -------------------- Declare defs of types -------------------- //
// --------------------------------------------------------------- //

typedef pcl::PointXYZINormal PointT;

//typedef pcl::PointNormalRADII PointT;

// --------------------------------------------------------------------- //
// -------------------- Declare method's parameters -------------------- //
// --------------------------------------------------------------------- //

// // // 
int iterations = 100; 

// Clustering's Parameters
int minimum_size_of_objects_clusters = 100; /* [points] */
double clustering_tolerance_of_objects = 0.025; /* [meters] */

// Fitting's Parameters
double   line_threshold = 0.010; /// [meters]
double circle_threshold = 0.010; /// [meters]
double voting_threshold =  0.25; /// [percentage]
double minimum_radius = 0.010; /// [meters]
double maximum_radius = 0.100; /// [meters]
int minimum_line_inliers   = 10; /// [points]
int minimum_circle_inliers = 50; /// [points]
int maximum_line_iterations   = 1000; /// [iterations]
int maximum_circle_iterations = 1000; /// [iterations]

double   line_inliers_clustering_tolerance = 0.010; /// [meters]
double circle_clustering_tolerance = 0.010; /// [meters]

int minimum_size_of_line_cluster = 10; /// [points]
int minimum_size_of_circle_cluster = 10; /// [points]



int normals_search_knn = 0; /// [points]
double normals_search_radius = 0.000; /// [meters]
double curvature_threshold = 0.010; /// 
double rsd_search_radius = 0.020; /// [meters]
double rsd_plane_radius = 0.200; /// [meters]
double low_r_min = 0.020; /// [meters]
double high_r_min = 0.060; /// [meters]
double angle_threshold = 45.0; /// [degrees]
double radius_threshold = 0.025; /// [meters]



bool clustering_feature = true;
bool curvature_feature = true;
bool rsd_feature = true;
bool normals_feature = true;






//
double circle_space_radius = 0.010;
double circle_space_percentage = 0.75;

// Visualization's Parameters
bool step = false;
bool verbose = false;
int size = 3;
bool   line_step = false;
bool circle = false;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Computes line's coefficients with regards to its inliers
 * \param inliers_cloud The point cloud of the line's inliers
 * \param coefficients The line's parameters where the first triplete is a point on the line and the second triplete is the direction
 */
void adjustLine (pcl::PointCloud<PointT>::Ptr &inliers_cloud, pcl::ModelCoefficients &coefficients)
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



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of the method. Segmentation of point cloud data based on Hough-voting of RANSAC-fitted models.
 */
int main (int argc, char** argv)
{
 
  // ------------------------------------------------------------------- //
  // -------------------- Check and parse arguments -------------------- //
  // ------------------------------------------------------------------- //

  // Argument check and info about
  if (argc < 2)
  {
    ROS_INFO (" ");
    ROS_INFO ("Syntax is: %s <input>.pcd <options>", argv[0]);
    ROS_INFO ("  where <options> are:");
    ROS_INFO ("    -iterations X                              = How many times to run the fitting routine.");
    ROS_INFO (" ");
    ROS_INFO ("    -line_threshold X                          = threshold for line inlier selection");
    ROS_INFO ("    -circle_threshold X                        = threshold for circle inlier selection");
    ROS_INFO ("    -voting_threshold X                        = threshold for Hough-based model voting");
    ROS_INFO ("    -minimum_radius X                          = ");
    ROS_INFO ("    -maximum_radius X                          = ");
    ROS_INFO ("    -minimum_line_inliers D                    = ");
    ROS_INFO ("    -minimum_circle_inliers D                  = ");
    ROS_INFO ("    -maximum_line_iterations D                 = ");
    ROS_INFO ("    -maximum_circle_iterations D               = ");
    ROS_INFO ("    -line_inliers_clustering_tolerance X       = ");
    ROS_INFO ("    -circle_clustering_tolerance X     = ");
    ROS_INFO (" ");
    ROS_INFO ("    -minimum_size_of_objects_clusters X        = ");
    ROS_INFO ("    -clustering_tolerance_of_objects X         = ");
    ROS_INFO (" ");



    ROS_INFO ("    -normals_search_knn X                      = ");
    ROS_INFO ("    -normals_search_radius X                   = ");
    ROS_INFO ("    -curvature_threshold X                     = ");
    ROS_INFO ("    -rsd_search_radius X                       = ");
    ROS_INFO ("    -rsd_plane_radius X                        = ");
    ROS_INFO ("    -low_r_min X                               = ");
    ROS_INFO ("    -high_r_min X                              = ");
    ROS_INFO ("    -angle_threshold X                         = ");
    ROS_INFO ("    -radius_threshold X                        = ");



    ROS_INFO ("    -clustering_feature X                      = ");
    ROS_INFO ("    -curvature_feature X                       = ");
    ROS_INFO ("    -rsd_feature X                             = ");
    ROS_INFO ("    -normals_feature X                         = ");



    ROS_INFO ("    -circle_space_radius X                     = ");
    ROS_INFO ("    -circle_space_percentage X                 = ");
    ROS_INFO (" ");
    ROS_INFO ("    -step B                                    = ");
    ROS_INFO ("    -verbose B                                 = ");
    ROS_INFO ("    -line_step B                               = wait or not wait");
    ROS_INFO ("    -circle B                             = wait or not wait");
    ROS_INFO ("    -size B                          = ");
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

  // // // 
  terminal_tools::parse_argument (argc, argv,   "-iterations", iterations);

  // Parsing parameters for clustering
  terminal_tools::parse_argument (argc, argv, "-clustering_tolerance_of_objects", clustering_tolerance_of_objects);
  terminal_tools::parse_argument (argc, argv, "-minimum_size_of_objects_clusters", minimum_size_of_objects_clusters);

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
  terminal_tools::parse_argument (argc, argv, "-circle_clustering_tolerance", circle_clustering_tolerance);
  terminal_tools::parse_argument (argc, argv, "-minimum_size_of_line_cluster", minimum_size_of_line_cluster);
  terminal_tools::parse_argument (argc, argv, "-minimum_size_of_circle_cluster", minimum_size_of_circle_cluster);



  terminal_tools::parse_argument (argc, argv, "-normals_search_knn", normals_search_knn);
  terminal_tools::parse_argument (argc, argv, "-normals_search_radius", normals_search_radius);
  terminal_tools::parse_argument (argc, argv, "-curvature_threshold", curvature_threshold);
  terminal_tools::parse_argument (argc, argv, "-rsd_search_radius", rsd_search_radius);  
  terminal_tools::parse_argument (argc, argv, "-rsd_plane_radius", rsd_plane_radius);  
  terminal_tools::parse_argument (argc, argv, "-low_r_min", low_r_min);  
  terminal_tools::parse_argument (argc, argv, "-high_r_min", high_r_min);  
  terminal_tools::parse_argument (argc, argv, "-angle_threshold", angle_threshold);
  terminal_tools::parse_argument (argc, argv, "-radius_threshold", radius_threshold);



  terminal_tools::parse_argument (argc, argv, "-clustering_feature", clustering_feature);  
  terminal_tools::parse_argument (argc, argv, "-curvature_feature", curvature_feature);  
  terminal_tools::parse_argument (argc, argv, "-rsd_feature", rsd_feature);  
  terminal_tools::parse_argument (argc, argv, "-normals_feature", normals_feature);



  //
  terminal_tools::parse_argument (argc, argv, "-circle_space_radius", circle_space_radius);
  terminal_tools::parse_argument (argc, argv, "-circle_space_percentage", circle_space_percentage);

  // Parsing the arguments for visualization
  terminal_tools::parse_argument (argc, argv, "-step", step);
  terminal_tools::parse_argument (argc, argv, "-verbose", verbose);
  terminal_tools::parse_argument (argc, argv, "-line_step", line_step);
  terminal_tools::parse_argument (argc, argv, "-circle", circle);
  terminal_tools::parse_argument (argc, argv, "-size", size);

  // --------------------------------------------------------- //
  // -------------------- Initializations -------------------- //
  // --------------------------------------------------------- //

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

  // --------------------------------------------------- //
  // -------------------- 3D Viewer -------------------- //
  // --------------------------------------------------- //

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

  // ----------------------------------------------------------- //
  // ------------------ Load point cloud data ------------------ //
  // ----------------------------------------------------------- //

  // Input point cloud data
  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT> ());

  // Working point cloud data 
  pcl::PointCloud<PointT>::Ptr working_cloud (new pcl::PointCloud<PointT> ());

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

  // Add the point cloud data
  viewer.addPointCloud (*input_cloud, "INPUT");
  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "INPUT");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "INPUT"); 
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Update working point cloud
  *working_cloud = *input_cloud;

  ///*
  // ------------------------------------------------------------- //
  // ------------------ Filter point cloud data ------------------ //
  // ------------------------------------------------------------- //

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
  sor.filter (*filtered_cloud);

  if ( verbose )
  {
    ROS_INFO ("Statistical Outlier Removal ! Before: %d points | After: %d points | Filtered: %d points",
              (int) working_cloud->points.size (),  (int) filtered_cloud->points.size (), (int) working_cloud->points.size () - (int) filtered_cloud->points.size ());
  }

  // Add the filtered point cloud data in the same viewer
  viewer.addPointCloud (*filtered_cloud, "FILTERED");
  // Color the filtered points in blue
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "FILTERED");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "FILTERED");
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Update working point cloud
  *working_cloud = *filtered_cloud;
  //*/








  // ------------------------------------------------------------------- //
  // ------------------ Estimate 3D normals of points ------------------ //
  // ------------------------------------------------------------------- //

  // Point cloud of normals
  pcl::PointCloud<PointT>::Ptr normals_cloud (new pcl::PointCloud<PointT> ());
  // Build kd-tree structure for normals
  pcl::KdTreeFLANN<PointT>::Ptr normals_tree (new pcl::KdTreeFLANN<PointT> ());

  // Create object for normal estimation
  pcl::NormalEstimation<PointT, PointT> ne;
  // Set for which point cloud to compute the normals
  ne.setInputCloud (working_cloud);
  // Provide pointer to the search method
  ne.setSearchMethod (normals_tree);

  if ( normals_search_knn )
  {
    // Set number of k nearest neighbors to use
    ne.setKSearch (normals_search_knn);
  }

  if ( normals_search_radius )
  {
    // Sphere radius used as the maximum distance to consider a point as neighbor
    ne.setRadiusSearch (normals_search_radius);
  }

  // Estimate the normals
  ne.compute (*normals_cloud);

  if ( verbose )
  {
    ROS_INFO ("Normal Estimation ! Returned: %d normals", (int) normals_cloud->points.size ());
  }

  // Add the point cloud of normals
  viewer.addPointCloudNormals (*working_cloud, *normals_cloud, 1, 0.025, "3D NORMALS");
  // Color the normals with red
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "3D NORMALS"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove the point cloud data
  viewer.removePointCloud ("3D NORMALS");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Save the fresh computed normal and curvature values to the working cloud
  for (int idx = 0; idx < (int) working_cloud->points.size (); idx++)
  {
    working_cloud->points.at (idx).normal_x = normals_cloud->points.at (idx).normal_x;
    working_cloud->points.at (idx).normal_y = normals_cloud->points.at (idx).normal_y;
    working_cloud->points.at (idx).normal_z = normals_cloud->points.at (idx).normal_z;
    working_cloud->points.at (idx).curvature = normals_cloud->points.at (idx).curvature;
  }

  // Set the color handler for curvature
  pcl_visualization::PointCloudColorHandlerGenericField<PointT> curvature_handler (*working_cloud, "curvature");
  // Add the point cloud of curvature values
  viewer.addPointCloud (*working_cloud, curvature_handler, "CURVATURE");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "CURVATURE"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove curvature point cloud
  viewer.removePointCloud ("CURVATURE");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Split the planar from the circular curvature points 
  pcl::PointIndices::Ptr curvature_planar_indices (new pcl::PointIndices ());
  pcl::PointIndices::Ptr curvature_circular_indices (new pcl::PointIndices ());

  for (int idx = 0; idx < (int) working_cloud->points.size(); idx++)
  {
    double curvature = working_cloud->points.at (idx).curvature;

    if ( curvature < curvature_threshold )
    {
      //cerr << " c: " << curvature << " < ct " << curvature_threshold << endl;
      //viewer.spin ();

      curvature_planar_indices->indices.push_back (idx);
    }
    else
    {
      //cerr << " c: " << curvature << " > ct " << curvature_threshold << endl;
      //viewer.spin ();

      curvature_circular_indices->indices.push_back (idx);
    }
  }

  // Extract the planar and circular curvature points from the working cloud
  pcl::PointCloud<PointT>::Ptr curvature_planar_cloud (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr curvature_circular_cloud (new pcl::PointCloud<PointT> ());

  pcl::ExtractIndices<PointT> curvature_extraction;
  curvature_extraction.setInputCloud (working_cloud);

  curvature_extraction.setIndices (curvature_planar_indices);
  curvature_extraction.setNegative (false);
  curvature_extraction.filter (*curvature_planar_cloud);

  curvature_extraction.setIndices (curvature_circular_indices);
  curvature_extraction.setNegative (false);
  curvature_extraction.filter (*curvature_circular_cloud);

/*
  std::stringstream curvature_planar_id;
  curvature_planar_id << "CURVATURE_PLANAR_" << ros::Time::now();
  viewer.addPointCloud (*curvature_planar_cloud, curvature_planar_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_planar_id.str ()); 
  if ( step ) viewer.spin ();
  viewer.removePointCloud (curvature_planar_id.str());
  if ( step ) viewer.spin ();

  std::stringstream curvature_circular_id;
  curvature_circular_id << "CURVATURE_CIRCULAR_" << ros::Time::now();
  viewer.addPointCloud (*curvature_circular_cloud, curvature_circular_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_circular_id.str ()); 
  if ( step ) viewer.spin ();
  viewer.removePointCloud (curvature_circular_id.str());
  if ( step ) viewer.spin ();
*/

  std::stringstream curvature_planar_id;
  curvature_planar_id << "CURVATURE_PLANAR_" << ros::Time::now();
  viewer.addPointCloud (*curvature_planar_cloud, curvature_planar_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_planar_id.str ()); 

  std::stringstream curvature_circular_id;
  curvature_circular_id << "CURVATURE_CIRCULAR_" << ros::Time::now();
  viewer.addPointCloud (*curvature_circular_cloud, curvature_circular_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_circular_id.str ()); 

  if ( step ) viewer.spin ();

  viewer.removePointCloud (curvature_planar_id.str());
  viewer.removePointCloud (curvature_circular_id.str());

  if ( step ) viewer.spin ();






  // ----------------------------------------------------------------------- //
  // -------------------- Estimate RSD values of points -------------------- //
  // ----------------------------------------------------------------------- //

  // Create the object for RSD estimation
  pcl::RSDEstimation<PointT, PointT, pcl::PrincipalRadiiRSD> rsd;
  pcl::KdTreeFLANN<PointT>::Ptr rsd_tree (new pcl::KdTreeFLANN<PointT> (working_cloud));

  pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsd_cloud (new pcl::PointCloud<pcl::PrincipalRadiiRSD> ());
  //pcl::PointCloud<pcl::PrincipalRadiiRSD> rsd_cloud;

  rsd.setInputCloud (working_cloud);
  rsd.setInputNormals (normals_cloud);
  rsd.setRadiusSearch (rsd_search_radius);
  rsd.setPlaneRadius (rsd_plane_radius);
  rsd.setSearchMethod (rsd_tree);
  rsd.compute (*rsd_cloud);

  // Get position of full stop in path of file
  std::string path =  argv [pFileIndicesPCD [0]];
  size_t fullstop = path.find (".");
  // Create name for saving pcd files
  std::string name = argv [1];
  name.insert (fullstop, "-rsd");

  // Declare the rsd working cloud of points
  pcl::PointCloud<pcl::PointXYZINormalRSD>::Ptr rsd_working_cloud (new pcl::PointCloud<pcl::PointXYZINormalRSD> ());
  // Concatenate radii values with the working cloud
  pcl::concatenateFields (*working_cloud, *rsd_cloud, *rsd_working_cloud);
  // Save these points to disk
  pcl::io::savePCDFile (name, *rsd_working_cloud);

  // Set the color handler for curvature
  pcl_visualization::PointCloudColorHandlerGenericField<pcl::PointXYZINormalRSD> rsd_handler (*rsd_working_cloud, "r_min");
  // Add the point cloud of curvature values
  viewer.addPointCloud (*rsd_working_cloud, rsd_handler, "R_MIN");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "R_MIN"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove r min point cloud
  viewer.removePointCloud ("R_MIN");

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Split the plausible from the implausible r min values
  pcl::PointIndices::Ptr r_min_plausible_indices (new pcl::PointIndices ());
  pcl::PointIndices::Ptr r_min_implausible_indices (new pcl::PointIndices ());

  for (int idx = 0; idx < (int) rsd_working_cloud->points.size(); idx++)
  {
    double r_min = rsd_working_cloud->points.at (idx).r_min;

    if ( (low_r_min < r_min) && (r_min < high_r_min) )
    {
      //cerr << " low < " << r_min << " < high " << endl;
      //viewer.spin ();

      r_min_plausible_indices->indices.push_back (idx);
    }
    else
    {
      //cerr << r_min << endl;
      //viewer.spin ();

      r_min_implausible_indices->indices.push_back (idx);
    }
  }

  // Extract the plausible and implausible r min values from the working cloud
  pcl::PointCloud<PointT>::Ptr r_min_plausible_cloud (new pcl::PointCloud<PointT> ());
  pcl::PointCloud<PointT>::Ptr r_min_implausible_cloud (new pcl::PointCloud<PointT> ());

  pcl::ExtractIndices<PointT> r_min_extraction;
  r_min_extraction.setInputCloud (working_cloud);

  r_min_extraction.setIndices (r_min_plausible_indices);
  r_min_extraction.setNegative (false);
  r_min_extraction.filter (*r_min_plausible_cloud);

  r_min_extraction.setIndices (r_min_implausible_indices);
  r_min_extraction.setNegative (false);
  r_min_extraction.filter (*r_min_implausible_cloud);

/*
  std::stringstream r_min_plausible_id;
  r_min_plausible_id << "CURVATURE_PLANAR_" << ros::Time::now();
  viewer.addPointCloud (*r_min_plausible_cloud, r_min_plausible_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_plausible_id.str ()); 
  if ( step ) viewer.spin ();
  viewer.removePointCloud (r_min_plausible_id.str());
  if ( step ) viewer.spin ();

  std::stringstream r_min_implausible_id;
  r_min_implausible_id << "CURVATURE_CIRCULAR_" << ros::Time::now();
  viewer.addPointCloud (*r_min_implausible_cloud, r_min_implausible_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_implausible_id.str ()); 
  if ( step ) viewer.spin ();
  viewer.removePointCloud (r_min_implausible_id.str());
  if ( step ) viewer.spin ();
*/

  std::stringstream r_min_plausible_id;
  r_min_plausible_id << "R_MIN_PLAUSIBLE_" << ros::Time::now();
  viewer.addPointCloud (*r_min_plausible_cloud, r_min_plausible_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_plausible_id.str ()); 

  std::stringstream r_min_implausible_id;
  r_min_implausible_id << "R_MIN_IMPLAUSIBLE_" << ros::Time::now();
  viewer.addPointCloud (*r_min_implausible_cloud, r_min_implausible_id.str ());
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_implausible_id.str ()); 

  if ( step ) viewer.spin ();

  viewer.removePointCloud (r_min_plausible_id.str());
  viewer.removePointCloud (r_min_implausible_id.str());

  if ( step ) viewer.spin ();






  // ----------------------------------------------------------------------- //
  // -------------------- Estimate 2D normals of points -------------------- //
  // ----------------------------------------------------------------------- //

  for (int idx = 0; idx < (int) normals_cloud->points.size (); idx++)
    normals_cloud->points[idx].normal_z = 0.0;  

  // Add the normals
  viewer.addPointCloudNormals (*working_cloud, *normals_cloud, 1, 0.025, "2D NORMALS");
  // Color the normals with red
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "2D NORMALS"); 
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove the point cloud data
  viewer.removePointCloud ("2D NORMALS");
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  for (int idx = 0; idx < (int) normals_cloud->points.size (); idx++)
  {
    double nx = normals_cloud->points[idx].normal_x;
    double ny = normals_cloud->points[idx].normal_y;
    double nz = normals_cloud->points[idx].normal_z;

    double nl = sqrt (nx*nx + ny*ny + nz*nz);
    nx = nx / nl;
    ny = ny / nl;
    nz = nz / nl;

    normals_cloud->points[idx].normal_x = nx;
    normals_cloud->points[idx].normal_y = ny;
    normals_cloud->points[idx].normal_z = nz;
  }

  // Add the normals
  viewer.addPointCloudNormals (*working_cloud, *normals_cloud, 1, 0.025, "NORMALS");
  // Color the normals with red
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "NORMALS"); 
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove the point cloud data
  viewer.removePointCloud ("NORMALS");
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }






  //exit (0);






  // -------------------------------------------------------------- //
  // ------------------ Cluster point cloud data ------------------ //
  // -------------------------------------------------------------- //

  // Vector of clusters from input cloud
  std::vector<pcl::PointIndices> objects_clusters;
  // Build kd-tree structure for clusters
  pcl::KdTreeFLANN<PointT>::Ptr objects_clusters_tree (new pcl::KdTreeFLANN<PointT> ());

  // Instantiate cluster extraction object
  pcl::EuclideanClusterExtraction<PointT> ece;
  // Set as input the cloud of handle
  ece.setInputCloud (working_cloud);
  // Radius of the connnectivity threshold
  ece.setClusterTolerance (clustering_tolerance_of_objects);
  // Minimum size of clusters
  ece.setMinClusterSize (minimum_size_of_objects_clusters);
  // Provide pointer to the search method
  ece.setSearchMethod (objects_clusters_tree);
  // Call the extraction function
  ece.extract (objects_clusters);

  if ( verbose )
  {
    ROS_INFO ("Euclidean Cluster Extraction ! Returned: %d clusters", (int) objects_clusters.size ());
  }

  // ------------------------------------------------------------- //
  // ------------------ Extract object clusters ------------------ //
  // ------------------------------------------------------------- //

  // Vector of indices which make up the objects clusters
  std::vector<pcl::PointIndices::Ptr> objects_clusters_indices;

  // Point clouds which represent the clusters of the objects
  std::vector<pcl::PointCloud<PointT>::Ptr> objects_clusters_clouds;

  for (int clu = 0; clu < (int) objects_clusters.size(); clu++)
  {
    // Pointer to the cluster object
    pcl::PointIndices::Ptr object_cluster_indices (new pcl::PointIndices (objects_clusters.at(clu)));

    // Cloud of the cluster obejct
    pcl::PointCloud<PointT>::Ptr object_cluster_cloud (new pcl::PointCloud<PointT> ());


    // Extract handle points from the input cloud
    pcl::ExtractIndices<PointT> ei;
    // Set point cloud from where to extract
    ei.setInputCloud (working_cloud);
    // Set which indices to extract
    ei.setIndices (object_cluster_indices);
    // Return the points which represent the inliers
    ei.setNegative (false);
    // Call the extraction function
    ei.filter (*object_cluster_cloud);

    if ( verbose )
    {
      ROS_INFO ("  Object cluster %2d has %4d points", clu, (int) object_cluster_cloud->points.size());
    }

    // Save indices of object
    objects_clusters_indices.push_back (object_cluster_indices);

    // Save the cloud of object cluster
    objects_clusters_clouds.push_back (object_cluster_cloud);
  }

  // --------------------------------------------------------------- //
  // ------------------ Visualize object clusters ------------------ //
  // --------------------------------------------------------------- //

  // Vector of ids of handles
  std::vector<std::string> objects_clusters_ids;

  for (int clu = 0; clu < (int) objects_clusters_clouds.size(); clu++)
  {
    // Create id for visualization
    std::stringstream object_cluster_id;
    object_cluster_id << "OBJECT_CLUSTER_" << ros::Time::now();
    // Add point cloud to viewer
    viewer.addPointCloud (*objects_clusters_clouds.at(clu), object_cluster_id.str());
    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, object_cluster_id.str()); 
    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    // Save id of object
    objects_clusters_ids.push_back (object_cluster_id.str());
  }










  // -------------------------------------------------------------------------------------------------- //
  // -------------------------------------------------------------------------------------------------- //
  // ------------------------------------ Computation of 2D circles ----------------------------------- //
  // -------------------------------------------------------------------------------------------------- //
  // -------------------------------------------------------------------------------------------------- //

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

  // Add the point cloud data
  circle_viewer.addPointCloud (*working_cloud, "WORKING");
  // Color the cloud in white
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "WORKING");
  // Set the size of points for cloud
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "WORKING"); 
  // And wait until Q key is pressed
  circle_viewer.spin ();




























  // ------------------------ //
  // Start fitting 2D circles //
  // ------------------------ //

  // Space of parameters for fitted circle models
  pcl::PointCloud<PointT>::Ptr circle_parameters_cloud (new pcl::PointCloud<PointT> ());

  for (int ite = 0; ite < iterations; ite++)
  {
    // Print current iteration number
    ROS_ERROR ("AT ITERATION = %d", ite);

    // Vector of circle ids
    std::vector<std::string> circles_ids;
    // Vector of circle inliers ids
    std::vector<std::string> circles_inliers_ids;

    for (int clu = 0; clu < (int) objects_clusters_clouds.size(); clu++)
    {

      // Working vector of indices which represents a cluster
      pcl::PointIndices working_cluster_indices;
      // Update the working cluster cloud 
      working_cluster_indices = *objects_clusters_indices.at (clu);

      // Working cluster cloud which represents an object
      pcl::PointCloud<PointT>::Ptr working_cluster_cloud (new pcl::PointCloud<PointT> ());
      // Update the working cluster cloud 
      *working_cluster_cloud = *objects_clusters_clouds.at (clu);

      int circle_fit = 0;
      bool valid_circle = true;
      bool stop_circle_fitting = false;

      do
      {
        // Coefficients of cirlce model
        pcl::ModelCoefficients circle_coefficients;
        // Inliers of circle model
        pcl::PointIndices::Ptr circle_inliers (new pcl::PointIndices ());

        // --------------------- //
        // Start fitting process //
        // --------------------- //

        // Create the segmentation object
        pcl::SACSegmentation<PointT> segmentation_of_circle;
        // Optimize coefficients
        segmentation_of_circle.setOptimizeCoefficients (false);
        // Set type of method
        segmentation_of_circle.setMethodType (pcl::SAC_RANSAC);
        // Set type of model
        segmentation_of_circle.setModelType (pcl::SACMODEL_CIRCLE2D);
        // Set threshold of model
        segmentation_of_circle.setDistanceThreshold (circle_threshold);
        // Set number of maximum iterations
        segmentation_of_circle.setMaxIterations (maximum_circle_iterations);
        // Give as input the filtered point cloud
        segmentation_of_circle.setInputCloud (working_cluster_cloud);
        // Set minimum and maximum radii
        segmentation_of_circle.setRadiusLimits (minimum_radius, maximum_radius);

        // Call the segmenting method
        segmentation_of_circle.segment (*circle_inliers, circle_coefficients);

        // ------------------------ //
        // Start extraction process //
        // ------------------------ //

        // Point cloud of circle inliers
        pcl::PointCloud<PointT>::Ptr circle_inliers_cloud (new pcl::PointCloud<PointT> ());

        // Extract the circular inliers 
        pcl::ExtractIndices<PointT> extraction_of_circle;
        // Set which indices to extract
        extraction_of_circle.setIndices (circle_inliers);
        // Set point cloud from where to extract
        extraction_of_circle.setInputCloud (working_cluster_cloud);

        // Return the points which represent the inliers
        extraction_of_circle.setNegative (false);
        // Call the extraction function
        extraction_of_circle.filter (*circle_inliers_cloud);

        // ------------------------ //
        // Start backing up process //
        // ------------------------ //

        // Declare the backup for the circle inliers 
        pcl::PointIndices::Ptr original_circle_inliers (new pcl::PointIndices ());
        // Declare the backup for the circle inliers cloud
        pcl::PointCloud<PointT>::Ptr original_circle_inliers_cloud (new pcl::PointCloud<PointT> ());

        // Back up the circle inliers
        *original_circle_inliers = *circle_inliers;
        // Back up the circle inliers cloud
        *original_circle_inliers_cloud = *circle_inliers_cloud;



        // START W/ THE CLUSTERING FEATURE //

        if ( clustering_feature )
        {
          if ( (int) circle_inliers->indices.size() < minimum_circle_inliers )
          {
            // The current circle model will be rejected
            valid_circle = false;
          }
          else
          {
            std::stringstream before_clustering_id;
            before_clustering_id << "CIRCLE_INLIERS_" << ros::Time::now();
            circle_viewer.addPointCloud (*circle_inliers_cloud, before_clustering_id.str ());
            circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_clustering_id.str ()); 
            circle_viewer.spin ();
            circle_viewer.removePointCloud (before_clustering_id.str());
            circle_viewer.spin ();

            // Vector of clusters from inliers
            std::vector<pcl::PointIndices> circle_clusters;
            // Build kd-tree structure for clusters
            pcl::KdTreeFLANN<PointT>::Ptr circle_clusters_tree (new pcl::KdTreeFLANN<PointT> ());

            // Instantiate cluster extraction object
            pcl::EuclideanClusterExtraction<PointT> clustering_of_circle;
            // Set as input the cloud of circle inliers
            clustering_of_circle.setInputCloud (circle_inliers_cloud);
            // Radius of the connnectivity threshold
            clustering_of_circle.setClusterTolerance (circle_clustering_tolerance);
            // Minimum number of points of any cluster
            clustering_of_circle.setMinClusterSize (minimum_size_of_circle_cluster);
            // Provide pointer to the search method
            clustering_of_circle.setSearchMethod (circle_clusters_tree);

            // Call the extraction function
            clustering_of_circle.extract (circle_clusters);

            ROS_INFO ("  Model has %d inliers clusters where", (int) circle_clusters.size());
            for (int c = 0; c < (int) (int) circle_clusters.size(); c++)
              ROS_INFO ("    Cluster %d has %d points", c, (int) circle_clusters.at(c).indices.size());

            pcl::PointIndices::Ptr clustering_circle_inliers (new pcl::PointIndices ());

            // Leave only the two biggest clusters
            if ( circle_clusters.size() > 0 )
            {
              for ( int idx = 0; idx < (int) circle_clusters.at (0).indices.size(); idx++ )
              {
                int inl = circle_clusters.at (0).indices.at (idx);
                clustering_circle_inliers->indices.push_back (circle_inliers->indices.at (inl));
              }

              if ( circle_clusters.size() > 1 )
              {
                for ( int idx = 0; idx < (int) circle_clusters.at (1).indices.size(); idx++ )
                {
                  int inl = circle_clusters.at (1).indices.at (idx);
                  clustering_circle_inliers->indices.push_back (circle_inliers->indices.at (inl));
                }
              }
            }

            // ------------------------ //
            // Start extraction process //
            // ------------------------ //

            pcl::PointCloud<PointT>::Ptr clustering_circle_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers 
            pcl::ExtractIndices<PointT> clustering_extraction_of_circle;
            // Set which indices to extract
            clustering_extraction_of_circle.setIndices (clustering_circle_inliers);
            // Set point cloud from where to extract
            clustering_extraction_of_circle.setInputCloud (working_cluster_cloud);

            // Return the points which represent the inliers
            clustering_extraction_of_circle.setNegative (false);
            // Call the extraction function
            clustering_extraction_of_circle.filter (*clustering_circle_inliers_cloud);

            std::stringstream after_clustering_id;
            after_clustering_id << "CIRCLE_INLIERS_" << ros::Time::now();
            circle_viewer.addPointCloud (*clustering_circle_inliers_cloud, after_clustering_id.str ());
            circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_clustering_id.str ()); 
            circle_viewer.spin ();
            circle_viewer.removePointCloud (after_clustering_id.str());
            circle_viewer.spin ();

            // Update the circle inliers cloud
            *circle_inliers_cloud = *clustering_circle_inliers_cloud;

            // Update the circle inliers
            *circle_inliers = *clustering_circle_inliers;
          }
        }



        // START W/ THE CURVATURE FEATURE //

        if ( curvature_feature )
        {

          if ( (int) circle_inliers->indices.size() < minimum_circle_inliers )
          {
            // The current circle model will be rejected
            valid_circle = false;
          }
          else
          {
            std::stringstream before_curvature_id;
            before_curvature_id << "CIRCLE_INLIERS_" << ros::Time::now();
            circle_viewer.addPointCloud (*circle_inliers_cloud, before_curvature_id.str ());
            circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_curvature_id.str ()); 
            circle_viewer.spin ();
            circle_viewer.removePointCloud (before_curvature_id.str());
            circle_viewer.spin ();

            pcl::PointIndices::Ptr curvature_circle_inliers (new pcl::PointIndices ());

            for (int inl = 0; inl < (int) circle_inliers->indices.size(); inl++)
            {
              int idx = circle_inliers->indices.at (inl);

              double curvature = working_cluster_cloud->points.at (idx).curvature;

              if ( curvature_threshold < curvature )
                curvature_circle_inliers->indices.push_back (idx);
            }

            // ------------------------ //
            // Start extraction process //
            // ------------------------ //

            pcl::PointCloud<PointT>::Ptr curvature_circle_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers 
            pcl::ExtractIndices<PointT> curvature_extraction_from_circle;
            // Set which indices to extract
            curvature_extraction_from_circle.setIndices (curvature_circle_inliers);
            // Set point cloud from where to extract
            curvature_extraction_from_circle.setInputCloud (working_cluster_cloud);

            // Return the points which represent the inliers
            curvature_extraction_from_circle.setNegative (false);
            // Call the extraction function
            curvature_extraction_from_circle.filter (*curvature_circle_inliers_cloud);

            std::stringstream after_curvature_id;
            after_curvature_id << "CIRCLE_INLIERS_" << ros::Time::now();
            circle_viewer.addPointCloud (*curvature_circle_inliers_cloud, after_curvature_id.str ());
            circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_curvature_id.str ()); 
            circle_viewer.spin ();
            circle_viewer.removePointCloud (after_curvature_id.str());
            circle_viewer.spin ();

            // Update the inliers of circle
            *circle_inliers = *curvature_circle_inliers;

            // Update the points of inliers
            *circle_inliers_cloud = *curvature_circle_inliers_cloud;
          }
        }



        // START W/ THE RSD VALUES // 

        if ( rsd_feature )
        {

          if ( (int) circle_inliers->indices.size() < minimum_circle_inliers )
          {
            // The current circle model will be rejected
            valid_circle = false;
          }
          else
          {
            std::stringstream before_rsd_id;
            before_rsd_id << "CIRCLE_INLIERS_" << ros::Time::now();
            circle_viewer.addPointCloud (*circle_inliers_cloud, before_rsd_id.str ());
            circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_rsd_id.str ()); 
            circle_viewer.spin ();
            circle_viewer.removePointCloud (before_rsd_id.str());
            circle_viewer.spin ();

            pcl::PointIndices::Ptr rsd_circle_inliers (new pcl::PointIndices ());

            double rc = circle_coefficients.values [2];

            for (int inl = 0; inl < (int) circle_inliers->indices.size(); inl++)
            {
              int idx = circle_inliers->indices.at (inl);

              //double r_min = rsd_working_cloud->points.at (working_cluster_indices.indices.at (idx)).r_min; 

              double rp = rsd_working_cloud->points.at (working_cluster_indices.indices.at (idx)).r_min;

              //if ( (low_r_min < r_min) && (r_min < high_r_min) ) 

              if ( fabs (rc - rp) < radius_threshold )
              {
                rsd_circle_inliers->indices.push_back (idx);
              }
            }

            // ---------------------------- //
            // Start the extraction process //
            // ---------------------------- //

            pcl::PointCloud<PointT>::Ptr rsd_circle_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers 
            pcl::ExtractIndices<PointT> rsd_extraction_of_circle;
            // Set which indices to extract
            rsd_extraction_of_circle.setIndices (rsd_circle_inliers);
            // Set point cloud from where to extract
            //rsd_extraction_of_circle.setInputCloud (working_cluster_cloud);
            rsd_extraction_of_circle.setInputCloud (working_cluster_cloud);

            // Return the points which represent the inliers
            rsd_extraction_of_circle.setNegative (false);
            // Call the extraction function
            rsd_extraction_of_circle.filter (*rsd_circle_inliers_cloud);

            // Erase those indices which have an implausible rsd value
            std::vector<int>::iterator it = working_cluster_indices.indices.begin();
            for (int inl = circle_inliers->indices.size() - 1; inl > -1; inl--)
            {
              int idx = circle_inliers->indices.at (inl);
              working_cluster_indices.indices.erase (it + idx);
            }

            std::stringstream after_rsd_id;
            after_rsd_id << "CIRCLE_INLIERS_" << ros::Time::now();
            circle_viewer.addPointCloud (*rsd_circle_inliers_cloud, after_rsd_id.str ());
            circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_rsd_id.str ()); 
            circle_viewer.spin ();
            circle_viewer.removePointCloud (after_rsd_id.str());
            circle_viewer.spin ();

            // Update the inliers of circle
            *circle_inliers = *rsd_circle_inliers;

            // Update the points of inliers
            *circle_inliers_cloud = *rsd_circle_inliers_cloud;
          }
        }



        // START W/ THE NORMALS FEATURE //

        if ( normals_feature )
        {
          std::stringstream before_normals_id;
          before_normals_id << "CIRCLE_INLIERS_" << ros::Time::now();
          circle_viewer.addPointCloud (*circle_inliers_cloud, before_normals_id.str ());
          circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_normals_id.str ()); 
          circle_viewer.spin ();
          circle_viewer.removePointCloud (before_normals_id.str());
          circle_viewer.spin ();

          pcl::PointIndices::Ptr normals_circle_inliers (new pcl::PointIndices ());

          float c[2];
          c[0] = circle_coefficients.values.at (0);
          c[1] = circle_coefficients.values.at (1);

          for (int inl = 0; inl < (int) circle_inliers->indices.size(); inl++)
          {
            int idx = circle_inliers->indices.at (inl);

            float p[2];
            p[0] = working_cluster_cloud->points.at (idx).x;
            p[1] = working_cluster_cloud->points.at (idx).y;

            float c2p[2];
            c2p[0] = p[0] - c[0];
            c2p[1] = p[1] - c[1];

            float lc2p = sqrt (c2p[0]*c2p[0] + c2p[1]*c2p[1]);
            c2p[0] = c2p[0] / lc2p;
            c2p[1] = c2p[1] / lc2p;

            float np[2];
            np[0] = working_cluster_cloud->points.at (idx).normal_x;
            np[1] = working_cluster_cloud->points.at (idx).normal_y;

            float lnp = sqrt (np[0]*np[0] + np[1]*np[1]);
            np[0] = np[0] / lnp;
            np[1] = np[1] / lnp;

            float dot = c2p[0]*np[0] + c2p[1]*np[1];
            float ang = acos (dot) * 180.0 / M_PI;

            if ( ((180.0 - angle_threshold) < ang) || (ang < angle_threshold) )
            {
              normals_circle_inliers->indices.push_back (idx);
            }
          }

          // ---------------------------- //
          // Start the extraction process //
          // ---------------------------- //

          pcl::PointCloud<PointT>::Ptr normals_circle_inliers_cloud (new pcl::PointCloud<PointT> ());

          // Extract the circular inliers 
          pcl::ExtractIndices<PointT> normals_extraction_of_circle;
          // Set which indices to extract
          normals_extraction_of_circle.setIndices (normals_circle_inliers);
          // Set point cloud from where to extract
          normals_extraction_of_circle.setInputCloud (working_cluster_cloud);

          // Return the points which represent the inliers
          normals_extraction_of_circle.setNegative (false);
          // Call the extraction function
          normals_extraction_of_circle.filter (*normals_circle_inliers_cloud);

          std::stringstream after_normals_id;
          after_normals_id << "CIRCLE_INLIERS_" << ros::Time::now();
          circle_viewer.addPointCloud (*normals_circle_inliers_cloud, after_normals_id.str ());
          circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_normals_id.str ()); 
          circle_viewer.spin ();
          circle_viewer.removePointCloud (after_normals_id.str());
          circle_viewer.spin ();

          // Update the inliers of circle
          *circle_inliers = *normals_circle_inliers;

          // Update the points of inliers
          *circle_inliers_cloud = *normals_circle_inliers_cloud;
        }






        // THIS IS ON, BECAUSE WE DO NOT NEED THE ANYMORE
        // Return the remaining points of inliers
        extraction_of_circle.setNegative (true);
        // Call the extraction function
        extraction_of_circle.filter (*working_cluster_cloud);

        //ROS_INFO ("Circle has %d inliers", (int) circle_inliers_cloud->points.size());
        //ROS_INFO ("%d points remain after extraction", (int) working_cluster_cloud->points.size ());



        valid_circle = true;
        //        // Check if the fitted circle has enough inliers in order to be accepted
        //        if ((int) circle_inliers->indices.size () < minimum_circle_inliers)
        if ( !valid_circle )
        {
          ROS_ERROR ("NOT ACCEPTED ! Circle [%2d] has %3d inliers with C = (%6.3f,%6.3f) and R = %5.3f in [%5.3f, %5.3f] found in maximum %d iterations",
              circle_fit, (int) circle_inliers->indices.size (), circle_coefficients.values [0], circle_coefficients.values [1], circle_coefficients.values [2], minimum_radius, maximum_radius, maximum_circle_iterations);

          // No need for fitting circles anymore
          stop_circle_fitting = true;
        }
        else
        {
          ROS_INFO ("ACCEPTED ! Circle [%2d] has %3d inliers with C = (%6.3f,%6.3f) and R = %5.3f in [%5.3f, %5.3f] found in maximum %d iterations",
              circle_fit, (int) circle_inliers->indices.size (), circle_coefficients.values [0], circle_coefficients.values [1], circle_coefficients.values [2], minimum_radius, maximum_radius, maximum_circle_iterations);

          // ------------------------------------- //
          // Build the parameter space for circles //
          // ------------------------------------- //

          // A vote consists of the actual circle parameters
          PointT circle_vot;
          circle_vot.x = circle_coefficients.values [0];
          circle_vot.y = circle_coefficients.values [1];
          circle_vot.z = circle_coefficients.values [2];

          // Cast one vot for the current circle
          circle_parameters_cloud->points.push_back (circle_vot);

          // --------------------------- //
          // Start visualization process //
          // --------------------------- //

          // Create ID for circle model
          std::stringstream circle_id;
          circle_id << "CIRCLE_" << ros::Time::now();

          // Create ID for circle inliers
          std::stringstream circle_inliers_id;
          circle_inliers_id << "CIRCLE_INLIERS_" << ros::Time::now();

          // Add circle model to point cloud data
          circle_viewer.addCircle (circle_coefficients, circle_id.str ());

          // Add the point cloud data
          circle_viewer.addPointCloud (*circle_inliers_cloud, circle_inliers_id.str ());

          // Set the size of points for cloud data
          circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, circle_inliers_id.str ()); 

          // Wait or not wait
          if ( circle )
          {
            // And wait until Q key is pressed
            circle_viewer.spin ();
          }

          // Save circle ids for cleaning the viewer afterwards
          circles_ids.push_back (circle_id.str());
          // Save also circle inliers ids
          circles_inliers_ids.push_back (circle_inliers_id.str());

          // Fit only one model for each cluster in every iteration
          // No need for fitting circles anymore
          //          stop_circle_fitting = true;



        }

        // number of fitted circles
        circle_fit++;

        // --------------------------------------------------- //
        // Check for continuing with the fitting of 2D circles //
        // --------------------------------------------------- //

        // Print the number of points left for model fitting
        if ( (int) working_cluster_cloud->points.size () < minimum_circle_inliers )
          ROS_WARN ("  %d < %d | Stop !", (int) working_cluster_cloud->points.size (), minimum_circle_inliers);
        else
          if ( (int) working_cluster_cloud->points.size () > minimum_circle_inliers )
            ROS_WARN ("  %d > %d | Continue... ", (int) working_cluster_cloud->points.size (), minimum_circle_inliers);
          else
            ROS_WARN ("  %d = %d | Continue... ", (int) working_cluster_cloud->points.size (), minimum_circle_inliers);

      } while ((int) working_cluster_cloud->points.size () > minimum_circle_inliers && stop_circle_fitting == false);




      // Wait or not wait
      if ( circle )
      {
        // And wait until Q key is pressed
        circle_viewer.spin ();
      }





    }

    // ---------------------- //
    // Start cleaning process //
    // ---------------------- //

    for (int id = 0; id < (int) circles_ids.size(); id++)
    {
      // Remove circle from the viewer
      circle_viewer.removeShape (circles_ids[id]);
    }

     // Wait or not wait
      if ( circle )
      {
        // And wait until Q key is pressed
        circle_viewer.spin ();
      }

    for (int id = 0; id < (int) circles_inliers_ids.size(); id++)
    {
      // Remove circle from the viewer
      circle_viewer.removePointCloud (circles_inliers_ids[id]);
    }

     // Wait or not wait
      if ( circle )
      {
        // And wait until Q key is pressed
        circle_viewer.spin ();
      }



  }











  // Create ID for circle model
  std::stringstream circle_parameters_id;
  circle_parameters_id << "CIRCLE_PARAMETERS_" << ros::Time::now();

  // Add the point cloud data
  circle_viewer.addPointCloud (*circle_parameters_cloud, circle_parameters_id.str ());

  // Set the size of points for cloud
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, circle_parameters_id.str ()); 

  // Wait or not wait
  if ( true )
  {
    // And wait until Q key is pressed
    circle_viewer.spin ();
  }

 // Wait or not wait
  if ( true )
  {
    // And wait until Q key is pressed
    circle_viewer.spin ();
  }

 // Wait or not wait
  if ( true )
  {
    // And wait until Q key is pressed
    circle_viewer.spin ();
  }




  exit (0);











/*

	void knnSearch(const Matrix<ElementType>& queries, Matrix<int>& indices, Matrix<DistanceType>& dists, int knn, const SearchParams& params);

	int radiusSearch(const Matrix<ElementType>& query, Matrix<int>& indices, Matrix<DistanceType>& dists, float radius, const SearchParams& params);

*/


/*
  KdTree<PointXYZ>::Ptr tree;

  tree = boost::make_shared<KdTreeFLANN<PointXYZ> > (false);
  tree-> setInputCloud (cloud.makeShared (), boost::make_shared <vector<int> > (indices));
*/

/*

  // Build kd-tree structure for clusters
  pcl::KdTreeFLANN<PointT>::Ptr circle_parameter_tree (new pcl::KdTreeFLANN<PointT> ());

  //const std::vector<int> &indices, const boost::shared_ptr<KdTree<PointT> > &tree,

  KdTree<PointXYZ>::Ptr tree;
  tree = boost::make_shared<KdTreeFLANN<PointXYZ> > (false);
  tree-> setInputCloud (cloud.makeShared (), boost::make_shared <vector<int> > (indices));


  std::vector<int> nnIndices;
  std::vector<float> nnSqrDistances;
  circle_parameter_tree->radiusSearch (3, 0.03, nnIndices, nnSqrDistances);

*/


 /** \brief Search for k-nearest neighbors for the given query point.
        * \param cloud the point cloud data
        * \param index the index in \a cloud representing the query point
        * \param k the number of neighbors to search for
        * \param k_indices the resultant indices of the neighboring points (must be resized to \a k a priori!)
        * \param k_distances the resultant squared distances to the neighboring points (must be resized to \a k 
        * a priori!)
        * \return number of neighbors found
        */


//  inline int nearestKSearch (const PointCloud &cloud, int index, int k, std::vector<int> &k_indices, std::vector<float> &k_distances)

/*

  pcl::KdTreeFLANN<PointT>::Ptr circle_parameter_tree (new pcl::KdTreeFLANN<PointT> (circle_parameters_cloud));

  std::vector<int> k_indices (25);
  std::vector<float> k_distances (25);

  circle_parameter_tree->nearestKSearch (*circle_parameters_cloud, 1, 25, k_indices, k_distances);

*/




  // TODO Gaussian Filter for radii sizes ! 
   



  //pcl::KdTreeFLANN<PointT>::Ptr kdtree_ (new pcl::KdTreeFLANN<PointT> ());
  //pcl::KdTreeFLANN<PointT>::Ptr kdtree_ (new pcl::KdTreeFLANN<PointT> (circle_parameters_cloud));
  
  pcl::KdTreeFLANN<PointT>::Ptr kdtree_ (new pcl::KdTreeFLANN<PointT> (false));
  kdtree_-> setInputCloud (circle_parameters_cloud);


//  cloud_kdtree::KdTree *kdtree_;
//  kdtree_ = new cloud_kdtree::KdTreeANN (*circle_parameters_cloud);

  // decalre
  std::vector<std::vector<int> > points_indices_;
  std::vector<std::vector<float> > points_sqr_distances_;

  // clear
//  points_indices_.clear ();
//  points_sqr_distances_.clear ();

  // Allocate enough space for point indices and distances
  points_indices_.resize (circle_parameters_cloud->points.size ());
  points_sqr_distances_.resize (circle_parameters_cloud->points.size ());

  //  // Get the nearest neighbors for all points to be fitted
  //  ts = ros::Time::now ();

//  int neighborhood_size_ = 25;
//  double radius_ = 0.015;
//  int max_nn_ = 25;

//  cerr << circle_parameters_cloud->points.size () << endl ; 


// Space of parameters for fitted circle models
  pcl::PointCloud<PointT>::Ptr those_points (new pcl::PointCloud<PointT> ());


//int k_min = max_nn_, k_max = 0;
  for (size_t cp = 0; cp < circle_parameters_cloud->points.size (); cp++)
  {
    //    cerr << cp << endl ; 

    //    points_indices_[cp].resize (max_nn_);
    //    points_sqr_distances_[cp].resize (max_nn_);

    //kdtree_->nearestKSearch (*circle_parameters_cloud, cp, neighborhood_size_, points_indices_[cp], points_sqr_distances_[cp]);
    kdtree_->radiusSearch (*circle_parameters_cloud, cp, circle_space_radius, points_indices_[cp], points_sqr_distances_[cp]);

    //int k = points_indices_[cp].size ();
    //if (k > k_max) k_max = k;
    //if (k < k_min) k_min = k;

    if ( points_indices_[cp].size() > iterations * circle_space_percentage )
    {

      cerr << " index : " << cp << " found : " << points_indices_[cp].size() << " points " << endl ; 

      those_points->points.push_back (circle_parameters_cloud->points[cp]);

      //PointT point;
      //point.x = circle_parameters_cloud->points[cp].x;
      //point.y = circle_parameters_cloud->points[cp].y;
      //point.z = circle_parameters_cloud->points[cp].z;

    }

  }



/*
  // Get position of full stop in path of file
  std::string file = argv [pFileIndicesPCD[0]];
  size_t fullstop = file.find (".");
*/
  // Create file name for saving
  std::string circle_space_filename = argv [pFileIndicesPCD [0]];
  circle_space_filename.insert (fullstop, "-circles");

  // Save these points to disk
  pcl::io::savePCDFile (circle_space_filename, *those_points);

  if ( verbose )
  {
    // Show the floor's number of points
    ROS_INFO ("The space which is represented by the circles models has %d points and was saved !", (int) those_points->points.size ());
  }







  // Create ID for circle model
  std::stringstream those_points_id;
  those_points_id << "THOSE_POINTS_" << ros::Time::now();

  // Add the point cloud data
  circle_viewer.addPointCloud (*those_points, those_points_id.str ());

  // Set the size of points for cloud
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size * size, those_points_id.str ()); 

  // Wait or not wait
  if ( true )
  {
    // And wait until Q key is pressed
    circle_viewer.spin ();
  }






/*


  // -------------------------------------------------------------- //
  // ------------------ Cluster point cloud data ------------------ //
  // -------------------------------------------------------------- //

  // Vector of clusters from input cloud
  std::vector<pcl::PointIndices> objects_clusters;
  // Build kd-tree structure for clusters
  pcl::KdTreeFLANN<PointT>::Ptr objects_clusters_tree (new pcl::KdTreeFLANN<PointT> ());

  // Instantiate cluster extraction object
  pcl::EuclideanClusterExtraction<PointT> ece;
  // Set as input the cloud of handle
  ece.setInputCloud (working_cloud);
  // Radius of the connnectivity threshold
  ece.setClusterTolerance (clustering_tolerance_of_objects);
  // Minimum size of clusters
  ece.setMinClusterSize (minimum_size_of_objects_clusters);
  // Provide pointer to the search method
  ece.setSearchMethod (objects_clusters_tree);
  // Call the extraction function
  ece.extract (objects_clusters);

  if ( verbose )
  {
    ROS_INFO ("Euclidean Cluster Extraction ! Returned: %d clusters", (int) objects_clusters.size ());
  }

  // ------------------------------------------------------------- //
  // ------------------ Extract object clusters ------------------ //
  // ------------------------------------------------------------- //

  // Vector of indices which make up the objects clusters
  std::vector<pcl::PointIndices::Ptr> objects_clusters_indices;
  // Point clouds which represent the clusters of the objects
  std::vector<pcl::PointCloud<PointT>::Ptr> objects_clusters_clouds;
//  // Auxiliary vector of object clusters
//  std::vector<pcl::PointCloud<PointT>::Ptr> auxiliary_clusters_clouds;


  for (int c = 0; c < (int) objects_clusters.size(); c++)
  {
    // Cloud of the cluster obejct
    pcl::PointCloud<PointT>::Ptr object_cluster_cloud (new pcl::PointCloud<PointT> ());
    // Pointer to the cluster object
    pcl::PointIndices::Ptr object_cluster_indices (new pcl::PointIndices (objects_clusters.at(c)));

    // Extract handle points from the input cloud
    pcl::ExtractIndices<PointT> extraction_of_objects_clusters;
    // Set point cloud from where to extract
    extraction_of_objects_clusters.setInputCloud (working_cloud);
    // Set which indices to extract
    extraction_of_objects_clusters.setIndices (object_cluster_indices);
    // Return the points which represent the inliers
    extraction_of_objects_clusters.setNegative (false);
    // Call the extraction function
    extraction_of_objects_clusters.filter (*object_cluster_cloud);

    if ( verbose )
    {
      ROS_INFO ("  Objet %2d has %4d points", c, (int) object_cluster_cloud->points.size());
    }

    // Save the cloud of object cluster
    objects_clusters_clouds.push_back (object_cluster_cloud);

    // Save indices of object
    objects_clusters_indices.push_back (object_cluster_indices);

//    // Update auxiliary clusters
//    auxiliary_clusters_clouds.push_back (object_cluster_cloud);


  }




*/











/*






    // -------------------------------------------------------------------------------------------------------------------------------

    // Check if the method found inliers for the current circle model
    if ((int) circle_inliers->indices.size () == 0)
    {
      ROS_ERROR ("Could not estimate a circular model for the given dataset.");
      ROS_ERROR (" %d points remain unfitted.", (int) filtered_cloud->points.size ());

      // Points which remain unfitted
      pcl::PointCloud<PointT>::Ptr circle_unfitted_cloud (new pcl::PointCloud<PointT> ());
      // Save these points to disk
      pcl::io::savePCDFile ("data/circle-unfitted-cloud.pcd", *circle_unfitted_cloud);
*/
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
/*
      return (-1);
    }

    // Check if the fitted circle has enough inliers in order to be accepted
    if ((int) circle_inliers->indices.size () < minimum_circle_inliers)
    {
      ROS_ERROR ("NOT ACCEPTED ! Circle [%2d] has %3d inliers with C = (%6.3f,%6.3f) and R = %5.3f in [%5.3f, %5.3f] found in maximum %d iterations",
                 circle_fit, (int) circle_inliers->indices.size (), circle_coefficients.values [0], circle_coefficients.values [1], circle_coefficients.values [2], minimum_radius, maximum_radius, maximum_circle_iterations);

      // No need for fitting circles anymore
      stop_circles = true;
    }
    else
    {
      ROS_INFO ("ACCEPTED ! Circle [%2d] has %3d inliers with C = (%6.3f,%6.3f) and R = %5.3f in [%5.3f, %5.3f] found in maximum %d iterations",
                circle_fit, (int) circle_inliers->indices.size (), circle_coefficients.values [0], circle_coefficients.values [1], circle_coefficients.values [2], minimum_radius, maximum_radius, maximum_circle_iterations);

      // Build the space of parameters for circles //

      // A vote consists of the actual circle parameters
      PointT circle_vot;
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
    circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, circle_inliers_id.str ()); 

    // Wait or not wait
    if ( circle )
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
      ROS_ERROR (" %d < %d | Stop !", (int) filtered_cloud->points.size (), minimum_circle_inliers);
    else
      if ( (int) filtered_cloud->points.size () > minimum_circle_inliers )
        ROS_WARN (" %d > %d | Continue... ", (int) filtered_cloud->points.size (), minimum_circle_inliers);
      else
        ROS_WARN (" %d = %d | Continue... ", (int) filtered_cloud->points.size (), minimum_circle_inliers);

  } while ((int) filtered_cloud->points.size () > minimum_circle_inliers && stop_circles == false);

  // And wait until Q key is pressed
  circle_viewer.spin ();

  // Save these points to disk
  pcl::io::savePCDFile ("data/circle-parameters-cloud.pcd", *circle_parameters_cloud);

  // Add the point cloud data
  circle_viewer.addPointCloud (*circle_parameters_cloud, "CIRCLE_PARAMETER");

  // Set the size of points for cloud
  circle_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size * 2, "CIRCLE_PARAMETER"); 

  // Save these points to disk
  pcl::io::savePCDFile ("data/circle-rest-cloud.pcd", *filtered_cloud);

  // Done with 2D circle models
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Done with 2D circle models in %5.3g [s]", tt.toc ());

*/

  // ---------------------------------------------------------------------- //
  // ------------------ Recover filtered point cloud data ----------------- //
  // ---------------------------------------------------------------------- //
/*
  filtered_cloud = working_cloud;
*/

/*
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
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "INPUT"); 
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
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "FILTERED");
  // And wait until Q key is pressed
  line_viewer.spin ();
  */

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
/*
  int line_fit = 0;

  bool stop_lines = false;

  // Space of parameters for fitted circle models
  pcl::PointCloud<PointT>::Ptr line_parameters_cloud (new pcl::PointCloud<PointT> ());

  do
  {
    // Inliers of line model
    pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());
    // Coefficients of cirlce model
    pcl::ModelCoefficients line_coefficients;
    // Create the segmentation object
    pcl::SACSegmentation<PointT> line_segmentation;
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
      ROS_ERROR (" %d points remain unfitted.", (int) filtered_cloud->points.size ());

      // Points which remain unfitted
      pcl::PointCloud<PointT>::Ptr line_unfitted_cloud (new pcl::PointCloud<PointT> ());
      // Save these points to disk
      pcl::io::savePCDFile ("data/line-unfitted-cloud.pcd", *line_unfitted_cloud);
*/
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
/*
      return (-1);
    }

    // Check if the fitted lines has enough inliers in order to be accepted
    if ((int) line_inliers->indices.size () < minimum_line_inliers)
    {
      ROS_ERROR ("NOT ACCEPTED ! Line [%2d] has %3d inliers with Point = (%6.3f,%6.3f,%6.3f) and Direction = (%6.3f,%6.3f,%6.3f) found in maximum %d iterations", line_fit, (int) line_inliers->indices.size (),
                 line_coefficients.values [0], line_coefficients.values [1], line_coefficients.values [2], line_coefficients.values [3], line_coefficients.values [4], line_coefficients.values [5], maximum_line_iterations);

      // No need for fitting lines anymore
      stop_lines = true;
    }
    else
    {

      ROS_INFO ("ACCEPTED ! Line [%2d] has %3d inliers with Point = (%6.3f,%6.3f,%6.3f) and Direction = (%6.3f,%6.3f,%6.3f) found in maximum %d iterations", line_fit, (int) line_inliers->indices.size (),
                line_coefficients.values [0], line_coefficients.values [1], line_coefficients.values [2], line_coefficients.values [3], line_coefficients.values [4], line_coefficients.values [5], maximum_line_iterations);

      // Build the space of parameters for lines //
 
      // Computing the line's parameters
      double m = line_coefficients.values [4] / line_coefficients.values [3];
      double b = line_coefficients.values [1] - m * line_coefficients.values [0];

      // A vote consists of the actual line parameters
      PointT line_vot;
      line_vot.x = m;
      line_vot.y = b;

      // Cast one vot for the current line
      line_parameters_cloud->points.push_back (line_vot);
    }

    // ---------------------------- //
    // Start the extraction process //
    // ---------------------------- //

    // Point cloud of line inliers
    pcl::PointCloud<PointT>::Ptr line_inliers_cloud (new pcl::PointCloud<PointT> ());

    // Extract the circular inliers from the input cloud
    pcl::ExtractIndices<PointT> line_extraction;
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
*/
    /*

    ROS_INFO ("Line has %d inliers", line_inliers_cloud->points.size());
    ROS_INFO ("%d points remain after extraction", filtered_cloud->points.size ());

    */

    // --------------------------- //
    // Start visualization process //
    // --------------------------- //
/*
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
    line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, line_inliers_id.str ()); 

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
    pcl::KdTreeFLANN<PointT>::Ptr line_clusters_tree (new pcl::KdTreeFLANN<PointT> ());

    // Instantiate cluster extraction object
    pcl::EuclideanClusterExtraction<PointT> line_clustering;
    // Set as input the cloud of line inliers
    line_clustering.setInputCloud (line_inliers_cloud);
    // Radius of the connnectivity threshold
    line_clustering.setClusterTolerance (line_inliers_clustering_tolerance);
    // Provide pointer to the search method
    line_clustering.setSearchMethod (line_clusters_tree);
    // Call the extraction function
    line_clustering.extract (line_clusters);
*/
    /*

    ROS_WARN (" has %d clusters where", (int) line_clusters.size() );
    for (int c = 0; c < (int) line_clusters.size(); c++)
      ROS_WARN ("       cluster %d has %d points", c, (int) line_clusters.at(c).indices.size() );

    */
/*
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
      ROS_ERROR (" %d < %d | Stop !", (int) filtered_cloud->points.size (), minimum_line_inliers);
    else
      if ( (int) filtered_cloud->points.size () > minimum_line_inliers )
        ROS_WARN (" %d > %d | Continue... ", (int) filtered_cloud->points.size (), minimum_line_inliers);
      else
        ROS_WARN (" %d = %d | Continue... ", (int) filtered_cloud->points.size (), minimum_line_inliers);

  } while ((int) filtered_cloud->points.size () > minimum_line_inliers && stop_lines == false);

  // And wait until Q key is pressed
  line_viewer.spin ();

  // Save these points to disk
  pcl::io::savePCDFile ("data/line-parameters-cloud.pcd", *line_parameters_cloud);

  // Add the point cloud data
  line_viewer.addPointCloud (*line_parameters_cloud, "LINE_PARAMETER");

  // Set the size of points for cloud
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size * 2, "LINE_PARAMETER"); 

  // Save these points to disk
  pcl::io::savePCDFile ("data/line-rest-cloud.pcd", *filtered_cloud);

  // Done with 2D line models
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Done with 2D line models in %5.3g [s]", tt.toc ());


*/







  // TODO Computing the best votes and segmenting the point cloud data



  if ( verbose )
  {
    // Displaying the overall time
    ROS_WARN ("Finished in %5.3g [s] !", tt.toc ());
  }

  // And wait until Q key is pressed
  circle_viewer.spin ();

/*
  // And wait until Q key is pressed
  line_viewer.spin ();
*/

  return (0);
}