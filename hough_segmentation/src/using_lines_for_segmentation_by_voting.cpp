/*
 * Copyright (c) 2011, Lucian Cosmin Goron <goron@in.tum.edu>
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
#include "pcl/sample_consensus/sac_model_line.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

#include "pcl_visualization/pcl_visualizer.h"

#include "pcl_cloud_algos/pcl_cloud_algos_point_types.h"

// --------------------------------------------------------------- //
// -------------------- Declare defs of types -------------------- //
// --------------------------------------------------------------- //

typedef pcl::PointXYZRGB Point;
typedef pcl::PointXYZINormal PointT;
typedef pcl::PointXYZINormalRSD PointTrsd;

// --------------------------------------------------------------- //
// -------------------- Define useful structs -------------------- //
// --------------------------------------------------------------- //

struct shape
{
  std::string type;
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::PointCloud<PointT> points;
  pcl::PointCloud<PointT> cluster;
};

// --------------------------------------------------------------------- //
// -------------------- Declare method's parameters -------------------- //
// --------------------------------------------------------------------- //

int iterations = 100; 

int mean_k_filter = 25; /* [points] */
int std_dev_filter = 1.0;
int line_mean_k_filter = 25; /* [points] */
int line_std_dev_filter = 1.0; 

// Clustering's Parameters
int minimum_size_of_objects_clusters = 100; /* [points] */
double clustering_tolerance_of_objects = 0.025; /* [meters] */

int line_minimum_size_of_objects_clusters = 10; /* [points] */
double line_clustering_tolerance_of_objects = 0.010; /* [meters] */

// Fitting's Parameters
double   line_threshold = 0.010; /// [meters]
double voting_threshold =  0.25; /// [percentage]
double minimum_radius = 0.010; /// [meters]
double maximum_radius = 0.100; /// [meters]
int minimum_line_inliers = 10; /// [points]
int maximum_line_iterations = 1000; /// [iterations]
double line_clustering_tolerance = 0.010; /// [meters]
int minimum_size_of_line_cluster = 10; /// [points]

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
bool percentage_feature = true;

double line_percentage = 50;
double clustering_tolerance_of_line_parameters = 0.025;
double minimum_size_of_line_parameters_clusters = 50;

double height = 0.010;
double epsilon = 0.010;

// Visualization's Parameters
int size = 3;
bool step = false;
bool color = false;
bool verbose = false;
bool line_step = false;
bool line_feature_step = false;



ofstream textfile;



#define _sqr(c) ((c)*(c))



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Computes line's inliers with regards to xOy plane
 * \param cloud The point cloud of the working cluster
 * \param inliers The inliers of the line model
 * \param coefficients The line's parameters where the first triplete is a point on the line and the second triplete is the direction
 */
void adjustLineInliers (pcl::PointCloud<PointT>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients &coefficients, double threshold)
{

  // First point of line
  double P1[2];
  P1[0] = coefficients.values [0];
  P1[1] = coefficients.values [1];

  // Second point of line
  double P2[2];
  P2[0] = coefficients.values [3] + coefficients.values [0];
  P2[1] = coefficients.values [4] + coefficients.values [1];

  // Set-up of variables
  double x1 = P1[0];
  double y1 = P1[1];
  double x2 = P2[0];
  double y2 = P2[1];

  for (unsigned int idx = 0; idx < cloud->points.size (); idx++)
  {
    double x0 =  cloud->points.at (idx).x;
    double y0 =  cloud->points.at (idx).y;

    double d = fabs( (x2-x1)*(y1-y0) - (x1-x0)*(y2-y1) ) / sqrt( (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1) );    
    
    if ( fabs(d) < threshold ) 
      inliers->indices.push_back (idx);
  }

  return;
}



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Computes line's coefficients with regards to its inliers
 * \param inliers_cloud The point cloud of the line's inliers
 * \param coefficients The line's parameters where the first triplete is a point on the line and the second triplete is the direction
 */
void adjustLineCoefficients (pcl::PointCloud<PointT>::Ptr &cloud, pcl::ModelCoefficients &coefficients)
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
  for (int idx = 0; idx < (int)cloud->points.size (); idx++)
  {
    double P[2];
    P[0] = cloud->points.at(idx).x - P1[0];
    P[1] = cloud->points.at(idx).y - P1[1];

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

  return;
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
    ROS_INFO ("    -iterations X                                      = How many times to run the fitting routine.");
    ROS_INFO (" ");
    ROS_INFO ("    -line_threshold X                                  = threshold for line inlier selection");
    ROS_INFO ("    -line_threshold X                                = threshold for line inlier selection");
    ROS_INFO ("    -voting_threshold X                                = threshold for Hough-based model voting");
    ROS_INFO ("    -minimum_radius X                                  = ");
    ROS_INFO ("    -maximum_radius X                                  = ");
    ROS_INFO ("    -minimum_line_inliers D                            = ");
    ROS_INFO ("    -minimum_line_inliers D                          = ");
    ROS_INFO ("    -maximum_line_iterations D                         = ");
    ROS_INFO ("    -maximum_line_iterations D                       = ");
    ROS_INFO ("    -line_clustering_tolerance X                       = ");
    ROS_INFO ("    -line_clustering_tolerance X                     = ");
    ROS_INFO (" ");

    ROS_INFO ("    -mean_k_filter X                 = ");
    ROS_INFO ("    -std_dev_filter X                 = ");
    ROS_INFO ("    -line_mean_k_filter X                 = ");
    ROS_INFO ("    -line_std_dev_filter X                 = ");

    ROS_INFO ("    -minimum_size_of_objects_clusters X                = ");
    ROS_INFO ("    -clustering_tolerance_of_objects X                 = ");
    ROS_INFO (" ");



    ROS_INFO ("    -normals_search_knn X                              = ");
    ROS_INFO ("    -normals_search_radius X                           = ");
    ROS_INFO ("    -curvature_threshold X                             = ");
    ROS_INFO ("    -rsd_search_radius X                               = ");
    ROS_INFO ("    -rsd_plane_radius X                                = ");
    ROS_INFO ("    -low_r_min X                                       = ");
    ROS_INFO ("    -high_r_min X                                      = ");
    ROS_INFO ("    -angle_threshold X                                 = ");
    ROS_INFO ("    -radius_threshold X                                = ");



    ROS_INFO ("    -clustering_feature X                              = ");
    ROS_INFO ("    -curvature_feature X                               = ");
    ROS_INFO ("    -rsd_feature X                                     = ");
    ROS_INFO ("    -normals_feature X                                 = ");
    ROS_INFO ("    -percentage_feature X                              = ");


    ROS_INFO ("    -line_percentage X                               = ");
    ROS_INFO ("    -clustering_tolerance_of_line_parameters X       = ");
    ROS_INFO ("    -minimum_size_of_line_parameters_clusters X      = ");

    ROS_INFO ("    -size B                                            = ");
    ROS_INFO ("    -step B                                            = ");
    ROS_INFO ("    -verbose B                                         = ");
    ROS_INFO ("    -line_step B                                       = wait or not wait");
    ROS_INFO ("    -line_step B                                     = wait or not wait");
    ROS_INFO ("    -line_feature_step B                             = wait or not wait");
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

  terminal_tools::parse_argument (argc, argv,   "-iterations", iterations);

  terminal_tools::parse_argument (argc, argv, "-mean_k_filter", mean_k_filter);
  terminal_tools::parse_argument (argc, argv, "-std_dev_filter", std_dev_filter);
  terminal_tools::parse_argument (argc, argv, "-line_mean_k_filter", line_mean_k_filter);
  terminal_tools::parse_argument (argc, argv, "-line_std_dev_filter", line_std_dev_filter);

  // Parsing parameters for clustering
  terminal_tools::parse_argument (argc, argv, "-clustering_tolerance_of_objects", clustering_tolerance_of_objects);
  terminal_tools::parse_argument (argc, argv, "-minimum_size_of_objects_clusters", minimum_size_of_objects_clusters);

  terminal_tools::parse_argument (argc, argv, "-line_clustering_tolerance_of_objects", line_clustering_tolerance_of_objects);
  terminal_tools::parse_argument (argc, argv, "-line_minimum_size_of_objects_clusters", line_minimum_size_of_objects_clusters);

  // Parsing the arguments of the method
  terminal_tools::parse_argument (argc, argv,   "-line_threshold",   line_threshold);
  terminal_tools::parse_argument (argc, argv, "-line_threshold", line_threshold);
  terminal_tools::parse_argument (argc, argv, "-voting_threshold", voting_threshold);
  terminal_tools::parse_argument (argc, argv, "-minimum_radius", minimum_radius);
  terminal_tools::parse_argument (argc, argv, "-maximum_radius", maximum_radius);
  terminal_tools::parse_argument (argc, argv, "-minimum_line_inliers",   minimum_line_inliers);
  terminal_tools::parse_argument (argc, argv, "-minimum_line_inliers", minimum_line_inliers);
  terminal_tools::parse_argument (argc, argv, "-maximum_line_iterations",   maximum_line_iterations);
  terminal_tools::parse_argument (argc, argv, "-maximum_line_iterations", maximum_line_iterations);
  terminal_tools::parse_argument (argc, argv,   "-line_clustering_tolerance",   line_clustering_tolerance);
  terminal_tools::parse_argument (argc, argv, "-line_clustering_tolerance", line_clustering_tolerance);
  terminal_tools::parse_argument (argc, argv, "-minimum_size_of_line_cluster", minimum_size_of_line_cluster);
  terminal_tools::parse_argument (argc, argv, "-minimum_size_of_line_cluster", minimum_size_of_line_cluster);

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
  terminal_tools::parse_argument (argc, argv, "-percentage_feature", percentage_feature);

  terminal_tools::parse_argument (argc, argv, "-line_percentage", line_percentage);
  terminal_tools::parse_argument (argc, argv, "-clustering_tolerance_of_line_parameters", clustering_tolerance_of_line_parameters);
  terminal_tools::parse_argument (argc, argv, "-minimum_size_of_line_parameters_clusters", minimum_size_of_line_parameters_clusters);

  terminal_tools::parse_argument (argc, argv, "-height", height);
  terminal_tools::parse_argument (argc, argv, "-epsilon", epsilon);

  // Parsing the arguments for visualization
  terminal_tools::parse_argument (argc, argv, "-size", size);
  terminal_tools::parse_argument (argc, argv, "-step", step);
  terminal_tools::parse_argument (argc, argv, "-color", color);
  terminal_tools::parse_argument (argc, argv, "-verbose", verbose);
  terminal_tools::parse_argument (argc, argv, "-line_step", line_step);
  terminal_tools::parse_argument (argc, argv, "-line_step", line_step);
  terminal_tools::parse_argument (argc, argv, "-line_feature_step", line_feature_step);

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
  viewer.setBackgroundColor (1.0, 1.0, 1.0);
//  // Add system coordiante to viewer
//  viewer.addCoordinateSystem (1.0f);
  // Parse the camera settings and update the internal camera
  viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render
  viewer.updateCamera ();

  // ----------------------------------------------------------- //
  // ------------------ Load point cloud data ------------------ //
  // ----------------------------------------------------------- //

  // The kinect input point cloud data
  pcl::PointCloud<Point>::Ptr the_kinect_input_cloud (new pcl::PointCloud<Point> ());

  // Load point cloud data
  if (pcl::io::loadPCDFile (argv [pFileIndicesPCD [0]], *the_kinect_input_cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv [pFileIndicesPCD [0]]);
    return (-1);
  }

  if ( verbose )
  {
    ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (the_kinect_input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*the_kinect_input_cloud).c_str ());
  }

  // Add the point cloud data
  viewer.addPointCloud (*the_kinect_input_cloud, "KINECT INPUT DATA");
  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "KINECT INPUT DATA");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "KINECT INPUT DATA"); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // ------------------------------------------------------------------ //
  // ------------------ TEXT FILE W/ SIZES OF MODELS ------------------ //
  // ------------------------------------------------------------------ //

  std::stringstream textname;

//  textname << "cylinder-sizes-" << ros::Time::now() << ".txt";

  textname << "cylinder-sizes-hough-ransac.txt";

  textfile.open (textname.str ().c_str (), ios::app);

  textfile << "\n" << std::flush;

  std::string p =  argv [pFileIndicesPCD [0]];

  size_t s = p.find_last_of ("/");

  size_t l = p.length ();

  std::string n = p.substr (s + 1, l);

  textfile << "   file " << n << "\n" << std::flush;

  // ------------------------------------------- //
  // ------------------------------------------- //
  // ------------------------------------------- //
  // ------------------ PATCH ------------------ //
  // ------------------------------------------- //
  // ------------------------------------------- //
  // ------------------------------------------- //

  // Input point cloud data
  pcl::PointCloud<PointT>::Ptr input_cloud (new pcl::PointCloud<PointT> ());

  unsigned int size_of_the_kinect_input_cloud = the_kinect_input_cloud->points.size();

  for (unsigned int kin = 0; kin < the_kinect_input_cloud->points.size (); kin++)
  {
    input_cloud->points.resize (size_of_the_kinect_input_cloud);

    input_cloud->points.at (kin).x = the_kinect_input_cloud->points.at (kin).x;
    input_cloud->points.at (kin).y = the_kinect_input_cloud->points.at (kin).y;
    input_cloud->points.at (kin).z = the_kinect_input_cloud->points.at (kin).z;
    
    input_cloud->points.at (kin).intensity = 0.0;
    input_cloud->points.at (kin).normal_x  = 0.0;
    input_cloud->points.at (kin).normal_y  = 0.0;
    input_cloud->points.at (kin).normal_z  = 0.0;
    input_cloud->points.at (kin).curvature = 0.0;
  }

  // Working point cloud data 
  pcl::PointCloud<PointT>::Ptr working_cloud (new pcl::PointCloud<PointT> ());

  // Update working point cloud
  *working_cloud = *input_cloud;

/*

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
  sor.setMeanK (mean_k_filter);
  // Set the standard deviation multiplier threshold
  sor.setStddevMulThresh (std_dev_filter);
  // Call the filtering method
  sor.filter (*filtered_cloud);

  if ( verbose )
  {
    ROS_INFO ("Statistical Outlier Removal ! Before: %d points | After: %d points | Filtered: %d points",
              (int) working_cloud->points.size (),  (int) filtered_cloud->points.size (), (int) working_cloud->points.size () - (int) filtered_cloud->points.size ());
  }

  if ( step )
  {
    // Add the filtered point cloud data in the same viewer
    viewer.addPointCloud (*filtered_cloud, "FILTERED");
    // Color the filtered points in blue
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "FILTERED");
    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "FILTERED");
    // And wait until Q key is pressed
    viewer.spin ();

    viewer.removePointCloud ("INPUT");
    viewer.spin ();
  }

  // Update working point cloud
  *working_cloud = *filtered_cloud;

*/

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
    ROS_INFO ("Normal Estimation ! Returned: %d 3D normals", (int) normals_cloud->points.size ());
  }

//  if ( false )
  if ( step )
  {
    // Add the point cloud of normals
    viewer.addPointCloudNormals (*working_cloud, *normals_cloud, 1, 0.025, "3D NORMALS");
    // Color the normals with red
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "3D NORMALS"); 
    // And wait until Q key is pressed
    viewer.spin ();

    // Remove the point cloud data
    viewer.removePointCloud ("3D NORMALS");
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

  if ( verbose )
  {
    ROS_INFO ("Curvature Estimation ! Returned: %d curvatures", (int) normals_cloud->points.size ());
  }

  if ( step )
  {
    // Set the color handler for curvature
    pcl_visualization::PointCloudColorHandlerGenericField<PointT> curvature_handler (*working_cloud, "curvature");
    // Add the point cloud of curvature values
    viewer.addPointCloud (*working_cloud, curvature_handler, "CURVATURE");
    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "CURVATURE"); 
    // And wait until Q key is pressed
    viewer.spin ();

    // Remove curvature point cloud
    viewer.removePointCloud ("CURVATURE");
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
      // Save the indices from planar surfaces
      curvature_planar_indices->indices.push_back (idx);
    }
    else
    {
      // Save the indices from circular surfaces
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
  if ( step )
  {
    std::stringstream curvature_planar_id;
    curvature_planar_id << "CURVATURE_PLANAR_" << ros::Time::now();
    viewer.addPointCloud (*curvature_planar_cloud, curvature_planar_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_planar_id.str ()); 
    viewer.spin ();

    viewer.removePointCloud (curvature_planar_id.str());
    viewer.spin ();

    std::stringstream curvature_circular_id;
    curvature_circular_id << "CURVATURE_CIRCULAR_" << ros::Time::now();
    viewer.addPointCloud (*curvature_circular_cloud, curvature_circular_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_circular_id.str ()); 
    viewer.spin ();

    viewer.removePointCloud (curvature_circular_id.str());
    viewer.spin ();
  }
  */

  if ( verbose )
  {
    ROS_INFO ("Curvature Mapping ! Returned: %d planars vs %d circulars", (int) curvature_planar_cloud->points.size (), (int) curvature_circular_cloud->points.size ());
  }

  if ( step )
  {
    std::stringstream curvature_planar_id;
    curvature_planar_id << "CURVATURE_PLANAR_" << ros::Time::now();
    viewer.addPointCloud (*curvature_planar_cloud, curvature_planar_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_planar_id.str ()); 

    std::stringstream curvature_circular_id;
    curvature_circular_id << "CURVATURE_CIRCULAR_" << ros::Time::now();
    viewer.addPointCloud (*curvature_circular_cloud, curvature_circular_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, curvature_circular_id.str ()); 

    viewer.spin ();

    viewer.removePointCloud (curvature_planar_id.str());
    viewer.removePointCloud (curvature_circular_id.str());

    viewer.spin ();
  }

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

  std::string path =  argv [pFileIndicesPCD [0]];
  size_t fullstop = path.find (".");
  // Create name for saving pcd files
  std::string name = argv [1];
  name.insert (fullstop, "-rsd");

  // Declare the rsd working cloud of points
  pcl::PointCloud<PointTrsd>::Ptr rsd_working_cloud (new pcl::PointCloud<PointTrsd> ());
  // Concatenate radii values with the working cloud
  pcl::concatenateFields (*working_cloud, *rsd_cloud, *rsd_working_cloud);
  // Save these points to disk
//  pcl::io::savePCDFile (name, *rsd_working_cloud);

  if ( verbose )
  {
    ROS_INFO ("RSD Estimation ! Returned: %d rsd values", (int) rsd_working_cloud->points.size ());
  }

  if ( step )
  {
    // Set the color handler for curvature
    pcl_visualization::PointCloudColorHandlerGenericField<PointTrsd> rsd_handler (*rsd_working_cloud, "r_min");
    // Add the point cloud of curvature values
    viewer.addPointCloud (*rsd_working_cloud, rsd_handler, "R_MIN");
    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "R_MIN"); 
    // And wait until Q key is pressed
    viewer.spin ();

    // Remove r min point cloud
    viewer.removePointCloud ("R_MIN");
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
      // Save the right indices of points
      r_min_plausible_indices->indices.push_back (idx);
    }
    else
    {
      // Save the wrong ones
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
  if ( step )
  {
    std::stringstream r_min_plausible_id;
    r_min_plausible_id << "CURVATURE_PLANAR_" << ros::Time::now();
    viewer.addPointCloud (*r_min_plausible_cloud, r_min_plausible_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_plausible_id.str ()); 
    viewer.spin ();

    viewer.removePointCloud (r_min_plausible_id.str());
    viewer.spin ();

    std::stringstream r_min_implausible_id;
    r_min_implausible_id << "CURVATURE_CIRCULAR_" << ros::Time::now();
    viewer.addPointCloud (*r_min_implausible_cloud, r_min_implausible_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_implausible_id.str ()); 
    viewer.spin ();

    viewer.removePointCloud (r_min_implausible_id.str());
    viewer.spin ();
  }
  */

  if ( verbose )
  {
    ROS_INFO ("RSD Mapping ! Returned: %d plausibles vs %d implausibles", (int) r_min_plausible_cloud->points.size (), (int) r_min_implausible_cloud->points.size ());
  }

  if ( step )
  {
    std::stringstream r_min_plausible_id;
    r_min_plausible_id << "R_MIN_PLAUSIBLE_" << ros::Time::now();
    viewer.addPointCloud (*r_min_plausible_cloud, r_min_plausible_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_plausible_id.str ()); 

    std::stringstream r_min_implausible_id;
    r_min_implausible_id << "R_MIN_IMPLAUSIBLE_" << ros::Time::now();
    viewer.addPointCloud (*r_min_implausible_cloud, r_min_implausible_id.str ());
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, r_min_implausible_id.str ()); 

    viewer.spin ();

    viewer.removePointCloud (r_min_plausible_id.str());
    viewer.removePointCloud (r_min_implausible_id.str());

    viewer.spin ();
  }

  // ATTENTION // 

  // Save the r_min values as intensity values
  for (int idx = 0; idx < (int) working_cloud->points.size (); idx++)
  {
    // This is a quick, but temporary fix of the problem
    working_cloud->points.at (idx).intensity = rsd_cloud->points.at (idx).r_min;
  }

  // ----------------------------------------------------------------------- //
  // -------------------- Estimate 2D normals of points -------------------- //
  // ----------------------------------------------------------------------- //

  for (int idx = 0; idx < (int) normals_cloud->points.size (); idx++)
  {
    // Force normals from 3D in 2D
    normals_cloud->points[idx].normal_z = 0.0;  
  }

  if ( verbose )
  {
    ROS_INFO ("Normal Flattening ! Returned: %d 2D normals", (int) rsd_working_cloud->points.size ());
  }

//  if ( false )
  if ( step )
  {
    // Add the normals
    viewer.addPointCloudNormals (*working_cloud, *normals_cloud, 1, 0.025, "2D NORMALS");
    // Color the normals with red
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "2D NORMALS"); 
    // And wait until Q key is pressed
    viewer.spin ();

    // Remove the point cloud data
    viewer.removePointCloud ("2D NORMALS");
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

  if ( verbose )
  {
    ROS_INFO ("Normal Refinement ! Returned: %d normals", (int) rsd_working_cloud->points.size ());
  }

//  if ( false )
  if ( step )
  {
    // Add the normals
    viewer.addPointCloudNormals (*working_cloud, *normals_cloud, 1, 0.025, "NORMALS");
    // Color the normals with red
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "NORMALS"); 
    // And wait until Q key is pressed
    viewer.spin ();

    // Remove the point cloud data
    viewer.removePointCloud ("NORMALS");
    // And wait until Q key is pressed
    viewer.spin ();
  }

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
    pcl::PointIndices::Ptr object_cluster_indices (new pcl::PointIndices (objects_clusters.at (clu)));
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

  if ( step )
  {
    // Vector of ids of handles
    std::vector<std::string> objects_clusters_ids;

    for (int clu = 0; clu < (int) objects_clusters_clouds.size(); clu++)
    {
      // Create id for visualization
      std::stringstream object_cluster_id;
      object_cluster_id << "OBJECT_CLUSTER_" << ros::Time::now();

      // Add point cloud to viewer
      viewer.addPointCloud (*objects_clusters_clouds.at (clu), object_cluster_id.str());
      // Set the size of points for cloud
      viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, object_cluster_id.str()); 
      // And wait until Q key is pressed
//      viewer.spin ();

      // Save id of object
      objects_clusters_ids.push_back (object_cluster_id.str());
    }

    viewer.spin ();
  }






// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ //



  // -------------------------------------------------------------------------------------------------- //
  // -------------------------------------------------------------------------------------------------- //
  // ------------------------------------- Computation of 2D lines ------------------------------------ //
  // -------------------------------------------------------------------------------------------------- //
  // -------------------------------------------------------------------------------------------------- //

  // Open a 3D viewer
  pcl_visualization::PCLVisualizer line_viewer ("LINE VIEWER");
  // Set the background of viewer
  line_viewer.setBackgroundColor (1.0, 1.0, 1.0);
//  // Add system coordiante to viewer
//  line_viewer.addCoordinateSystem (1.0f);
  // Parse the camera settings and update the internal camera
  line_viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render
  line_viewer.updateCamera ();

  // Add the point cloud data
  line_viewer.addPointCloud (*working_cloud, "WORKING");
  // Color the cloud in white
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "WORKING");
  // Set the size of points for cloud
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, "WORKING"); 
  // And wait until Q key is pressed
  line_viewer.spin ();

  // ---------------------- //
  // Start fitting 2D lines //
  // ---------------------- //

  // Space of parameters for fitted line models
  pcl::PointCloud<pcl::PointNormal>::Ptr line_parameters_cloud (new pcl::PointCloud<pcl::PointNormal> ());

  // Space of parameters for fitted line models, more or less, the data on how to reconstruct the model
  pcl::PointCloud<pcl::Histogram<6> >::Ptr line_parameters_cloud_histogram (new pcl::PointCloud<pcl::Histogram<6> > ());

  for (int ite = 0; ite < iterations; ite++)
  {

    // Vector of lines ids
    std::vector<std::string> lines_ids;
    // Vector of lines inliers ids
    std::vector<std::string> lines_inliers_ids;

    for (int clu = 0; clu < (int) objects_clusters_clouds.size(); clu++)
    {

      int line_fit = 0;
      bool valid_line = true;
      bool stop_line_fitting = false;

      // Working cluster cloud which represents an object
      pcl::PointCloud<PointT>::Ptr working_cluster_cloud (new pcl::PointCloud<PointT> ());
      // Update the working cluster cloud 
      *working_cluster_cloud = *objects_clusters_clouds.at (clu);

/*

      /// FIT LINE MODELS ONLY IN THE XY INFO OF THE CLOUD ///
      /// EXPERIMENTAL, BTW ///

      pcl::PointCloud<PointT>::Ptr flattened_working_cluster_cloud (new pcl::PointCloud<PointT> ());
      *flattened_working_cluster_cloud = *objects_clusters_clouds.at (clu);

      for (unsigned int idx = 0; idx < flattened_working_cluster_cloud->points.size (); idx++)
        flattened_working_cluster_cloud->points.at (idx).z = 0.0;

      /// FIT LINE MODELS ONLY IN THE XY INFO OF THE CLOUD ///
      /// EXPERIMENTAL, BTW ///

*/

      do
      {

        /// ATTENTION ! BIG SNEAKY BUG FOUND ///
        /// ALSO POSSIBLE CONTAMINATION IN THE HOUGH SEGMENTATION WITH CIRCLES CODE ///
        /// OR IT COULD BE THE CASE ONLY FOR LINES ///
        valid_line = true;

        // Print current iteration number
        ROS_INFO ("AT ITERATION = %d AT GROUP = %d AT MODEL = %d", ite, clu, line_fit);
//        ROS_INFO ("AT ITERATION = %d AT GROUP = %d AT MODEL = %d AT %3.0g [s]", ite, clu, line_fit, tt.toc ());

        // Coefficients of line model
        pcl::ModelCoefficients line_coefficients;
        // Inliers of line model
        pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());

        // --------------------- //
        // Start fitting process //
        // --------------------- //

        // Create the segmentation object
        pcl::SACSegmentation<PointT> segmentation_of_line;
        // Optimize coefficients
        segmentation_of_line.setOptimizeCoefficients (false);
        // Set type of method
        segmentation_of_line.setMethodType (pcl::SAC_RANSAC);
        // Set type of model
        segmentation_of_line.setModelType (pcl::SACMODEL_LINE);
        // Set threshold of model
        segmentation_of_line.setDistanceThreshold (line_threshold);
        // Set number of maximum iterations
        segmentation_of_line.setMaxIterations (maximum_line_iterations);
        // Give as input the filtered point cloud
        segmentation_of_line.setInputCloud (working_cluster_cloud);

        // Call the segmenting method
        segmentation_of_line.segment (*line_inliers, line_coefficients);

/*

            // Create ID for line model
            std::stringstream line_id_original;
            line_id_original << "LINE_" << ros::Time::now();

            // Add line model to point cloud data
            line_viewer.addLine (line_coefficients, line_id_original.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

            // Remove line afterwards
            line_viewer.removeShape (line_id_original.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

*/

        // Adjust inliers of line model 
        adjustLineInliers (working_cluster_cloud, line_inliers, line_coefficients, line_threshold);

/*
            // Create ID for line model
            std::stringstream line_id_adjust_inliers;
            line_id_adjust_inliers << "LINE_" << ros::Time::now();

            // Add line model to point cloud data
            line_viewer.addLine (line_coefficients, line_id_adjust_inliers.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

            // Remove line afterwards
            line_viewer.removeShape (line_id_adjust_inliers.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

*/

        // ------------------------ //
        // Start extraction process //
        // ------------------------ //

        // Point cloud of line inliers
        pcl::PointCloud<PointT>::Ptr line_inliers_cloud (new pcl::PointCloud<PointT> ());

        // Extract the circular inliers 
        pcl::ExtractIndices<PointT> extraction_of_line;
        // Set which indices to extract
        extraction_of_line.setIndices (line_inliers);
        // Set point cloud from where to extract
        extraction_of_line.setInputCloud (working_cluster_cloud);

        // Return the points which represent the inliers
        extraction_of_line.setNegative (false);
        // Call the extraction function
        extraction_of_line.filter (*line_inliers_cloud);



        // Adjust the coefficients of the line model
        adjustLineCoefficients (line_inliers_cloud, line_coefficients);

/*

            // Create ID for line model
            std::stringstream line_id_adjust_coeffs;
            line_id_adjust_coeffs << "LINE_" << ros::Time::now();

            // Add line model to point cloud data
            line_viewer.addLine (line_coefficients, line_id_adjust_coeffs.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

            // Remove line afterwards
            line_viewer.removeShape (line_id_adjust_coeffs.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

*/

        // WARNING //

        // Needed for the bug in pcl extract indices
        int the_actual_number_of_points_from_the_fitted_line = (int) line_inliers_cloud->points.size ();



        // START W/ THE CLUSTERING FEATURE //

        if ( clustering_feature )
        {
          if ( (int) line_inliers->indices.size() < minimum_line_inliers )
          {
            ROS_ERROR ("  [MINIMUM LINE INLIERS] Reject line model !");

            // The current line model will be rejected
            valid_line = false;
          }
          else
          {
            if ( line_feature_step )
            {
              std::stringstream before_clustering_id;
              before_clustering_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*line_inliers_cloud, before_clustering_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_clustering_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (before_clustering_id.str());
              line_viewer.spin ();
            }

            // Vector of clusters from inliers
            std::vector<pcl::PointIndices> line_clusters;
            // Build kd-tree structure for clusters
            pcl::KdTreeFLANN<PointT>::Ptr line_clusters_tree (new pcl::KdTreeFLANN<PointT> ());

            // Instantiate cluster extraction object
            pcl::EuclideanClusterExtraction<PointT> clustering_of_line;
            // Set as input the cloud of line inliers
            clustering_of_line.setInputCloud (line_inliers_cloud);
            // Radius of the connnectivity threshold
            clustering_of_line.setClusterTolerance (line_clustering_tolerance);
            // Minimum number of points of any cluster
            clustering_of_line.setMinClusterSize (minimum_size_of_line_cluster);
            // Provide pointer to the search method
            clustering_of_line.setSearchMethod (line_clusters_tree);

            // Call the extraction function
            clustering_of_line.extract (line_clusters);

            if ( verbose )
            {
              ROS_INFO ("  [CLUSTERING FEATURE] Model has %d inliers clusters where", (int) line_clusters.size());
              for (int c = 0; c < (int) line_clusters.size(); c++)
                ROS_INFO ("  [CLUSTERING FEATURE]   Cluster %d has %d points", c, (int) line_clusters.at (c).indices.size());
            }

            pcl::PointIndices::Ptr clustering_line_inliers (new pcl::PointIndices ());

            // Leave only the biggest cluster
            if ( line_clusters.size() > 0 )
            {
              for ( int idx = 0; idx < (int) line_clusters.at (0).indices.size(); idx++ )
              {
                int inl = line_clusters.at (0).indices.at (idx);
                clustering_line_inliers->indices.push_back (line_inliers->indices.at (inl));
              }
            }
            else
            {
              ROS_ERROR ("  [CLUSTERING FEATURE] Reject line model !");

              // The current line model will be rejected
              valid_line = false;
            }

            // ------------------------ //
            // Start extraction process //
            // ------------------------ //

            pcl::PointCloud<PointT>::Ptr clustering_line_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers 
            pcl::ExtractIndices<PointT> clustering_extraction_of_line;
            // Set which indices to extract
            clustering_extraction_of_line.setIndices (clustering_line_inliers);
            // Set point cloud from where to extract
            clustering_extraction_of_line.setInputCloud (working_cluster_cloud);

            // Return the points which represent the inliers
            clustering_extraction_of_line.setNegative (false);
            // Call the extraction function
            clustering_extraction_of_line.filter (*clustering_line_inliers_cloud);

            if ( line_feature_step )
            {
              std::stringstream after_clustering_id;
              after_clustering_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*clustering_line_inliers_cloud, after_clustering_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_clustering_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (after_clustering_id.str());
              line_viewer.spin ();
            }

            // Update the line inliers
            *line_inliers = *clustering_line_inliers;
            // Update the line inliers cloud
            *line_inliers_cloud = *clustering_line_inliers_cloud;
          }
        }









        // ------------------------- //
        // Update extraction process //
        // ------------------------- //

        // Set which indices to extract
        extraction_of_line.setIndices (line_inliers);
        // Set point cloud from where to extract
        extraction_of_line.setInputCloud (working_cluster_cloud);

        // Return the points which represent the inliers
        extraction_of_line.setNegative (false);
        // Call the extraction function
        extraction_of_line.filter (*line_inliers_cloud);



        // Adjust the coefficients of the line model
        adjustLineCoefficients (line_inliers_cloud, line_coefficients);


/*

           // Create ID for line model
            std::stringstream line_id_after_clustering;
            line_id_after_clustering << "LINE_" << ros::Time::now();

            // Add line model to point cloud data
            line_viewer.addLine (line_coefficients, line_id_after_clustering.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

            // Remove line afterwards
            line_viewer.removeShape (line_id_after_clustering.str ());

            // And wait until Q key is pressed
            line_viewer.spin ();

*/

        // START W/ THE CURVATURE FEATURE //

        if ( curvature_feature )
        {
          if ( (int) line_inliers->indices.size() < minimum_line_inliers )
          {
            ROS_ERROR ("  [MINIMUM LINE INLIERS] Reject line model !");

            // The current line model will be rejected
            valid_line = false;
          }
          else
          {
            if ( line_feature_step )
            {
              std::stringstream before_curvature_id;
              before_curvature_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*line_inliers_cloud, before_curvature_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_curvature_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (before_curvature_id.str());
              line_viewer.spin ();
            }

            pcl::PointIndices::Ptr curvature_line_inliers (new pcl::PointIndices ());

            for (int inl = 0; inl < (int) line_inliers->indices.size(); inl++)
            {
              int idx = line_inliers->indices.at (inl);

              double curvature = working_cluster_cloud->points.at (idx).curvature;

              if ( curvature_threshold < curvature )
              {
                // Save the right indices of points
                curvature_line_inliers->indices.push_back (idx);
              }
            }

            // ------------------------ //
            // Start extraction process //
            // ------------------------ //

            pcl::PointCloud<PointT>::Ptr curvature_line_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers 
            pcl::ExtractIndices<PointT> curvature_extraction_from_line;
            // Set which indices to extract
            curvature_extraction_from_line.setIndices (curvature_line_inliers);
            // Set point cloud from where to extract
            curvature_extraction_from_line.setInputCloud (working_cluster_cloud);

            // Return the points which represent the inliers
            curvature_extraction_from_line.setNegative (false);
            // Call the extraction function
            curvature_extraction_from_line.filter (*curvature_line_inliers_cloud);

            if ( line_feature_step )
            {
              std::stringstream after_curvature_id;
              after_curvature_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*curvature_line_inliers_cloud, after_curvature_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_curvature_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (after_curvature_id.str());
              line_viewer.spin ();
            }

            // Update the inliers of line
            *line_inliers = *curvature_line_inliers;
            // Update the points of inliers
            *line_inliers_cloud = *curvature_line_inliers_cloud;
          }
        }



        // START W/ THE RSD FEATURE // 

        if ( rsd_feature )
        {
          if ( (int) line_inliers->indices.size() < minimum_line_inliers )
          {
            ROS_ERROR ("  [MINIMUM LINE INLIERS] Reject line model !");

            // The current line model will be rejected
            valid_line = false;
          }
          else
          {
            if ( line_feature_step )
            {
              std::stringstream before_rsd_id;
              before_rsd_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*line_inliers_cloud, before_rsd_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_rsd_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (before_rsd_id.str());
              line_viewer.spin ();
            }

            pcl::PointIndices::Ptr rsd_line_inliers (new pcl::PointIndices ());

            double rc = line_coefficients.values [2];

            for (int inl = 0; inl < (int) line_inliers->indices.size(); inl++)
            {
              int idx = line_inliers->indices.at (inl);

              // ATTENTION // 

              // The intensity values are actually the r_min values 
              //double r_min = working_cluster_cloud->points.at (idx).intensity; 
              double rp = working_cluster_cloud->points.at (idx).intensity;

              //if ( (low_r_min < r_min) && (r_min < high_r_min) ) 
              if ( fabs (rc - rp) < radius_threshold )
              {
                // Save the right indices of points
                rsd_line_inliers->indices.push_back (idx);
              }
            }

            // ---------------------------- //
            // Start the extraction process //
            // ---------------------------- //

            pcl::PointCloud<PointT>::Ptr rsd_line_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers 
            pcl::ExtractIndices<PointT> rsd_extraction_of_line;
            // Set which indices to extract
            rsd_extraction_of_line.setIndices (rsd_line_inliers);
            // Set point cloud from where to extract
            //rsd_extraction_of_line.setInputCloud (working_cluster_cloud);
            rsd_extraction_of_line.setInputCloud (working_cluster_cloud);

            // Return the points which represent the inliers
            rsd_extraction_of_line.setNegative (false);
            // Call the extraction function
            rsd_extraction_of_line.filter (*rsd_line_inliers_cloud);

            if ( line_feature_step )
            {
              std::stringstream after_rsd_id;
              after_rsd_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*rsd_line_inliers_cloud, after_rsd_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_rsd_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (after_rsd_id.str());
              line_viewer.spin ();
            }

            // Update the inliers of line
            *line_inliers = *rsd_line_inliers;
            // Update the points of inliers
            *line_inliers_cloud = *rsd_line_inliers_cloud;
          }
        }



        // START W/ THE NORMALS FEATURE //

        if ( normals_feature )
        {
          if ( (int) line_inliers->indices.size() < minimum_line_inliers )
          {
            ROS_ERROR ("  [MINIMUM LINE INLIERS] Reject line model !");

            // The current line model will be rejected
            valid_line = false;
          }
          else
          {
            if ( line_feature_step )
            {
              std::stringstream before_normals_id;
              before_normals_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*line_inliers_cloud, before_normals_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, before_normals_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (before_normals_id.str());
              line_viewer.spin ();
            }

            ROS_INFO ("  [NORMALS FEATURE] Model has now %d inliers", (int) line_inliers->indices.size());

            pcl::PointIndices::Ptr normals_line_inliers (new pcl::PointIndices ());

            //float c[2];
            //c[0] = line_coefficients.values.at (0);
            //c[1] = line_coefficients.values.at (1);

            for (int inl = 0; inl < (int) line_inliers->indices.size(); inl++)
            {
              int idx = line_inliers->indices.at (inl);

              //float p[2];
              //p[0] = working_cluster_cloud->points.at (idx).x;
              //p[1] = working_cluster_cloud->points.at (idx).y;
              //
              //float c2p[2];
              //c2p[0] = p[0] - c[0];
              //c2p[1] = p[1] - c[1];
              //
              //float lc2p = sqrt (c2p[0]*c2p[0] + c2p[1]*c2p[1]);
              //c2p[0] = c2p[0] / lc2p;
              //c2p[1] = c2p[1] / lc2p;

              float c2p[2];
              c2p[0] = line_coefficients.values.at (3);
              c2p[1] = line_coefficients.values.at (4);

              float np[2];
              np[0] = working_cluster_cloud->points.at (idx).normal_x;
              np[1] = working_cluster_cloud->points.at (idx).normal_y;

              float lnp = sqrt (np[0]*np[0] + np[1]*np[1]);
              np[0] = np[0] / lnp;
              np[1] = np[1] / lnp;

              float dot = c2p[0]*np[0] + c2p[1]*np[1];
              float ang = acos (dot) * 180.0 / M_PI;

              //cerr <<  angle_threshold << endl ;

              if ( ((90.0 - angle_threshold) < ang) && (ang < (90.0 + angle_threshold)) )
              {
                //cerr << ang << " ! " << inl << " -> " << idx << endl;
                //line_viewer.spin ();
 
                // Save the right indices of points
                normals_line_inliers->indices.push_back (idx);
              }
            }

            ROS_INFO ("  [NORMALS FEATURE] Model has %d inliers left", (int) normals_line_inliers->indices.size());

            if ( normals_line_inliers->indices.size() == 0 )
            {
              ROS_ERROR ("  [NORMALS FEATURE] Reject line model !");

              // The current line model will be rejected
              valid_line = false;
            }

            // ---------------------------- //
            // Start the extraction process //
            // ---------------------------- //

            pcl::PointCloud<PointT>::Ptr normals_line_inliers_cloud (new pcl::PointCloud<PointT> ());

            // Extract the circular inliers 
            pcl::ExtractIndices<PointT> normals_extraction_of_line;
            // Set which indices to extract
            normals_extraction_of_line.setIndices (normals_line_inliers);
            // Set point cloud from where to extract
            normals_extraction_of_line.setInputCloud (working_cluster_cloud);

            // Return the points which represent the inliers
            normals_extraction_of_line.setNegative (false);
            // Call the extraction function
            normals_extraction_of_line.filter (*normals_line_inliers_cloud);

            if ( line_feature_step )
            {
              std::stringstream after_normals_id;
              after_normals_id << "LINE_INLIERS_" << ros::Time::now();
              line_viewer.addPointCloud (*normals_line_inliers_cloud, after_normals_id.str ());
              line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, after_normals_id.str ()); 
              line_viewer.spin ();
              line_viewer.removePointCloud (after_normals_id.str());
              line_viewer.spin ();
            }

            // Update the inliers of line
            *line_inliers = *normals_line_inliers;
            // Update the points of inliers
            *line_inliers_cloud = *normals_line_inliers_cloud;
          }
        }




/*

        // ------------------------- //
        // Update extraction process //
        // ------------------------- //

        // Set which indices to extract
        extraction_of_line.setIndices (line_inliers);
        // Set point cloud from where to extract
        extraction_of_line.setInputCloud (working_cluster_cloud);

        // Return the points which represent the inliers
        extraction_of_line.setNegative (false);
        // Call the extraction function
        extraction_of_line.filter (*line_inliers_cloud);



        // Adjust the coefficients of the line model
        adjustLineCoefficients (line_inliers_cloud, line_coefficients);

*/








        // START W/ THE PERCENTAGE FEATURE //
 
        // Percentage of fitted inliers 
        double the_percentage_of_the_remaining_line_inliers = 0;

        if ( percentage_feature )
        {
          if ( (int) line_inliers->indices.size() < minimum_line_inliers )
          {
            ROS_ERROR ("  [MINIMUM LINE INLIERS] Reject line model !");

            // The current line model will be rejected
            valid_line = false;
          }
          else
          {
            // Percentage of fitted inleirs 
            the_percentage_of_the_remaining_line_inliers = round ((double) line_inliers_cloud->points.size() / (double) the_actual_number_of_points_from_the_fitted_line * 100);

            if ( the_percentage_of_the_remaining_line_inliers < line_percentage )
            {
              // The current line model will be rejected
              valid_line = false;
            }
            else
            {
              // The current line model will be accepted
              valid_line = true;
            }
          }
        }



        // WARNING //
       
        // Bug in pcl extract indices class, or maybe in pcl filter class 
        if ( (int) working_cluster_cloud->points.size () == the_actual_number_of_points_from_the_fitted_line )
        {
          // Clear manually the working cluster cloud
          working_cluster_cloud->points.clear ();
        }
        else
        {
          // Return the remaining points of inliers
          extraction_of_line.setNegative (true);
          // Call the extraction function
          extraction_of_line.filter (*working_cluster_cloud);
        }



        if ( verbose )
        {
          ROS_INFO ("  The actual number of points from the fitted line is %d !", the_actual_number_of_points_from_the_fitted_line);
          ROS_INFO ("  Line has %d inliers left", (int) line_inliers_cloud->points.size());
          ROS_INFO ("  %d points remain after extraction", (int) working_cluster_cloud->points.size ());
        }



        // First point of line
        double P1[2];
        P1[0] = line_coefficients.values [0];
        P1[1] = line_coefficients.values [1];

        // Second point of line
        double P2[2];
        P2[0] = line_coefficients.values [3] + line_coefficients.values [0];
        P2[1] = line_coefficients.values [4] + line_coefficients.values [1];

        // FINAL CHECK BEFORE ACCEPTING/REJECTING MODEL //
        // NOT SURE IF REALLY NECESSARY, BUT FOR SURE IT IS BETTER TO BE SAFE THAN SORRY //
        if ( (int) line_inliers->indices.size() < minimum_line_inliers )
        {
          ROS_ERROR ("  [MINIMUM LINE INLIERS] Reject line model !");

          // The current line model will be rejected
          valid_line = false;
        }


        if ( !valid_line )
        {
          ROS_ERROR ("  REJECTED ! %3.0f [%] ! Line [%2d] has %3d inliers with P1 = [%6.3f,%6.3f] and P2 = [%6.3f,%6.3f] found in maximum %d iterations",
              the_percentage_of_the_remaining_line_inliers, line_fit, (int) line_inliers->indices.size (), P1[0], P1[1], P2[0], P2[1], maximum_line_iterations);

          /*
          // No need for fitting lines anymore
          stop_line_fitting = true;
          */
        }
        else
        {
          ROS_INFO ("  ACCEPTED ! %3.0f [%] ! Line [%2d] has %3d inliers with P1 = [%6.3f,%6.3f] and P2 = [%6.3f,%6.3f] found in maximum %d iterations",
              the_percentage_of_the_remaining_line_inliers, line_fit, (int) line_inliers->indices.size (), P1[0], P1[1], P2[0], P2[1], maximum_line_iterations);

          // ----------------------------------- //
          // Build the parameter space for lines //
          // ----------------------------------- //
          

          // First point of line
          double x1 = line_coefficients.values [0];
          double y1 = line_coefficients.values [1];

          // Second point of line
          double x2 = line_coefficients.values [3] + line_coefficients.values [0];
          double y2 = line_coefficients.values [4] + line_coefficients.values [1];

/*

          // Distances to the origin
          double d1 = sqrt ( (x1 - 0)*(x1 - 0) + (y1 - 0)*(y1 - 0) );
          double d2 = sqrt ( (x2 - 0)*(x2 - 0) + (y2 - 0)*(y2 - 0) );

          // Vector from the origin to the point
          double o2p[2];

          // Which end of the segment is closer to the origin
          if ( d1 < d2 )
          {
            o2p[0] = x1 - 0;
            o2p[1] = y1 - 0;
          }
          else
          {
            o2p[0] = x2 - 0;
            o2p[1] = y2 - 0;
          }

          // The radius parameter of the polar coordinates
          double radius = sqrt ( o2p[0]*o2p[0] + o2p[1]*o2p[1] );

          // Normalize the vector
          float lo2p = sqrt (o2p[0]*o2p[0] + o2p[1]*o2p[1]);
          o2p[0] = o2p[0] / lo2p;
          o2p[1] = o2p[1] / lo2p;

          // Unit vector of the X axis
          double o2x[2];
          o2x[0] = 1;
          o2x[1] = 0;

          // The angle parameter of the polar coordinates
          float dot = o2p[0]*o2x[0] + o2p[1]*o2x[1];
          float theta = acos (dot);

*/

/*

          cerr << setprecision (3) << " x1 = " << x1 << " y1 = " << y1 << endl ;
          cerr << setprecision (3) << " x2 = " << x2 << " y2 = " << y2 << endl ;
          cerr << setprecision (3) << "  x = " << o2p[0]*radius << "  y = " << o2p[1]*radius << endl ;

          cerr << endl ;

          cerr << "  acos " << theta * 180.0 / M_PI << endl ;
          cerr << "  atan " << atan (o2p[1] / o2p[0]) * 180.0 / M_PI << endl ;
          cerr << " atan2 " << atan2 (o2p[1], o2p[0]) * 180.0 / M_PI << endl ;

*/

          // A vote consists of polar coordinates
          pcl::PointNormal line_vote;
//          line_vote.x = (P1[0] + P2[0]) / 2;
//          line_vote.y = (P1[1] + P2[1]) / 2;
          line_vote.x = (x1 + x2) / 2;
          line_vote.y = (y1 + y2) / 2;
          line_vote.z = sqrt ( _sqr (P2[0] - P1[0]) + _sqr (P2[1] - P1[1]) );
          //line_vote.z = 0.0;

          line_vote.normal_x  = x1;
          line_vote.normal_y  = y1;
          line_vote.normal_z  = x2;
          line_vote.curvature = y2;

          // Cast one vot for the current line
          line_parameters_cloud->points.push_back (line_vote);

/*

             pcl::Histogram<6> data_of_model;

             data_of_model.histogram[0] = x1;
             data_of_model.histogram[1] = y1;
             data_of_model.histogram[2] = x2;
             data_of_model.histogram[3] = y2;
             data_of_model.histogram[4] = x2 - x1;
             data_of_model.histogram[5] = y2 - y1;


          // Cast one vot for the current line, you know what !
          line_parameters_cloud_histogram->points.push_back (data_of_model);

*/

          // --------------------------- //
          // Start visualization process //
          // --------------------------- //

          if ( line_step )
          {
            // Create ID for line model
            std::stringstream line_id;
            line_id << "LINE_" << ros::Time::now();

            // Create ID for line inliers
            std::stringstream line_inliers_id;
            line_inliers_id << "LINE_INLIERS_" << ros::Time::now();
 
            // Add line model to point cloud data
            line_viewer.addLine (line_coefficients, line_id.str ());

            // Add the point cloud data
            line_viewer.addPointCloud (*line_inliers_cloud, line_inliers_id.str ());

            // Set the size of points for cloud data
            line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size, line_inliers_id.str ()); 

            // And wait until Q key is pressed
            line_viewer.spin ();

            // Save line ids for cleaning the viewer afterwards
            lines_ids.push_back (line_id.str());
            // Save also line inliers ids
            lines_inliers_ids.push_back (line_inliers_id.str());
          }

          // Fit only one model for each cluster in every iteration
          // No need for fitting lines anymore
          //          stop_line_fitting = true;
        }

        // number of fitted lines
        line_fit++;

        // --------------------------------------------------- //
        // Check for continuing with the fitting of 2D lines //
        // --------------------------------------------------- //

        // Print the number of points left for model fitting
        if ( (int) working_cluster_cloud->points.size () < minimum_line_inliers )
          ROS_WARN ("    %d < %d | Stop !", (int) working_cluster_cloud->points.size (), minimum_line_inliers);
        else
          if ( (int) working_cluster_cloud->points.size () > minimum_line_inliers )
            ROS_WARN ("    %d > %d | Continue... ", (int) working_cluster_cloud->points.size (), minimum_line_inliers);
          else
            ROS_WARN ("    %d = %d | Continue... ", (int) working_cluster_cloud->points.size (), minimum_line_inliers);

      } while ((int) working_cluster_cloud->points.size () > minimum_line_inliers && stop_line_fitting == false);
    }

    // ---------------------- //
    // Start cleaning process //
    // ---------------------- //

    if ( line_step )
    {
      for (int id = 0; id < (int) lines_ids.size(); id++)
      {
        // Remove line from the viewer
        line_viewer.removeShape (lines_ids[id]);
      }

      for (int id = 0; id < (int) lines_inliers_ids.size(); id++)
      {
        // Remove line from the viewer
        line_viewer.removePointCloud (lines_inliers_ids[id]);
      }

      // And wait until Q key is pressed
      line_viewer.spin ();
    }
  }





/// JUST FOR SAVING TIME IF WORKING W/ THE SAME PARAMETERS SPACE OVER AND OVER AGAIN ///

/// NOT IMPLEMENTED YET ///


  // -------------------------------------------------------------- //
  // ------------------- Lines Parameters Space ------------------- //
  // -------------------------------------------------------------- //



  std::stringstream line_parameters_id;
  line_parameters_id << "LINE_PARAMETERS_" << ros::Time::now();
  line_viewer.addPointCloud (*line_parameters_cloud, line_parameters_id.str ());
  line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 10, line_parameters_id.str ()); 
  line_viewer.spin ();

  std::string line_parameters_filename = argv [pFileIndicesPCD [0]];
  line_parameters_filename.insert (fullstop, "-lines");
  pcl::io::savePCDFile (line_parameters_filename, *line_parameters_cloud);

  if ( verbose )
  {
    ROS_INFO ("The parameters space of line models has %d votes !", (int) line_parameters_cloud->points.size ());
  }

  std::vector<pcl::PointIndices> line_parameters_clusters;
  pcl::KdTreeFLANN<pcl::PointNormal>::Ptr line_parameters_clusters_tree (new pcl::KdTreeFLANN<pcl::PointNormal> ());

  pcl::EuclideanClusterExtraction<pcl::PointNormal> line_parameters_extraction_of_clusters;
  line_parameters_extraction_of_clusters.setInputCloud (line_parameters_cloud);
  line_parameters_extraction_of_clusters.setClusterTolerance (clustering_tolerance_of_line_parameters);
  line_parameters_extraction_of_clusters.setMinClusterSize (minimum_size_of_line_parameters_clusters);
  line_parameters_extraction_of_clusters.setSearchMethod (line_parameters_clusters_tree);
  line_parameters_extraction_of_clusters.extract (line_parameters_clusters);

  if ( verbose )
  {
    ROS_INFO ("The parameters space has also %d clusters", (int) line_parameters_clusters.size ());
    for (int clu = 0; clu < (int) line_parameters_clusters.size(); clu++)
      ROS_INFO ("  Cluster %d has %d points", clu, (int) line_parameters_clusters.at (clu).indices.size());
  }

  std::vector<pcl::PointIndices::Ptr> line_parameters_clusters_indices;
  std::vector<pcl::PointCloud<pcl::PointNormal>::Ptr> line_parameters_clusters_clouds;

  for (int clu = 0; clu < (int) line_parameters_clusters.size(); clu++)
  {
    pcl::PointIndices::Ptr  cluster_indices (new pcl::PointIndices (line_parameters_clusters.at (clu)));
    pcl::PointCloud<pcl::PointNormal>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointNormal> ());

    pcl::ExtractIndices<pcl::PointNormal> line_parameters_extraction_of_indices;
    line_parameters_extraction_of_indices.setInputCloud (line_parameters_cloud);
    line_parameters_extraction_of_indices.setIndices (cluster_indices);
    line_parameters_extraction_of_indices.setNegative (false);
    line_parameters_extraction_of_indices.filter (*cluster_cloud);

    line_parameters_clusters_indices.push_back (cluster_indices);
    line_parameters_clusters_clouds.push_back (cluster_cloud);
  }

  std::vector<std::string> line_parameters_clusters_ids;

  for (int clu = 0; clu < (int) line_parameters_clusters.size(); clu++)
  {
    std::stringstream cluster_id;
    cluster_id << "CIRLCE_PARAMETERS_CLUSTER_" << ros::Time::now();
    line_viewer.addPointCloud (*line_parameters_clusters_clouds.at (clu), cluster_id.str());
    line_viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, 20, cluster_id.str()); 
    line_viewer.spin ();













    float sxm = 0.0;
    float sym = 0.0;
    float  sl = 0.0;

    float sx1 = 0.0;
    float sy1 = 0.0;
    float sx2 = 0.0;
    float sy2 = 0.0;

    int votes = line_parameters_clusters_clouds.at (clu)->points.size();

    for (int vot = 0; vot < votes; vot++)
    {
      float xm = line_parameters_clusters_clouds.at (clu)->points.at (vot).x;
      float ym = line_parameters_clusters_clouds.at (clu)->points.at (vot).y;
      float  l = line_parameters_clusters_clouds.at (clu)->points.at (vot).z;

      float x1 = line_parameters_clusters_clouds.at (clu)->points.at (vot).normal_x;
      float y1 = line_parameters_clusters_clouds.at (clu)->points.at (vot).normal_y;
      float x2 = line_parameters_clusters_clouds.at (clu)->points.at (vot).normal_z;
      float y2 = line_parameters_clusters_clouds.at (clu)->points.at (vot).curvature;

      sxm = sxm + xm;
      sym = sym + ym;
       sl =  sl +  l;

      sx1 = sx1 + x1;
      sy1 = sy1 + y1;
      sx2 = sx2 + x2;
      sy2 = sy2 + y2;
    }

    float mxm = sxm / votes;
    float mym = sym / votes;
    float  ml =  sl / votes;

    float mx1 = sx1 / votes;
    float my1 = sy1 / votes;
    float mx2 = sx2 / votes;
    float my2 = sy2 / votes;







/*



    pcl::ModelCoefficients M2P1;
    M2P1.values.push_back (mxm);
    M2P1.values.push_back (mym);
    M2P1.values.push_back (0.0);
    M2P1.values.push_back (mx1 - mxm);
    M2P1.values.push_back (my1 - mym);
    M2P1.values.push_back (0.0);

    std::stringstream M2P1_id;
    M2P1_id << "M2P1_LINE_" << ros::Time::now();

    line_viewer.addLine (M2P1, M2P1_id.str ());





    pcl::ModelCoefficients M2P2;
    M2P2.values.push_back (mxm);
    M2P2.values.push_back (mym);
    M2P2.values.push_back (0.0);
    M2P2.values.push_back (mx2 - mxm);
    M2P2.values.push_back (my2 - mym);
    M2P2.values.push_back (0.0);

    std::stringstream M2P2_id;
    M2P2_id << "M2P2_LINE_" << ros::Time::now();

    line_viewer.addLine (M2P2, M2P2_id.str ());





*/

/*

    pcl::ModelCoefficients P1P2;
    P1P2.values.push_back (mx1);
    P1P2.values.push_back (my1);
    P1P2.values.push_back (0.0);
    P1P2.values.push_back (mx2 - mx1);
    P1P2.values.push_back (my2 - my1);
    P1P2.values.push_back (0.0);

    std::stringstream P1P2_id;
    P1P2_id << "P1P2_LINE_" << ros::Time::now();

    line_viewer.addLine (P1P2, P1P2_id.str ());

*/




    double vec[2];
    vec[0] = mx2 - mx1;
    vec[1] = my2 - my1;

    double nor = sqrt ( _sqr (vec[0]) + _sqr (vec[1]) );
    vec[0] = vec[0] / nor;
    vec[1] = vec[1] / nor;

    double A[2];
    A[0] = mxm + vec[0] * ml / 2;
    A[1] = mym + vec[1] * ml / 2;

    double B[2];
    B[0] = mxm - vec[0] * ml / 2;
    B[1] = mym - vec[1] * ml / 2;

    pcl::ModelCoefficients AB;
    AB.values.push_back (A[0]);
    AB.values.push_back (A[1]);
    AB.values.push_back (0.0);
    AB.values.push_back (B[0] - A[0]);
    AB.values.push_back (B[1] - A[1]);
    AB.values.push_back (0.0);

    std::stringstream AB_id;
    AB_id << "AB_LINE_" << ros::Time::now();

    line_viewer.addLine (AB, AB_id.str ());




    cerr << ml << " meters ! " << endl;

    cerr <<  sqrt ( _sqr (B[0] - A[0]) + _sqr (B[1] - A[1]) ) << " meters ! " << endl;

    cerr <<  sqrt ( _sqr (mx2 - mx1) + _sqr (my2 - my1) ) << " meters ! " << endl;

    cerr << " Model for LPS cluster " << clu << " ! " << endl ;

    line_viewer.spin ();






















    line_parameters_clusters_ids.push_back (cluster_id.str());
  }

  cerr << " Finished w/ printing the clusters of parameters space ! " << endl ;

  line_viewer.spin ();
  line_viewer.spin ();
  line_viewer.spin ();

  for (int clu = 0; clu < (int) line_parameters_clusters.size(); clu++)
  {
    float sxm = 0.0;
    float sym = 0.0;
    float  sl = 0.0;

    float sx1 = 0.0;
    float sy1 = 0.0;
    float sx2 = 0.0;
    float sy2 = 0.0;

    int votes = line_parameters_clusters_clouds.at (clu)->points.size();

    for (int vot = 0; vot < votes; vot++)
    {
      float xm = line_parameters_clusters_clouds.at (clu)->points.at (vot).x;
      float ym = line_parameters_clusters_clouds.at (clu)->points.at (vot).y;
      float  l = line_parameters_clusters_clouds.at (clu)->points.at (vot).z;

      float x1 = line_parameters_clusters_clouds.at (clu)->points.at (vot).normal_x;
      float y1 = line_parameters_clusters_clouds.at (clu)->points.at (vot).normal_y;
      float x2 = line_parameters_clusters_clouds.at (clu)->points.at (vot).normal_z;
      float y2 = line_parameters_clusters_clouds.at (clu)->points.at (vot).curvature;

      sxm = sxm + xm;
      sym = sym + ym;
       sl =  sl +  l;

      sx1 = sx1 + x1;
      sy1 = sy1 + y1;
      sx2 = sx2 + x2;
      sy2 = sy2 + y2;
    }

    float mxm = sxm / votes;
    float mym = sym / votes;
    float  ml =  sl / votes;

    float mx1 = sx1 / votes;
    float my1 = sy1 / votes;
    float mx2 = sx2 / votes;
    float my2 = sy2 / votes;







/*



    pcl::ModelCoefficients M2P1;
    M2P1.values.push_back (mxm);
    M2P1.values.push_back (mym);
    M2P1.values.push_back (0.0);
    M2P1.values.push_back (mx1 - mxm);
    M2P1.values.push_back (my1 - mym);
    M2P1.values.push_back (0.0);

    std::stringstream M2P1_id;
    M2P1_id << "M2P1_LINE_" << ros::Time::now();

    line_viewer.addLine (M2P1, M2P1_id.str ());





    pcl::ModelCoefficients M2P2;
    M2P2.values.push_back (mxm);
    M2P2.values.push_back (mym);
    M2P2.values.push_back (0.0);
    M2P2.values.push_back (mx2 - mxm);
    M2P2.values.push_back (my2 - mym);
    M2P2.values.push_back (0.0);

    std::stringstream M2P2_id;
    M2P2_id << "M2P2_LINE_" << ros::Time::now();

    line_viewer.addLine (M2P2, M2P2_id.str ());





*/


    pcl::ModelCoefficients P1P2;
    P1P2.values.push_back (mx1);
    P1P2.values.push_back (my1);
    P1P2.values.push_back (0.0);
    P1P2.values.push_back (mx2 - mx1);
    P1P2.values.push_back (my2 - my1);
    P1P2.values.push_back (0.0);

    std::stringstream P1P2_id;
    P1P2_id << "P1P2_LINE_" << ros::Time::now();

    line_viewer.addLine (P1P2, P1P2_id.str ());




    cerr << ml << " meters ! " << endl;

    cerr <<  sqrt ( _sqr (mx2 - mx1) + _sqr (my2 - my1) ) << " meters ! " << endl;

    cerr << " Model for LPS cluster " << clu << " ! " << endl ;

    line_viewer.spin ();
    line_viewer.spin ();
    line_viewer.spin ();

  }



  exit (0);



  textfile << "\n" << std::flush;

  textfile.close();



  if ( verbose )
  {
    // Displaying the overall time
    ROS_WARN ("Finished in %5.3g [s] !", tt.toc ());
  }

  // And wait until Q key is pressed
  line_viewer.spin ();

  return (0);
}
