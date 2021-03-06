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

// ---------- Dependencies ---------- //

#include "pcl/features/boundary.h"

#include "pcl/console/parse.h"
#include "pcl/console/time.h"
#include "pcl/features/normal_3d.h"
#include "pcl/features/rsd.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/io/pcd_io.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/surface/mls.h"
#include "pcl/visualization/pcl_visualizer.h"

#include "pcl/features/impl/normal_3d.hpp"
#include "pcl/features/impl/rsd.hpp"
#include "pcl/filters/impl/extract_indices.hpp"
#include "pcl/segmentation/impl/extract_clusters.hpp"
#include "pcl/segmentation/impl/sac_segmentation.hpp"
#include "pcl/visualization/impl/pcl_visualizer.hpp"

#include "../include/ransac.hpp"
#include "../include/fetch_cuboid.hpp"
#include "../include/inside.hpp"



// ---------- Types ---------- //

typedef pcl::PointXYZ I; //
// typedef pcl::PointXYZRGB I; //

typedef pcl::Normal N;

typedef pcl::PointNormal PointT; //
// typedef pcl::PointXYZRGBNormal PointT; //

typedef pcl::Boundary B;

// ---------- Structures ---------- //

struct object
{
  int type;
  Eigen::Vector4f centroid;
  pcl::ModelCoefficients cylinder;
  std::vector<pcl::ModelCoefficients> cuboid;
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cloud;
};

// ---------- Variables ---------- //

// Method //
int vransac_iterations = 100;
double normal_search_radius = 0.020;
double smoothing_search_radius = 0.020;
double curvature_threshold = 0.010;
double rsd_search_radius = 0.020;
double rsd_plane_radius = 0.200;
double low_r_min = 0.020;
double high_r_min = 0.080;

// Planar //
double significant_plane_threshold = 0.0; /// [meters]
int minimum_inliers_of_significant_plane = 0; /// [points]
int maximum_iterations_of_significant_plane = 0; /// [iterations]
int minimum_size_of_significant_plane_cluster = 0; /// [points]
double clustering_tolerance_of_significant_plane_inliers = 0.0; /// [meters]

// Boundary //
double boundary_search_radius = 0.0;
double boundary_angle_threshold = 0.0;

// Faster
bool smoothed = 0;
bool normal = 0;
bool rsd = 0;
bool normal_refinement = 0;

// Fitting //
bool fitting_step = false;
double line_threshold = 0.010;
double circle_threshold = 0.010;
double minimum_line_length = 0.025;
double maximum_line_length = 0.250;
double minimum_circle_radius = 0.010;
double maximum_circle_radius = 0.100;
int maximum_line_iterations = 1000;
int maximum_circle_iterations = 1000;
int minimum_line_inliers = 100;
int minimum_circle_inliers = 100;

// Features //
bool curvature_feature_for_lines = true;
bool curvature_feature_for_circles = true;

bool clustering_feature_for_lines = true;
bool clustering_feature_for_circles = true;
double line_inliers_clustering_tolerance = 0.010;
double circle_inliers_clustering_tolerance = 0.010;
int minimum_size_of_line_inliers_clusters = 100;
int minimum_size_of_circle_inliers_clusters = 100;

bool normal_feature_for_lines = true;
bool normal_feature_for_circles = true;
double line_normals_angle_threshold = 2.5;
double circle_normals_angle_threshold = 10.0;

// Spaces //
bool space_step = false;
double clustering_tolerance_of_line_parameters_space = 0.001;
double clustering_tolerance_of_circle_parameters_space = 0.001;
int minimum_size_of_line_parameters_clusters = 10;
int minimum_size_of_circle_parameters_clusters = 10;
double tX, tY, tZ;
double rX, rY, rZ;
double vX, vY, vZ;

// Growing //
double growing_step = 0.010;
double growing_height = 0.010;
bool growing_visualization = false;
int mean_k_filter = 100;
int std_dev_filter = 10.0;

// Rest //
double r_clustering_tolerance = 0.010;
int minimum_size_of_r_clusters = 50;

// Visualization //
bool verbose = true;
bool step = false;
int size = 1;

bool denoising = false;
bool smoothing = false;

bool sign = false;

bool till_the_end = true;
bool classification = true;

int number_of_box = 1;
int number_of_flat = 1;

int number_of_tall = 1;
int number_of_medium = 1;
int number_of_short = 1;

double flat_value = 0.250;

bool deal_with_the_rest_of_the_points = true;
bool consider_height_from_table_plane = true;
bool shapes_from_table_plane = true;

double tall_value = 0.750;
double medium_value = 0.500;
double short_value = 0.250;

// Different //
double sat = 1.0;

// Centroids //
double threshold_between_centroids_of_cuboids = 0.0;
double threshold_between_centroids_of_cylinders = 0.0;

// New //
int too_many_planar_curvatures = 0;



// ---------- Macros ---------- //

#define _sqr(c) ((c)*(c))

#define M_PI 3.14159265358979323846



/////////////////////////////////////////////////////////////
/** \brief Print the usage instructions for the code at hand.
 * \param command The command line binary file.
*/
void printUsage (const char* command)
{
  pcl::console::print_info ("\nThe syntax for the vransac segmentation is:\n");
  pcl::console::print_info ("\n  %s [input].pcd [options]\n", command);
  pcl::console::print_info ("\nWhere options are the following for:\n");
  pcl::console::print_info ("\n[method]\n");
  pcl::console::print_info ("  -vransac_iterations D                                        = How many times to run the VRANSAC routine.\n");
  pcl::console::print_info ("  -normal_search_radius X                                      = Sphere radius for search of neighbor points.\n");
  pcl::console::print_info ("  -smoothing_search_radius X                                   = Sphere radius for smoothing the point cloud.\n");
  pcl::console::print_info ("  -curvature_threshold X                                       = Threshold between planar and circular patches of surfaces.\n");
  pcl::console::print_info ("  -rsd_search_radius X                                         = The search radius for RSD estimations.\n");
  pcl::console::print_info ("  -rsd_plane_radius X                                          = The maximum radius for RSD estimations.\n");
  pcl::console::print_info ("  -low_r_min X                                                 = Lowest value of minimum RSD radius.\n");
  pcl::console::print_info ("  -high_r_min X                                                = Highest value of minimum RSD radius.\n");
  pcl::console::print_info ("\n[fitting]\n");
  pcl::console::print_info ("  -fitting_step B                                              = Wait or not wait.\n");
  pcl::console::print_info ("  -line_threshold X                                            = Threshold for line inliers selection.\n");
  pcl::console::print_info ("  -circle_threshold X                                          = Threshold for circle inliers selection.\n");
  pcl::console::print_info ("  -maximum_line_iterations D                                   = Maximum number of iterations for lines.\n");
  pcl::console::print_info ("  -maximum_circle_iterations D                                 = Maximum number of iterations for circles.\n");
  pcl::console::print_info ("  -minimum_circle_radius X                                     = Minimum radius of circle.\n");
  pcl::console::print_info ("  -maximum_circle_radius X                                     = Maximum radius of circle.\n");
  pcl::console::print_info ("  -minimum_line_inliers D                                      = Minimum number of line inliers.\n");
  pcl::console::print_info ("  -minimum_circle_inliers D                                    = Minimum number of circle inliers.\n");
  pcl::console::print_info ("\n[features]\n");
  pcl::console::print_info ("  -curvature_feature_for_lines B                               = .\n");
  pcl::console::print_info ("  -curvature_feature_for_circles B                             = .\n");
  pcl::console::print_info ("\n");
  pcl::console::print_info ("  -clustering_feature_for_lines B                              = Clustering feature for lines or not.\n");
  pcl::console::print_info ("  -clustering_feature_for_circles B                            = Clustering feature for circles or not.\n");
  pcl::console::print_info ("  -line_inliers_clustering_tolerance X                         = Clustering tolerance of line inliers.\n");
  pcl::console::print_info ("  -circle_inliers_clustering_tolerance X                       = Clustering tolerance of circle inliers.\n");
  pcl::console::print_info ("  -minimum_size_of_line_inliers_clusters D                     = Minimum size of line inliers clusters.\n");
  pcl::console::print_info ("  -minimum_size_of_circle_inliers_clusters D                   = Minimum size of circle inliers clusters.\n");
  pcl::console::print_info ("\n");
  pcl::console::print_info ("  -normal_feature_for_lines B                                  = Normal feature for lines or not.\n");
  pcl::console::print_info ("  -normal_feature_for_circles B                                = Normal feature for circles or not.\n");
  pcl::console::print_info ("  -line_normals_angle_threshold X                              = Angle threshold of normals for lines.\n");
  pcl::console::print_info ("  -circle_normals_angle_threshold X                            = Angle threshold of normals for circles.\n");
  pcl::console::print_info ("\n[spaces]\n");
  pcl::console::print_info ("  -space_step B                                                = Wait or not wait.\n");
  pcl::console::print_info ("  -clustering_tolerance_of_line_parameters_space X             = .\n");
  pcl::console::print_info ("  -clustering_tolerance_of_circle_parameters_space X           = .\n");
  pcl::console::print_info ("  -minimum_size_of_line_parameters_clusters D                  = .\n");
  pcl::console::print_info ("  -minimum_size_of_circle_parameters_clusters D                = .\n");
  pcl::console::print_info ("  -viewpoint_translation X,X,X                                 = Set the viewpoint's translation in millimiters.\n");
  pcl::console::print_info ("  -viewpoint_rotation X,X,X                                    = Set the viewpoint's rotation in degrees.\n");
  pcl::console::print_info ("  -viewpoint X,X,X                                             = Set the viewpoint.\n");
  pcl::console::print_info ("\n[visualization]\n");
  pcl::console::print_info ("  -verbose B                                                   = Show all print messages.\n");
  pcl::console::print_info ("  -step B                                                      = Wait or not wait.\n");
  pcl::console::print_info ("  -size D                                                      = Size of points from cloud.\n");

  return;
}

/////////////////////////////////////////////////////
/** \brief Get the current timestamp as an unique key
 */
std::string getTimestamp ()
{
  time_t raw_time;
  struct tm *time_info;
  struct timeval fine_time;

  raw_time = time (NULL);
  time_info = localtime (&raw_time);
  gettimeofday (&fine_time, NULL);

  std::stringstream ss;
  ss << std::setw (4) << std::setfill ('0') << time_info->tm_year + 1900;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_mon + 1;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_mday;
  ss << ".";
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_hour;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_min;
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_sec;
  ss << ".";
  ss << std::setw (3) << std::setfill ('0') << fine_time.tv_usec / 1000;
  ss << ".";
  ss << std::setw (3) << std::setfill ('0') << fine_time.tv_usec - fine_time.tv_usec / 1000 * 1000;

  return ss.str ();
}

//////////////////////////////////////////////////////
/** \brief Get the current timestamp as an unique key,
 *         which can be read bye humans
 */
std::string getStampOfTime ()
{
  time_t raw_time;
  struct tm *time_info;
  struct timeval fine_time;

  raw_time = time (NULL);
  time_info = localtime (&raw_time);
  gettimeofday (&fine_time, NULL);

  std::stringstream ss;
  ss << std::setw (4) << std::setfill ('0') << time_info->tm_year + 1900;
  ss << "/";
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_mon + 1;
  ss << "/";
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_mday;
  ss << " ";
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_hour;
  ss << ":";
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_min;
  ss << ":";
  ss << std::setw (2) << std::setfill ('0') << time_info->tm_sec;
  ss << " ";
  ss << std::setw (3) << std::setfill ('0') << fine_time.tv_usec / 1000;
  ss << "ms";
  ss << " ";
  ss << std::setw (3) << std::setfill ('0') << fine_time.tv_usec - fine_time.tv_usec / 1000 * 1000;
  ss << "us";
  ss << " ";

  return ss.str ();
}

/////////////////////////////////////////////////////////////////
/** \brief Computes the inliers of line with regards to xOy plane
 * \param cloud The point cloud data
 * \param inliers The inliers of line
 * \param coefficients The parameters of line
 */
void adjustLineInliers (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients, double threshold)
{
  // First point of line
  double P1[2];
  P1[0] = coefficients->values [0];
  P1[1] = coefficients->values [1];

  // Second point of line
  double P2[2];
  P2[0] = coefficients->values [3] + coefficients->values [0];
  P2[1] = coefficients->values [4] + coefficients->values [1];

  double x1 = P1[0];
  double y1 = P1[1];
  double x2 = P2[0];
  double y2 = P2[1];

  // Start from scratch
  inliers->indices.clear ();

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

////////////////////////////////////////////////////////////////////////
/** \brief Computes the coefficients of line with regards to its inliers
 * \param cloud The point cloud of the line
 * \param coefficients The parameters of the line
 */
void adjustLineCoefficients (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &cloud, pcl::ModelCoefficients::Ptr &coefficients)
{
  // Vector of line
  double line[2];
  line[0] = coefficients->values [3];
  line[1] = coefficients->values [4];

  // Normalize the vector of line
  double norm = sqrt (line[0]*line[0] + line[1]*line[1]);
  line[0] = line[0] / norm;
  line[1] = line[1] / norm;

  // First point of line
  double P1[2];
  P1[0] = coefficients->values [0];
  P1[1] = coefficients->values [1];

  // Second point of line
  double P2[2];
  P2[0] = coefficients->values [3] + coefficients->values [0];
  P2[1] = coefficients->values [4] + coefficients->values [1];

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

  // New Model's Coefficients
  P2[0] = P1[0] + line[0]*maximum_lengthwise;
  P2[1] = P1[1] + line[1]*maximum_lengthwise;

  P1[0] = P1[0] + line[0]*minimum_lengthwise;
  P1[1] = P1[1] + line[1]*minimum_lengthwise;

  // Save the new coefficients
  coefficients->values [0] = P1[0];
  coefficients->values [1] = P1[1];
  coefficients->values [2] = 0.0;

  coefficients->values [3] = P2[0] - P1[0];
  coefficients->values [4] = P2[1] - P1[1];
  coefficients->values [5] = 0.0;

  return;
}

////////////////////////////////////////////////////////////////////////
void adjustLineModel (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients, double threshold, int maximum_iterations)
{
  inliers->indices.clear ();
  coefficients->values.clear ();

  int iterations = fitLine (cloud, inliers, coefficients, threshold, maximum_iterations);

  adjustLineCoefficients (cloud, coefficients);

  //pcl::SACSegmentation<pcl::PointXYZRGBNormalRSD> sacs;
  //sacs.setInputCloud (cloud);
  //sacs.setMethodType (pcl::SAC_RANSAC);
  //sacs.setOptimizeCoefficients (false);
  //
  //sacs.setModelType (pcl::SACMODEL_LINE);
  //sacs.setMaxIterations (maximum_iterations);
  //sacs.setDistanceThreshold (threshold);
  //sacs.segment (*inliers, *coefficients);

  return;
}

////////////////////////////////////////////////////////////////////////
void adjustCircleModel (pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &cloud, pcl::PointIndices::Ptr &inliers, pcl::ModelCoefficients::Ptr &coefficients, double threshold, double minimum_radius, double maximum_radius, int maximum_iterations)
{
  inliers->indices.clear ();
  coefficients->values.clear ();

  int iterations = fitCircle (cloud, inliers, coefficients, threshold, minimum_radius, maximum_radius, maximum_iterations);

  //pcl::SACSegmentation<pcl::PointXYZRGBNormalRSD> sacs;
  //sacs.setInputCloud (cloud);
  //sacs.setMethodType (pcl::SAC_RANSAC);
  //sacs.setOptimizeCoefficients (false);
  //
  //sacs.setModelType (pcl::SACMODEL_CIRCLE2D);
  //sacs.setMaxIterations (maximum_iterations);
  //sacs.setDistanceThreshold (threshold);
  //sacs.setRadiusLimits (minimum_radius, maximum_radius);
  //sacs.segment (*inliers, *coefficients);

  return;
}

////////////////////////////////////////////////////////////////////////
/** \brief
 * \param valid_line
 * \param working_cloud
 * \param line_cloud
 * \param line_inliers
 * \param line_coefficients
 *[Ma \param viewer
 * \param verbose
 * \param step
 */
void CurvatureFeatureForLines (bool &valid_line,
                               pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &working_cloud,
                               pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &line_cloud,
                               pcl::PointIndices::Ptr &line_inliers,
                               pcl::ModelCoefficients::Ptr &line_coefficients,
                               pcl::visualization::PCLVisualizer &viewer,
                               bool verbose = true,
                               bool step = false)
{
  //if ( step )
  //{
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 255, 255, 0);
  //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "INLIERS");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
  //viewer.addLine (*line_coefficients, 1.0, 1.0, 0.0, "LINE");
  //viewer.spin ();
  //viewer.removePointCloud ("INLIERS");
  //viewer.removeShape ("LINE");
  //viewer.spin ();
  //}

  if ( verbose ) pcl::console::print_info ("  [CURVATURE FEATURE FOR LINES] Line has now %d inliers\n", (int) line_inliers->indices.size());

  pcl::PointIndices::Ptr curvature_feature_inliers (new pcl::PointIndices ());

  for (int inl = 0; inl < (int) line_inliers->indices.size(); inl++)
  {
    int idx = line_inliers->indices.at (inl);
    double curvature = working_cloud->points.at (idx).curvature;

    if ( curvature_threshold > curvature )
    {
      // Save The Right Indices Of Points //
      curvature_feature_inliers->indices.push_back (idx);
    }
  }

  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr curvature_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> ei;
  ei.setInputCloud (working_cloud);
  ei.setIndices (curvature_feature_inliers);
  ei.setNegative (false);
  ei.filter (*curvature_feature_cloud);

  if ( verbose ) pcl::console::print_info ("  [CURVATURE FEATURE FOR LINES] Line has %d inliers left\n", (int) curvature_feature_inliers->indices.size());

  if ( curvature_feature_inliers->indices.size() == 0 )
  {
    valid_line = false;
    if ( verbose ) pcl::console::print_error ("  [CURVATURE FEATURE FOR LINES] Line Rejected !\n");
  }

  // Update Inliers And Cloud Of Line //
  *line_inliers = *curvature_feature_inliers;
  *line_cloud = *curvature_feature_cloud;

  // Adjust Coefficients Of Line Model //
  adjustLineCoefficients (line_cloud, line_coefficients);

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
    viewer.addLine (*line_coefficients, 1.0, 1.0, 0.0, "LINE");
    viewer.spin ();
    viewer.removePointCloud ("INLIERS");
    viewer.removeShape ("LINE");
    viewer.spin ();
  }

  return;
}

////////////////////////////////////////////////////////////////////////
/** \brief
 * \param valid_circle
 * \param working_cloud
 * \param circle_cloud
 * \param circle_inliers
 * \param circle_coefficients
 * \param viewer
 * \param verbose
 * \param step
 */
void CurvatureFeatureForCircles (bool &valid_circle,
                                 pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &working_cloud,
                                 pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &circle_cloud,
                                 pcl::PointIndices::Ptr &circle_inliers,
                                 pcl::ModelCoefficients::Ptr &circle_coefficients,
                                 pcl::visualization::PCLVisualizer &viewer,
                                 bool verbose = true,
                                 bool step = false)
{
  //if ( step )
  //{
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 255, 255, 0);
  //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "INLIERS");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
  //viewer.addCircle (*circle_coefficients, 1.0, 1.0, 0.0, "CIRCLE");
  //viewer.spin ();
  //viewer.removePointCloud ("INLIERS");
  //viewer.removeShape ("CIRCLE");
  //viewer.spin ();
  //}

  if ( verbose ) pcl::console::print_info ("  [CURVATURE FEATURE FOR CIRCLES] Circle has now %d inliers\n", (int) circle_inliers->indices.size());

  pcl::PointIndices::Ptr curvature_feature_inliers (new pcl::PointIndices ());

  for (int inl = 0; inl < (int) circle_inliers->indices.size(); inl++)
  {
    int idx = circle_inliers->indices.at (inl);
    double curvature = working_cloud->points.at (idx).curvature;

    if ( curvature_threshold < curvature )
    {
      // Save The Right Indices Of Points //
      curvature_feature_inliers->indices.push_back (idx);
    }
  }

  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr curvature_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> ei;
  ei.setInputCloud (working_cloud);
  ei.setIndices (curvature_feature_inliers);
  ei.setNegative (false);
  ei.filter (*curvature_feature_cloud);

  if ( verbose ) pcl::console::print_info ("  [CURVATURE FEATURE FOR CIRCLES] Circle has %d inliers left\n", (int) curvature_feature_inliers->indices.size());

  if ( curvature_feature_inliers->indices.size() == 0 )
  {
    valid_circle = false;
    if ( verbose ) pcl::console::print_error ("  [CURVATURE FEATURE FOR CIRCLES] Circle Rejected !\n");
  }

  // Update Inliers And Cloud Of Circle //
  *circle_inliers = *curvature_feature_inliers;
  *circle_cloud = *curvature_feature_cloud;

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
    viewer.addCircle (*circle_coefficients, 1.0, 1.0, 0.0, "CIRCLE");
    viewer.spin ();
    viewer.removePointCloud ("INLIERS");
    viewer.removeShape ("CIRCLE");
    viewer.spin ();
  }

  return;
}

////////////////////////////////////////////////////////////////////////
/** \brief
 * \param valid_line
 * \param working_cloud
 * \param line_cloud
 * \param line_inliers
 * \param line_coefficients
 * \param viewer
 * \param verbose
 * \param step
 */
void ClusteringFeatureForLines (bool &valid_line,
                                pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &working_cloud,
                                pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &line_cloud,
                                pcl::PointIndices::Ptr &line_inliers,
                                pcl::ModelCoefficients::Ptr &line_coefficients,
                                pcl::visualization::PCLVisualizer &viewer,
                                pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &adjust_using_curvature,
                                bool verbose = true,
                                bool step = false)
{
  //if ( step )
  //{
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 0, 255, 0);
  //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "INLIERS");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
  //viewer.addLine (*line_coefficients, 0.0, 1.0, 0.0, "LINE");
  //viewer.spin ();
  //viewer.removePointCloud ("INLIERS");
  //viewer.removeShape ("LINE");
  //viewer.spin ();
  //}

  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl::PointXYZRGBNormalRSD>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormalRSD> ());
  tree->setInputCloud (line_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormalRSD> ece;
  ece.setInputCloud (line_cloud);
  ece.setClusterTolerance (line_inliers_clustering_tolerance);
  ece.setMinClusterSize (minimum_size_of_line_inliers_clusters);
  ece.setSearchMethod (tree);
  ece.extract (clusters);



  // New // Begin // ------

  std::vector<pcl::PointIndices> curvature_clusters (clusters.size ());

  for (int clu = 0; clu < clusters.size (); clu++ )
  {
    for (int inl = 0; inl < clusters.at (clu).indices.size (); inl++ )
    {
      int idx = clusters.at (clu).indices.at (inl);
      double curvature = line_cloud->points.at (idx).curvature;

      //double curvature = line_cloud->points.at (inl).curvature;

      if ( curvature_threshold > curvature )
        curvature_clusters.at (clu).indices.push_back (idx);
    }
  }

  // New // End // ------



  int s;

  pcl::PointIndices::Ptr clustering_feature_inliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr curvature_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr clustering_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  if ( clusters.size() > 0 )
  {

  if ( verbose )
  {
    pcl::console::print_info ("  [CLUSTERING FEATURE FOR LINES] Line has %d inliers clusters where\n", (int) clusters.size());
    for (int c = 0; c < (int) clusters.size(); c++)
      pcl::console::print_info ("  [CLUSTERING FEATURE FOR LINES]   Cluster %d has %5d points with %5d planar curvatures.\n", c, (int) clusters.at (c).indices.size (), curvature_clusters. at (c).indices.size ());
  }



  // New // Begin // ------

  int maximum_size_of_curvature_clusters = INT_MIN;
  int index_of_maximum_size_of_curvature_clusters = -1;

  for (int clu = 0; clu < curvature_clusters.size (); clu++ )
  {
    if ( maximum_size_of_curvature_clusters < (int) curvature_clusters. at (clu).indices.size () )
    {
      maximum_size_of_curvature_clusters = curvature_clusters. at (clu).indices.size ();
      index_of_maximum_size_of_curvature_clusters = clu;
    }
  }

  s = index_of_maximum_size_of_curvature_clusters;

  cerr << endl ;
  cerr << "  found cluster " << s << " with " << clusters.at (s).indices.size () << " points and " << maximum_size_of_curvature_clusters << " curvatures " << endl;
  cerr << endl ;

  // New // End // ------


  // IF NO PLANAR CURVATURES, THEN ...
  if ( maximum_size_of_curvature_clusters == 0 ) valid_line = false;
  if ( maximum_size_of_curvature_clusters == 1 ) valid_line = false;
  //if ( valid_line == false ) viewer.spin ();

  }
  else
    valid_line = false;

  if ( valid_line )
  {
//  pcl::PointIndices::Ptr clustering_feature_inliers (new pcl::PointIndices ());

  // Leave Only The Biggest Cluster //
  if ( clusters.size() > 0 )
  {
    for ( int idx = 0; idx < (int) clusters.at (s).indices.size(); idx++ )
    {
      int inl = clusters.at (s).indices.at (idx);
      clustering_feature_inliers->indices.push_back (line_inliers->indices.at (inl));
    }
  }
  else
  {
    valid_line = false;
    if ( verbose ) pcl::console::print_error ("  [CLUSTERING FEATURE FOR LINES] Line Rejected !\n");
  }



  // New // Begin // ------

  pcl::PointIndices::Ptr curvature_feature_inliers (new pcl::PointIndices ());

  if ( curvature_clusters.at (s).indices.size() > 0 )
  {
    for ( int idx = 0; idx < (int) curvature_clusters.at (s).indices.size(); idx++ )
    {
      int inl = curvature_clusters.at (s).indices.at (idx);
      curvature_feature_inliers->indices.push_back (line_inliers->indices.at (inl));
    }
  }
  else
  {
    valid_line = false;
    if ( verbose ) pcl::console::print_error ("  [CURVATURE] Line Rejected !\n");
  }

//  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr curvature_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> cur_ei;
  cur_ei.setInputCloud (working_cloud);
  cur_ei.setIndices (curvature_feature_inliers);
  cur_ei.setNegative (false);
  cur_ei.filter (*curvature_feature_cloud);

  // New // End // ------



//  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr clustering_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> ei;
  ei.setInputCloud (working_cloud);
  ei.setIndices (clustering_feature_inliers);
  ei.setNegative (false);
  ei.filter (*clustering_feature_cloud);
  }





        pcl::console::print_error ("      cfp.size() = %d \n\n", (int) curvature_feature_cloud->points.size());

       // First we need to verify that there actually are 'cfp' points present //

       if ( curvature_feature_cloud->points.size() > 0 )
       {

         // There is a need for clustering the cloud which represents the 'curvature feature points' -- cfp //

          std::vector<pcl::PointIndices> cfp_clusters;
          pcl::search::KdTree<pcl::PointXYZRGBNormalRSD>::Ptr cfp_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormalRSD> ());
          cfp_tree->setInputCloud (curvature_feature_cloud);

          pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormalRSD> cfp_ece;
          cfp_ece.setInputCloud (curvature_feature_cloud);
          cfp_ece.setClusterTolerance (line_inliers_clustering_tolerance);
          cfp_ece.setMinClusterSize (2); // not 1, because pcl has problems when dealing with such a value
          cfp_ece.setSearchMethod (cfp_tree);
          cfp_ece.extract (cfp_clusters);

          if ( cfp_clusters.size() > 0 )
          {
            if ( verbose )
            {
              pcl::console::print_error ("      Curvature Feature Points -- have %d clusters with\n", (int) cfp_clusters.size());
              for (int cfp = 0; cfp < (int) cfp_clusters.size(); cfp++)
                pcl::console::print_error ("        Cluster %d has %5d points\n", cfp, (int) cfp_clusters.at (cfp).indices.size ());
              pcl::console::print_error ("\n");
            }

          pcl::PointIndices::Ptr                          largest_cfp_cluster       (new pcl::PointIndices (cfp_clusters.at (0)));
          pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr largest_cfp_cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

          pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> cfp_ei;
          cfp_ei.setInputCloud (curvature_feature_cloud);
          cfp_ei.setIndices (largest_cfp_cluster);
          cfp_ei.setNegative (false);
          cfp_ei.filter (*largest_cfp_cluster_cloud);

          // Well, do not forget to retain only the largest cluster of the 'cfp' //

          *curvature_feature_cloud = *largest_cfp_cluster_cloud;

           pcl::console::print_error ("      size of 'cfp' is %d points\n",   (int) largest_cfp_cluster_cloud->points.size ());
           pcl::console::print_error ("      size of 'cfp' is %d points\n\n", (int) curvature_feature_cloud->points.size ());

          }
        }







  // Update Inliers And Cloud Of Line //
  *line_inliers = *clustering_feature_inliers;
  *line_cloud = *clustering_feature_cloud;
  *adjust_using_curvature = *curvature_feature_cloud;

  // Adjust Coefficients Of Line Model //
  adjustLineCoefficients (line_cloud, line_coefficients);

  // XXX TODO XXX // MAYBE THE BEST WAY FOR THE FUTURE, WOULD BE TO WORK ONLY W/ LINE SEGMENTS AND CIRCLE ARCS //

  // limit the lenghts of line segments, after the same reasoning you are limiting the radius of circle models //

  double pt1[2];
  pt1[0] = line_coefficients->values.at (0);
  pt1[1] = line_coefficients->values.at (1);

  double pt2[2];
  pt2[0] = line_coefficients->values.at (3) + line_coefficients->values.at (0);
  pt2[1] = line_coefficients->values.at (4) + line_coefficients->values.at (1);

  // A.K.A. length of line //
  float lol = sqrt (_sqr (pt2[0] - pt1[0]) + _sqr (pt2[1] - pt1[1]));

  pcl::console::print_warn ("  (a) length of line = %.3f | where minimum length of line = %.3f \n", lol, minimum_line_length);

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
    viewer.addLine (*line_coefficients, 0.0, 1.0, 0.0, "LINE");
    viewer.spin ();
/*
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> curvature_feature_cloud_color (curvature_feature_cloud, 255, 255,   0);
*/
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> curvature_feature_cloud_color (curvature_feature_cloud, 127,   0, 127);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (curvature_feature_cloud, curvature_feature_cloud_color, "CURVATURE_INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "CURVATURE_INLIERS");
    viewer.spin ();
    viewer.removePointCloud ("CURVATURE_INLIERS");
    viewer.removePointCloud ("INLIERS");
    viewer.removeShape ("LINE");
    viewer.spin ();
  }

  return;
}

////////////////////////////////////////////////////////////////////////
/** \brief
 * \param valid_circle
 * \param working_cloud
 * \param circle_cloud
 * \param circle_inliers
 * \param circle_coefficients
 * \param viewer
 * \param verbose
 * \param step
 */
void ClusteringFeatureForCircles (bool &valid_circle,
                                  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &working_cloud,
                                  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &circle_cloud,
                                  pcl::PointIndices::Ptr &circle_inliers,
                                  pcl::ModelCoefficients::Ptr &circle_coefficients,
                                  pcl::visualization::PCLVisualizer &viewer,
                                  bool verbose = true,
                                  bool step = false)
{
  //if ( step )
  //{
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 0, 255, 0);
  //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "INLIERS");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
  //viewer.addCircle (*circle_coefficients, 0.0, 1.0, 0.0, "CIRCLE");
  //viewer.spin ();
  //viewer.removePointCloud ("INLIERS");
  //viewer.removeShape ("CIRCLE");
  //viewer.spin ();
  //}

  std::vector<pcl::PointIndices> clusters;
  pcl::search::KdTree<pcl::PointXYZRGBNormalRSD>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormalRSD> ());
  tree->setInputCloud (circle_cloud);

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormalRSD> ece;
  ece.setInputCloud (circle_cloud);
  ece.setClusterTolerance (circle_inliers_clustering_tolerance);
  ece.setMinClusterSize (minimum_size_of_circle_inliers_clusters);
  ece.setSearchMethod (tree);
  ece.extract (clusters);



  // IF THERE ARE NO CLUSTERS OF CIRCLE INLIERS
  bool no_clusters_of_circle_inliers = true;
  if ( clusters.size () == 0 )
  {
    valid_circle = false;
    no_clusters_of_circle_inliers = false;
  }

  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr curvature_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());
  pcl::PointIndices::Ptr clustering_feature_inliers (new pcl::PointIndices ());
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr clustering_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  if ( no_clusters_of_circle_inliers )
  {
    // New // Begin // ------

    std::vector<pcl::PointIndices> curvature_clusters (clusters.size ());

    for (int clu = 0; clu < clusters.size (); clu++ )
    {
      for (int inl = 0; inl < clusters.at (clu).indices.size (); inl++ )
      {
        int idx = clusters.at (clu).indices.at (inl);
        double curvature = circle_cloud->points.at (idx).curvature;

        //double curvature = line_cloud->points.at (inl).curvature;

        if ( curvature < curvature_threshold )
          curvature_clusters.at (clu).indices.push_back (idx);
      }
    }

    // New // End // ------

    if ( verbose )
    {
      pcl::console::print_info ("  [CLUSTERING FEATURE FOR CIRCLES] Circle has %d inliers clusters where\n", (int) clusters.size());
      for (int c = 0; c < (int) clusters.size(); c++)
        pcl::console::print_info ("  [CLUSTERING FEATURE FOR CIRCLES]   Cluster %d has %5d points with %5d planar curvatures.\n", c, (int) clusters.at (c).indices.size(), curvature_clusters. at (c).indices.size ());
    }

    // New // Begin // ------

    int minimum_size_of_curvature_clusters = INT_MAX;
    int index_of_minimum_size_of_curvature_clusters = -1;

    for (int clu = 0; clu < curvature_clusters.size (); clu++ )
    {
      if ( minimum_size_of_curvature_clusters > (int) curvature_clusters. at (clu).indices.size () )
      {
        minimum_size_of_curvature_clusters = curvature_clusters. at (clu).indices.size ();
        index_of_minimum_size_of_curvature_clusters = clu;
      }
    }

    int z = index_of_minimum_size_of_curvature_clusters;

    cerr << endl ;
    cerr << "  found cluster " << z << " with " << clusters.at (z).indices.size () << " points and " << minimum_size_of_curvature_clusters << " curvatures " << endl;
    cerr << endl ;

    // New // End // ------

    // New // Begin // ------

    pcl::PointIndices::Ptr curvature_feature_inliers (new pcl::PointIndices ());

    if ( curvature_clusters.at (z).indices.size() < 750 ) /// HEADS UP, THIS MIGHT EXCLUDE THE TALL CYLINDER ON THE LEFT, SET TO 25 MAYBE ?!
    {
      for ( int idx = 0; idx < (int) curvature_clusters.at (z).indices.size(); idx++ )
      {
        int inl = curvature_clusters.at (z).indices.at (idx);
        curvature_feature_inliers->indices.push_back (circle_inliers->indices.at (inl));
      }
    }
    else
    {
      valid_circle = false;
      if ( verbose ) pcl::console::print_error ("  [CURVATURE] Circle Rejected ! Too Many Planar Curvatures !\n");
    }

//    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr curvature_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> cur_ei;
    cur_ei.setInputCloud (working_cloud);
    cur_ei.setIndices (curvature_feature_inliers);
    cur_ei.setNegative (false);
    cur_ei.filter (*curvature_feature_cloud);

    // New // End // ------

//    pcl::PointIndices::Ptr clustering_feature_inliers (new pcl::PointIndices ());

    // NO MORE COMPUTING, IF...
    if ( valid_circle )
    {
      if ( clusters.size() > 0 )
      {
        for ( int idx = 0; idx < (int) clusters.at (z).indices.size(); idx++ )
        {
          int inl = clusters.at (z).indices.at (idx);
          clustering_feature_inliers->indices.push_back (circle_inliers->indices.at (inl));
        }
      }
      else
      {
        valid_circle = false;
        if ( verbose ) pcl::console::print_error ("  [CLUSTERING FEATURE FOR CIRCLES] Circle Rejected !\n");
      }
    }
    else
      if ( verbose ) pcl::console::print_error ("  [CLUSTERING FEATURE FOR CIRCLES] Circle Rejected ! Not Valid !\n");

//    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr clustering_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> ei;
    ei.setInputCloud (working_cloud);
    ei.setIndices (clustering_feature_inliers);
    ei.setNegative (false);
    ei.filter (*clustering_feature_cloud);
  }



/*

  // Analyze Maximum Height Of Clusters //
  if ( clusters.size() > 1 )
  {
    pcl::PointIndices::Ptr first_biggest_cluster (new pcl::PointIndices (clusters.at (0)));
    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr first_biggest_cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> fbc_ei;
    fbc_ei.setInputCloud (circle_cloud);
    fbc_ei.setIndices (first_biggest_cluster);
    fbc_ei.setNegative (false);
    fbc_ei.filter (*first_biggest_cluster_cloud);

    pcl::PointXYZRGBNormalRSD minimum_fbc, maximum_fbc;
    pcl::getMinMax3D (*first_biggest_cluster_cloud, minimum_fbc, maximum_fbc);

    double fbc_z = 0.0;
    for ( int idx = 0; idx < first_biggest_cluster_cloud->points.size (); idx++ )
    {
      if ( fbc_z < first_biggest_cluster_cloud->points.at (idx).z )
        fbc_z = first_biggest_cluster_cloud->points.at (idx).z;
    }

    if ( step )
    {
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBNormalRSD> fbc_handler (first_biggest_cluster_cloud);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (first_biggest_cluster_cloud, fbc_handler, "1ST");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "1ST");
    }

    //

    pcl::PointIndices::Ptr second_biggest_cluster (new pcl::PointIndices (clusters.at (1)));
    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr second_biggest_cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> sbc_ei;
    sbc_ei.setInputCloud (circle_cloud);
    sbc_ei.setIndices (second_biggest_cluster);
    sbc_ei.setNegative (false);
    sbc_ei.filter (*second_biggest_cluster_cloud);

    pcl::PointXYZRGBNormalRSD minimum_sbc, maximum_sbc;
    pcl::getMinMax3D (*second_biggest_cluster_cloud, minimum_sbc, maximum_sbc);

    double sbc_z = 0.0;
    for ( int idx = 0; idx < second_biggest_cluster_cloud->points.size (); idx++ )
    {
      if ( sbc_z < second_biggest_cluster_cloud->points.at (idx).z )
        sbc_z = second_biggest_cluster_cloud->points.at (idx).z;
    }

    if ( step )
    {
      pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBNormalRSD> sbc_handler (second_biggest_cluster_cloud);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (second_biggest_cluster_cloud, sbc_handler, "2ND");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "2ND");
    }

    //

    viewer.spin ();
    viewer.removePointCloud ("1ST");
    viewer.removePointCloud ("CURVATURE_INLIERS");
    viewer.removePointCloud ("2ND");

    //

    std::cerr << maximum_fbc.z << " vs " << maximum_sbc.z << std::endl;
    std::cerr << fbc_z << " vs " << sbc_z << std::endl;
    std::cerr << fabs ( maximum_fbc.z - maximum_sbc.z ) << std::endl;
    if ( fabs ( maximum_fbc.z - maximum_sbc.z ) > 0.010 )
    {
      clusters.clear ();
      valid_circle = false;
      if ( verbose ) pcl::console::print_error ("  [CLUSTERING FEATURE FOR CIRCLES] Circle Rejected ! Height Inconsistency !\n");
    }
  }

  // Leave Only The Two Biggest Clusters //
  pcl::PointIndices::Ptr clustering_feature_inliers (new pcl::PointIndices ());

  if ( clusters.size() > 0 )
  {
    for ( int idx = 0; idx < (int) clusters.at (0).indices.size(); idx++ )
    {
      int inl = clusters.at (0).indices.at (idx);
      clustering_feature_inliers->indices.push_back (circle_inliers->indices.at (inl));
    }

    if ( clusters.size() > 1 )
    {
      for ( int idx = 0; idx < (int) clusters.at (1).indices.size(); idx++ )
      {
        int inl = clusters.at (1).indices.at (idx);
        clustering_feature_inliers->indices.push_back (circle_inliers->indices.at (inl));
      }
    }
  }
  else
  {
    valid_circle = false;
    if ( verbose ) pcl::console::print_error ("  [CLUSTERING FEATURE FOR CIRCLES] Circle Rejected !\n");
  }

  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr clustering_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> ei;
  ei.setInputCloud (working_cloud);
  ei.setIndices (clustering_feature_inliers);
  ei.setNegative (false);
  ei.filter (*clustering_feature_cloud);

*/

  // Update Inliers And Cloud Of Circle //
  *circle_inliers = *clustering_feature_inliers;
  *circle_cloud = *clustering_feature_cloud;

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> curvature_feature_cloud_color (curvature_feature_cloud, 255, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (curvature_feature_cloud, curvature_feature_cloud_color, "CURVATURE_INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size * 2, "CURVATURE_INLIERS");

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 0, 255, 0);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
    viewer.addCircle (*circle_coefficients, 0.0, 1.0, 0.0, "CIRCLE");
    viewer.spin ();

    viewer.removePointCloud ("CURVATURE_INLIERS");

    viewer.removePointCloud ("INLIERS");
    viewer.removeShape ("CIRCLE");
    viewer.spin ();
  }

  return;
}

////////////////////////////////////////////////////////////////////////
/** \brief
 * \param valid_line
 * \param working_cloud
 * \param line_cloud
 * \param line_inliers
 * \param line_coefficients
 * \param viewer
 * \param verbose
 * \param step
 */
void NormalFeatureForLines (bool &valid_line,
                            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &working_cloud,
                            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &line_cloud,
                            pcl::PointIndices::Ptr &line_inliers,
                            pcl::ModelCoefficients::Ptr &line_coefficients,
                            pcl::visualization::PCLVisualizer &viewer,
                            bool verbose = true,
                            bool step = false)
{
  //if ( step )
  //{
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 0, 0, 255);
  //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "INLIERS");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
  //viewer.addLine (*line_coefficients, 0.0, 0.0, 1.0, "LINE");
  //viewer.spin ();
  //viewer.removePointCloud ("INLIERS");
  //viewer.removeShape ("LINE");
  //viewer.spin ();
  //}

  if ( verbose ) pcl::console::print_info ("  [NORMAL FEATURE FOR LINES] Line has now %d inliers\n", (int) line_inliers->indices.size());

  pcl::PointIndices::Ptr normal_feature_inliers (new pcl::PointIndices ());

  float c2p[2];
  c2p[0] = line_coefficients->values.at (3);
  c2p[1] = line_coefficients->values.at (4);

  for (int inl = 0; inl < (int) line_inliers->indices.size(); inl++)
  {
    int idx = line_inliers->indices.at (inl);

    float np[2];
    np[0] = working_cloud->points.at (idx).normal_x;
    np[1] = working_cloud->points.at (idx).normal_y;

    float lnp = sqrt (np[0]*np[0] + np[1]*np[1]);
    np[0] = np[0] / lnp;
    np[1] = np[1] / lnp;

    float dot = c2p[0]*np[0] + c2p[1]*np[1];
    float ang = acos (dot) * 180.0 / M_PI;

    if ( ((90.0 - line_normals_angle_threshold) < ang) && (ang < (90.0 + line_normals_angle_threshold)) )
    {
      // Save The Right Indices Of Points //
      normal_feature_inliers->indices.push_back (idx);
    }
  }

  if ( verbose ) pcl::console::print_info ("  [NORMAL FEATURE FOR LINES] Line has %d inliers left\n", (int) normal_feature_inliers->indices.size());

  if ( normal_feature_inliers->indices.size() == 0 )
  {
    valid_line = false;
    if ( verbose ) pcl::console::print_error ("  [NORMAL FEATURE FOR LINES] Line Rejected !\n");
  }

  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr normal_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> ei;
  ei.setIndices (normal_feature_inliers);
  ei.setInputCloud (working_cloud);
  ei.setNegative (false);
  ei.filter (*normal_feature_cloud);

  // Update Inliers And Cloud Of Line //
  *line_inliers = *normal_feature_inliers;
  *line_cloud = *normal_feature_cloud;

  // Adjust Coefficients Of Line Model //
  adjustLineCoefficients (line_cloud, line_coefficients);

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
    viewer.addLine (*line_coefficients, 0.0, 0.0, 1.0, "LINE");
    viewer.spin ();
    viewer.removePointCloud ("INLIERS");
    viewer.removeShape ("LINE");
    viewer.spin ();
  }

  return;
}

////////////////////////////////////////////////////////////////////////
/** \brief
 * \param working_cloud
 * \param circle_cloud
 * \param circle_inliers
 * \param circle_coefficients
 * \param valid_circle
 * \param verbose
 * \param step
 */
void NormalFeatureForCircles (bool &valid_circle,
                            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &working_cloud,
                            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr &circle_cloud,
                            pcl::PointIndices::Ptr &circle_inliers,
                            pcl::ModelCoefficients::Ptr &circle_coefficients,
                            pcl::visualization::PCLVisualizer &viewer,
                            bool verbose = true,
                            bool step = false)
{
  //if ( step )
  //{
  //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 0, 0, 255);
  //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "INLIERS");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
  //viewer.addCircle (*circle_coefficients, 0.0, 0.0, 1.0, "CIRCLE");
  //viewer.spin ();
  //viewer.removePointCloud ("INLIERS");
  //viewer.removeShape ("CIRCLE");
  //viewer.spin ();
  //}

  if ( verbose ) pcl::console::print_info ("  [NORMAL FEATURE FOR CIRCLES] Circle has now %d inliers\n", (int) circle_inliers->indices.size());

  pcl::PointIndices::Ptr normal_feature_inliers (new pcl::PointIndices ());

  float c[2];
  c[0] = circle_coefficients->values.at (0);
  c[1] = circle_coefficients->values.at (1);

  for (int inl = 0; inl < (int) circle_inliers->indices.size(); inl++)
  {
    int idx = circle_inliers->indices.at (inl);

    float p[2];
    p[0] = working_cloud->points.at (idx).x;
    p[1] = working_cloud->points.at (idx).y;

    float c2p[2];
    c2p[0] = p[0] - c[0];
    c2p[1] = p[1] - c[1];

    float lc2p = sqrt (c2p[0]*c2p[0] + c2p[1]*c2p[1]);
    c2p[0] = c2p[0] / lc2p;
    c2p[1] = c2p[1] / lc2p;

    float np[2];
    np[0] = working_cloud->points.at (idx).normal_x;
    np[1] = working_cloud->points.at (idx).normal_y;

    float lnp = sqrt (np[0]*np[0] + np[1]*np[1]);
    np[0] = np[0] / lnp;
    np[1] = np[1] / lnp;

    float dot = c2p[0]*np[0] + c2p[1]*np[1];
    float ang = acos (dot) * 180.0 / M_PI;

    if ( ((180.0 - circle_normals_angle_threshold) < ang) || (ang < circle_normals_angle_threshold) )
    {
      // Save The Right Indices Of Points //
      normal_feature_inliers->indices.push_back (idx);
    }
  }

  if (verbose) pcl::console::print_info ("  [NORMAL FEATURE FOR CIRCLES] Circle has %d inliers left\n", (int) normal_feature_inliers->indices.size());

  if ( normal_feature_inliers->indices.size() == 0 )
  {
    valid_circle = false;
    if ( verbose ) pcl::console::print_error ("  [NORMAL FEATURE FOR CIRCLES] Circle Rejected !\n");
  }

  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr normal_feature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> ei;
  ei.setIndices (normal_feature_inliers);
  ei.setInputCloud (working_cloud);
  ei.setNegative (false);
  ei.filter (*normal_feature_cloud);

  // Update Inliers And Cloud Of Circle //
  *circle_inliers = *normal_feature_inliers;
  *circle_cloud = *normal_feature_cloud;

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 0, 0, 255);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "INLIERS");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "INLIERS");
    viewer.addCircle (*circle_coefficients, 0.0, 0.0, 1.0, "CIRCLE");
    viewer.spin ();
    viewer.removePointCloud ("INLIERS");
    viewer.removeShape ("CIRCLE");
    viewer.spin ();
  }

  return;
}


double whatAngle (pcl::PointXYZ a, pcl::PointXYZ b, pcl::PointXYZ c)
{
  pcl::PointXYZ ab;
  ab.x = b.x - a.x;
  ab.y = b.y - a.y;

  pcl::PointXYZ cb;
  cb.x = b.x - c.x;
  cb.y = b.y - c.y;

  // dot product
  float dot = (ab.x * cb.x + ab.y * cb.y);

  // length square of both vectors
  float abSqr = ab.x * ab.x + ab.y * ab.y;
  float cbSqr = cb.x * cb.x + cb.y * cb.y;

  // square of cosine of the needed angle
  float cosSqr = dot * dot / abSqr / cbSqr;

  // this is a known trigonometric equality:
  // cos(alpha * 2) = [ cos(alpha) ]^2 * 2 - 1
  float cos2 = 2 * cosSqr - 1;

  // Here's the only invocation of the heavy function.
  // It's a good idea to check explicitly if cos2 is within [-1 .. 1] range

  const float pi = 3.141592f;

  float alpha2 =
    (cos2 <= -1) ? pi :
    (cos2 >= 1) ? 0 :
    acosf(cos2);

  float rslt = alpha2 / 2;

  float rs = rslt * 180. / pi;

  // Now revolve the ambiguities.
  // 1. If dot product of two vectors is negative - the angle is definitely
  // above 90 degrees. Still we have no information regarding the sign of the angle.

  // NOTE: This ambiguity is the consequence of our method: calculating the cosine
  // of the double angle. This allows us to get rid of calling sqrt.

  if (dot < 0)
    rs = 180 - rs;

  // 2. Determine the sign. For this we'll use the Determinant of two vectors.

  float det = (ab.x * cb.y - ab.y * cb.y);
  if (det < 0)
    rs = -rs;

  return (int) floor(rs + 0.5);
}

//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of method. Segmentation of 3D point clouds by Voting RANSAC fitted models.
*/
int main (int argc, char** argv)
{



          FILE * cad_data;
          cad_data = fopen ("cad_models_parameters.txt", "a");

          fprintf (cad_data, "\n");
          fprintf (cad_data, "Timestamp = %s \n\n", getStampOfTime().c_str());



  // ---------- Command Line Arguments ---------- //

  if (argc < 2)
  {
    printUsage (argv[0]);
    return (-1);
  }

  if (pcl::console::find_argument (argc, argv, "-h") > 0)
  {
    printUsage (argv[0]);
    pcl::console::print_info ("\nContinuing...\n\n");
  }

  std::vector<int> pcd_file_indices = pcl::console::parse_file_extension_argument (argc, argv, ".pcd");

  if (pcd_file_indices.size () == 0)
  {
    pcl::console::print_error ("No .pcd file given as input!\n");
    pcl::console::print_info  ("No .pcd file given as input!\n");
    pcl::console::print_value ("No .pcd file given as input!\n");
    pcl::console::print_warn  ("No .pcd file given as input!\n");
    return (-1);
  }

  // Method //
  pcl::console::parse_argument (argc, argv, "-VRANSAC_ITERATIONS", vransac_iterations);
  pcl::console::parse_argument (argc, argv, "-normal_search_radius", normal_search_radius);
  pcl::console::parse_argument (argc, argv, "-smoothing_search_radius", smoothing_search_radius);
  pcl::console::parse_argument (argc, argv, "-curvature_threshold", curvature_threshold);
  pcl::console::parse_argument (argc, argv, "-rsd_search_radius", rsd_search_radius);
  pcl::console::parse_argument (argc, argv, "-rsd_plane_radius", rsd_plane_radius);

  // Planar //
  pcl::console::parse_argument (argc, argv, "-significant_plane_threshold", significant_plane_threshold);
  pcl::console::parse_argument (argc, argv, "-minimum_inliers_of_significant_plane", minimum_inliers_of_significant_plane);
  pcl::console::parse_argument (argc, argv, "-maximum_iterations_of_significant_plane", maximum_iterations_of_significant_plane);
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_significant_plane_cluster", minimum_size_of_significant_plane_cluster);
  pcl::console::parse_argument (argc, argv, "-clustering_tolerance_of_significant_plane_inliers", clustering_tolerance_of_significant_plane_inliers);

  // Boundary //
  pcl::console::parse_argument (argc, argv, "-boundary_search_radius", boundary_search_radius);
  pcl::console::parse_argument (argc, argv, "-boundary_angle_threshold", boundary_angle_threshold);

  boundary_angle_threshold = boundary_angle_threshold * M_PI / 180;

  // Faster
  pcl::console::parse_argument (argc, argv, "-smoothed", smoothed);
  pcl::console::parse_argument (argc, argv, "-normal", normal);
  pcl::console::parse_argument (argc, argv, "-rsd", rsd);
  pcl::console::parse_argument (argc, argv, "-normal_refinement", normal_refinement);

  // Fitting //
  pcl::console::parse_argument (argc, argv, "-fitting_step", fitting_step);
  pcl::console::parse_argument (argc, argv, "-line_threshold", line_threshold);
  pcl::console::parse_argument (argc, argv, "-circle_threshold", circle_threshold);
  pcl::console::parse_argument (argc, argv, "-minimum_line_length", minimum_line_length);
  pcl::console::parse_argument (argc, argv, "-maximum_line_length", maximum_line_length);
  pcl::console::parse_argument (argc, argv, "-minimum_circle_radius", minimum_circle_radius);
  pcl::console::parse_argument (argc, argv, "-maximum_circle_radius", maximum_circle_radius);
  pcl::console::parse_argument (argc, argv, "-maximum_line_iterations", maximum_line_iterations);
  pcl::console::parse_argument (argc, argv, "-maximum_circle_iterations", maximum_circle_iterations);
  pcl::console::parse_argument (argc, argv, "-minimum_line_inliers", minimum_line_inliers);
  pcl::console::parse_argument (argc, argv, "-minimum_circle_inliers", minimum_circle_inliers);

  // Features //
  pcl::console::parse_argument (argc, argv, "-curvature_feature_for_lines", curvature_feature_for_lines);
  pcl::console::parse_argument (argc, argv, "-curvature_feature_for_circles", curvature_feature_for_circles);

  pcl::console::parse_argument (argc, argv, "-clustering_feature_for_lines", clustering_feature_for_lines);
  pcl::console::parse_argument (argc, argv, "-clustering_feature_for_circles", clustering_feature_for_circles);
  pcl::console::parse_argument (argc, argv, "-line_inliers_clustering_tolerance", line_inliers_clustering_tolerance);
  pcl::console::parse_argument (argc, argv, "-circle_inliers_clustering_tolerance", circle_inliers_clustering_tolerance);
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_line_inliers_clusters", minimum_size_of_line_inliers_clusters);
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_circle_inliers_clusters", minimum_size_of_circle_inliers_clusters);

  pcl::console::parse_argument (argc, argv, "-normal_feature_for_lines", normal_feature_for_lines);
  pcl::console::parse_argument (argc, argv, "-normal_feature_for_circles", normal_feature_for_circles);
  pcl::console::parse_argument (argc, argv, "-line_normals_angle_threshold", line_normals_angle_threshold);
  pcl::console::parse_argument (argc, argv, "-circle_normals_angle_threshold", circle_normals_angle_threshold);

  // Spaces //
  pcl::console::parse_argument (argc, argv, "-space_step", space_step);
  pcl::console::parse_argument (argc, argv, "-clustering_tolerance_of_line_parameters_space", clustering_tolerance_of_line_parameters_space);
  pcl::console::parse_argument (argc, argv, "-clustering_tolerance_of_circle_parameters_space", clustering_tolerance_of_circle_parameters_space);
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_line_parameters_clusters", minimum_size_of_line_parameters_clusters);
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_circle_parameters_clusters", minimum_size_of_circle_parameters_clusters);
  pcl::console::parse_3x_arguments (argc, argv, "-viewpoint_translation", tX, tY, tZ);
  pcl::console::parse_3x_arguments (argc, argv, "-viewpoint_rotation", rX, rY, rZ);
  pcl::console::parse_3x_arguments (argc, argv, "-viewpoint", vX, vY, vZ);

  // Growing //
  pcl::console::parse_argument (argc, argv, "-growing_step", growing_step);
  pcl::console::parse_argument (argc, argv, "-growing_height", growing_height);
  pcl::console::parse_argument (argc, argv, "-growing_visualization", growing_visualization);
  pcl::console::parse_argument (argc, argv, "-mean_k_filter", mean_k_filter);
  pcl::console::parse_argument (argc, argv, "-std_dev_filter", std_dev_filter);

  // Rest //
  pcl::console::parse_argument (argc, argv, "-r_clustering_tolerance", r_clustering_tolerance);
  pcl::console::parse_argument (argc, argv, "-minimum_size_of_r_clusters", minimum_size_of_r_clusters);

  // Visualization //
  pcl::console::parse_argument (argc, argv, "-verbose", verbose);
  pcl::console::parse_argument (argc, argv, "-step", step);
  pcl::console::parse_argument (argc, argv, "-size", size);

  pcl::console::parse_argument (argc, argv, "-denoising", denoising);
  pcl::console::parse_argument (argc, argv, "-smoothing", smoothing);

  pcl::console::parse_argument (argc, argv, "-sign", sign);

  pcl::console::parse_argument (argc, argv, "-till_the_end", till_the_end);
  pcl::console::parse_argument (argc, argv, "-classification", classification);

  pcl::console::parse_argument (argc, argv, "-flat_value", flat_value);

  pcl::console::parse_argument (argc, argv, "-deal_with_the_rest_of_the_points", deal_with_the_rest_of_the_points);
  pcl::console::parse_argument (argc, argv, "-consider_height_from_table_plane", consider_height_from_table_plane);
  pcl::console::parse_argument (argc, argv, "-shapes_from_table_plane", shapes_from_table_plane);

  pcl::console::parse_argument (argc, argv, "-tall_value", tall_value);
  pcl::console::parse_argument (argc, argv, "-medium_value", medium_value);
  pcl::console::parse_argument (argc, argv, "-short_value", short_value);

  // Different //
  pcl::console::parse_argument (argc, argv, "-sat", sat);

  // Centroids //
  pcl::console::parse_argument (argc, argv, "-threshold_between_centroids_of_cuboids", threshold_between_centroids_of_cuboids);
  pcl::console::parse_argument (argc, argv, "-threshold_between_centroids_of_cylinders", threshold_between_centroids_of_cylinders);

  // New //
  pcl::console::parse_argument (argc, argv, "-too_many_planar_curvatures", too_many_planar_curvatures);

  // ---------- Initializations ---------- //

  srand (time(0));

  pcl::console::TicToc tt;

  tt.tic ();

  if ( verbose ) pcl::console::print_warn ("\nTimer started !\n");

  // ---------- 3D Viewer ---------- //

  pcl::visualization::PCLVisualizer viewer ("3D VIEWER");

  viewer.setBackgroundColor (1.0, 1.0, 1.0);
  viewer.addCoordinateSystem (0.25f);
  viewer.getCameraParameters (argc, argv);
  viewer.updateCamera ();

  //pcl::ModelCoefficients xOy;
  //xOy.values.push_back (0.0);
  //xOy.values.push_back (0.0);
  //xOy.values.push_back (-1);
  //xOy.values.push_back (0.125);
  //
  //viewer.addPlane (xOy, 1.0, 0.5, 0.0, "xOy");
  //viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "xOy");
  //viewer.spin ();

  std::vector<std::vector<pcl::ModelCoefficients> > cyls;
  std::vector<std::vector<std::vector<pcl::ModelCoefficients> > > cubs;

  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr > > clouds_of_cyls;
  std::vector<std::vector<pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr > > clouds_of_cubs;

  if ( true )
  {

  for (int fff = 0; fff < pcd_file_indices.size (); fff++)
  {

  std::vector<pcl::ModelCoefficients> cyls_per_view;
  std::vector<std::vector<pcl::ModelCoefficients> > cubs_per_view;

  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr > clouds_of_cyls_per_view;
  std::vector<pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr > clouds_of_cubs_per_view;

  pcl::console::print_value ("\nNow processing %s\n\n", argv [pcd_file_indices [fff]]);

  // ------------------------------------------- //
  // ---------- Declare Working Cloud ---------- //
  // ------------------------------------------- //
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr working_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  // ---------- Load Input Data ---------- //

  pcl::PointCloud<I>::Ptr input_cloud (new pcl::PointCloud<I> ());

  if (pcl::io::loadPCDFile (argv [pcd_file_indices [fff]], *input_cloud) == -1)
  {
    pcl::console::print_error ("Couldn't read file %s\n", argv [pcd_file_indices [fff]]);
    return (-1);
  }

  // ------------------------------------------ //
  // ---------- Create Working Cloud ---------- //
  // ------------------------------------------ //
  pcl::copyPointCloud (*input_cloud, *working_cloud);

  if ( verbose ) pcl::console::print_info ("Loaded %d data points from %s with the following fields: %s\n", (int) (working_cloud->points.size ()), argv [pcd_file_indices [fff]], pcl::getFieldsList (*working_cloud).c_str ());

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_color (working_cloud, 0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_color, "generic");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
    viewer.spin ();
  }

  // ---------- Path and Name of File ---------- //

  std::string file = argv [pcd_file_indices [fff]];
  size_t dot = file.find_last_of (".");
  std::string name = file.substr (0, dot);
  cerr << name << endl;

  // ---------- For Classification ---------- //

  std::string directory;

  size_t f;

  if ( classification )
  {
    std::string file = argv [pcd_file_indices [fff]];
    f = file.find_last_of (".");
    directory = file.substr (0, f);

    //cerr << f << endl ;
    //cerr << directory << endl;
  }

/*
//        // ---------- Filter Point Cloud Data ---------- //
//
//        pcl::PointCloud<I>::Ptr filtered_cloud (new pcl::PointCloud<I> ());
//
//        pcl::StatisticalOutlierRemoval<I> sor;
//        sor.setInputCloud (input_cloud);
//        sor.setMeanK (mean_k_filter);
//        sor.setStddevMulThresh (std_dev_filter);
//        sor.filter (*filtered_cloud);
//
//        // ------------------------------------------ //
//        // ---------- Update Working Cloud ---------- //
//        // ------------------------------------------ //
//        pcl::copyPointCloud (*filtered_cloud, *working_cloud);
//
//        if ( verbose )
//          pcl::console::print_info ("Filtering... Before: %d points | After: %d points | Filtered: %d points\n", (int) input_cloud->points.size (), (int) working_cloud->points.size (), (int) input_cloud->points.size () - (int) working_cloud->points.size ());
//
//        if ( step )
//        {
//          viewer.removePointCloud ("generic");
//          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_color (working_cloud, 0, 0, 0);
//          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_color, "generic");
//          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
//          viewer.spin ();
//        }
//
//        // ---------- For Classification ---------- //
//
//        if ( classification )
//        {
//          std::stringstream filtered_output_filename;
//          filtered_output_filename << directory << "---" << "denoise.pcd" ;
//          pcl::io::savePCDFileASCII (filtered_output_filename.str (), *filtered_cloud);
//
//          std::string file = filtered_output_filename.str ();
//          f = file.find_last_of (".");
//          directory = file.substr (0, f);
//
//          //cerr << f << endl ;
//          //cerr << directory << endl;
//        }
//
//        //pcl::PointCloud<I>::Ptr filtered_cloud (new pcl::PointCloud<I> ());
//        //*filtered_cloud = *input_cloud;
*/

  // ---------- Smoothing ---------- //

  pcl::PointCloud<I>::Ptr smooth_cloud (new pcl::PointCloud<I> ());

if (smoothing_search_radius == 0)
{
  if (verbose) pcl::console::print_error (" -- No smoothing!\n");
  *smooth_cloud = *input_cloud;
}
else
{

  pcl::search::KdTree<I>::Ptr mls_tree (new pcl::search::KdTree<I> ());
/*
        mls_tree->setInputCloud (filtered_cloud);
*/
  mls_tree->setInputCloud (input_cloud);

  if ( !smoothed )
  {
    if ( verbose ) pcl::console::print_info ("Started smoothing of surfaces... \n");

    pcl::MovingLeastSquares<I, pcl::Normal> mls;
/*
          mls.setInputCloud (filtered_cloud);
*/
    mls.setInputCloud (input_cloud);
    mls.setPolynomialFit (true);
    mls.setSearchMethod (mls_tree);
    mls.setSearchRadius (smoothing_search_radius);
    mls.reconstruct (*smooth_cloud);

    if ( verbose ) pcl::console::print_info ("[Done!]\n");

    // ------------------------------------------ //
    // ---------- Update Working Cloud ---------- //
    // ------------------------------------------ //
    pcl::copyPointCloud (*smooth_cloud, *working_cloud);

    if ( step )
    {
      viewer.removePointCloud ("generic");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_color (working_cloud, 0, 0, 0);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_color, "generic");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
      viewer.spin ();
    }

    // ---------- For Classification ---------- //

    if ( classification )
    {
      std::stringstream smooth_output_filename;
      smooth_output_filename << directory << "_mls" << smoothing_search_radius << ".pcd" ;
      pcl::io::savePCDFileASCII (smooth_output_filename.str (), *smooth_cloud);

      std::string file = smooth_output_filename.str ();
      f = file.find_last_of (".");
      directory = file.substr (0, f);

      //cerr << f << endl ;
      //cerr << directory << endl;
    }
  }

}






        // ---------- Filtering of Smoothed Data Set ---------- //

        pcl::PointCloud<I>::Ptr filtered_cloud (new pcl::PointCloud<I> ());

if ((mean_k_filter == 0) && (std_dev_filter == 0))
{
  if (verbose) pcl::console::print_error (" -- No filtering!\n");
  *filtered_cloud = *smooth_cloud;
}
else
{

        pcl::StatisticalOutlierRemoval<I> sor;
        sor.setInputCloud (smooth_cloud);
        sor.setMeanK (mean_k_filter);
        sor.setStddevMulThresh (std_dev_filter);
        sor.filter (*filtered_cloud);

        // ------------------------------------------ //
        // ---------- Update Working Cloud ---------- //
        // ------------------------------------------ //
        pcl::copyPointCloud (*filtered_cloud, *working_cloud);

        if ( verbose )
          pcl::console::print_info ("Filtering... Before: %d points | After: %d points | Filtered: %d points\n", (int) smooth_cloud->points.size (), (int) working_cloud->points.size (), (int) smooth_cloud->points.size () - (int) working_cloud->points.size ());

        if ( step )
        {
          viewer.removePointCloud ("generic");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_color (working_cloud, 0, 0, 0);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_color, "generic");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
          viewer.spin ();
        }

        // ---------- For Classification ---------- //

        if ( classification )
        {
          std::stringstream filtered_output_filename;
          filtered_output_filename << directory << "_sor" << mean_k_filter << ":" << std_dev_filter << ".pcd" ;
          pcl::io::savePCDFileASCII (filtered_output_filename.str (), *filtered_cloud);

          std::string file = filtered_output_filename.str ();
          f = file.find_last_of (".");
          directory = file.substr (0, f);

          //cerr << f << endl ;
          //cerr << directory << endl;
        }

}






  // ---------- Estimate 3D Normals ---------- //

  pcl::PointCloud<N>::Ptr normal_cloud (new pcl::PointCloud<N> ());
  pcl::search::KdTree<pcl::PointXYZRGBNormalRSD>::Ptr normal_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormalRSD> ());
  normal_tree->setInputCloud (working_cloud);

  if ( !normal )
  {
    if ( verbose ) pcl::console::print_info ("Started normal estimation of points... \n");

    pcl::NormalEstimation<pcl::PointXYZRGBNormalRSD, N> ne;
    ne.setInputCloud (working_cloud);
    ne.setSearchMethod (normal_tree);
    ne.setRadiusSearch (normal_search_radius);
    ne.compute (*normal_cloud);

    if ( verbose ) pcl::console::print_info ("[Done!]\n");

    // ------------------------------------------ //
    // ---------- Update Working Cloud ---------- //
    // ------------------------------------------ //
    pcl::copyPointCloud (*normal_cloud, *working_cloud);

    if ( false )
    {
      viewer.addPointCloudNormals<pcl::PointXYZRGBNormalRSD, N> (working_cloud, normal_cloud, 10, 0.025, "normal");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 1.0, "normal");
      viewer.spin ();
      viewer.removePointCloud ("normal");
      viewer.spin ();
    }
  }

/*         *
 * Section *
 *         *

  // ---------------------------- //
  // ---------------------------- //
  // ---------- Faster ---------- //
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr smooth_cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());

  if (( !smoothed ) && ( !normal ))
  {
    pcl::copyPointCloud (*smooth_cloud, *smooth_cloud_with_normals);
    pcl::copyPointCloud (*normal_cloud, *smooth_cloud_with_normals);

    std::stringstream smooth_file_with_normals;
    smooth_file_with_normals << name << "_smoothed-" << smoothing_search_radius << "-normal-" << normal_search_radius <<".pcd" ;
    pcl::io::savePCDFileASCII (smooth_file_with_normals.str(), *smooth_cloud_with_normals);
    if ( verbose ) pcl::console::print_info ("The smoothed cloud with normals was saved!\n");
  }

  if (( smoothed ) && ( normal ))
  {
    std::stringstream smooth_file_with_normals;
    smooth_file_with_normals << name << "_smoothed-" << smoothing_search_radius << "-normal-" << normal_search_radius <<".pcd" ;
    std::string smooth_file_with_normals_as_string = smooth_file_with_normals.str();

    if (pcl::io::loadPCDFile (smooth_file_with_normals.str(), *smooth_cloud_with_normals) == -1)
    {
      pcl::console::print_error ("Couldn't read file %s\n", smooth_file_with_normals.str().c_str());
      return (-1);
    }

    if ( verbose ) pcl::console::print_info ("Loaded %d points from %s with fields: %s\n", (int) (smooth_cloud_with_normals->points.size ()), smooth_file_with_normals.str().c_str(), pcl::getFieldsList (*smooth_cloud_with_normals).c_str ());

    // ------------------------------------------ //
    // ---------- Update Working Cloud ---------- //
    // ------------------------------------------ //
    pcl::copyPointCloud (*smooth_cloud_with_normals, *working_cloud);

    if ( step )
    {
      viewer.removePointCloud ("generic");
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_color (working_cloud, 0, 0, 0);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_color, "generic");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
      viewer.spin ();
    }

    // ---------- Just For Classification Sake ---------- //

    if ( classification )
    {
      pcl::copyFields (*smooth_cloud_with_normals, *smooth_cloud);

      std::stringstream smooth_output_filename;
      smooth_output_filename << directory << "-" << "smooth.pcd" ;
      pcl::io::savePCDFileASCII (smooth_output_filename.str (), *smooth_cloud);

      std::string file = smooth_output_filename.str ();
      f = file.find_last_of (".");
      directory = file.substr (0, f);

      //cerr << f << endl ;
      //cerr << directory << endl;
    }
  }
  // ---------- Faster ---------- //
  // ---------------------------- //
  // ---------------------------- //

*         *
* Section *
*         */

  if ( verbose ) pcl::console::print_info ("Curvature Estimation ! Returned: %d curvatures\n", (int) working_cloud->points.size ());

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGBNormalRSD> curvature_colors (working_cloud, "curvature");
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, curvature_colors, "CURVATURE");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "CURVATURE");
    viewer.spin ();

    viewer.removePointCloud ("CURVATURE");
    viewer.spin ();
  }

  // Split Planar From Circular Surface Points //
  pcl::PointIndices::Ptr planar_curvature_indices (new pcl::PointIndices ());
  pcl::PointIndices::Ptr circular_curvature_indices (new pcl::PointIndices ());

  for (int idx = 0; idx < (int) working_cloud->points.size(); idx++)
  {
    double curvature = working_cloud->points.at (idx).curvature;

    if ( curvature < curvature_threshold )
      planar_curvature_indices->indices.push_back (idx);
    else
      circular_curvature_indices->indices.push_back (idx);
  }

  // Extract Planar And Circular Surface Points //
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr planar_curvature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr circular_curvature_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> c_ei;
  c_ei.setInputCloud (working_cloud);

  c_ei.setIndices (planar_curvature_indices);
  c_ei.setNegative (false);
  c_ei.filter (*planar_curvature_cloud);

  c_ei.setIndices (circular_curvature_indices);
  c_ei.setNegative (false);
  c_ei.filter (*circular_curvature_cloud);

  if ( verbose ) pcl::console::print_info ("Curvature Mapping ! Returned: %d planars vs %d circulars\n", (int) planar_curvature_cloud->points.size (), (int) circular_curvature_cloud->points.size ());

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBNormalRSD> planar_curvature_cloud_color (planar_curvature_cloud);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBNormalRSD> circular_curvature_cloud_color (circular_curvature_cloud);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (planar_curvature_cloud, planar_curvature_cloud_color, "PLANAR_CURVATURE_CLOUD");
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circular_curvature_cloud, circular_curvature_cloud_color, "CIRCULAR_CURVATURE_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "PLANAR_CURVATURE_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "CIRCULAR_CURVATURE_CLOUD");
    viewer.spin ();

    viewer.removePointCloud ("PLANAR_CURVATURE_CLOUD");
    viewer.removePointCloud ("CIRCULAR_CURVATURE_CLOUD");
    viewer.spin ();
  }

  if ( !rsd )
  {

  // ---------- Estimate RSD Values ---------- //

  pcl::PointCloud<pcl::PrincipalRadiiRSD>::Ptr rsd_cloud (new pcl::PointCloud<pcl::PrincipalRadiiRSD> ());
  pcl::search::KdTree<I>::Ptr rsd_tree (new pcl::search::KdTree<I> ());
  rsd_tree->setInputCloud (smooth_cloud);

  pcl::RSDEstimation<I, pcl::Normal, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud (smooth_cloud);
  rsd.setInputNormals (normal_cloud);
  rsd.setRadiusSearch (rsd_search_radius);
  rsd.setPlaneRadius (rsd_plane_radius);
  rsd.setSearchMethod (rsd_tree);
  rsd.compute (*rsd_cloud);

  // ------------------------------------------ //
  // ---------- Update Working Cloud ---------- //
  // ------------------------------------------ //
  pcl::copyPointCloud (*rsd_cloud, *working_cloud);

  if ( verbose ) pcl::console::print_info ("RSD Estimation ! Returned: %d rsd values\n", (int) working_cloud->points.size ());

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZRGBNormalRSD> r_min_colors (working_cloud, "r_min");
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, r_min_colors, "R_MIN");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "R_MIN");
    viewer.spin ();

    viewer.removePointCloud ("R_MIN");
    viewer.spin ();
  }

  /*

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr _rsd_cloud_ (new pcl::PointCloud<pcl::PointXYZRGBNormal> ());
  for (int idx = 0; idx < (int) rsd_cloud->points.size (); idx++)
  {
    double r_min = rsd_cloud->points[idx].r_min;
    double r_max = rsd_cloud->points[idx].r_max;

    pcl::PointXYZRGBNormal p;

           p.x = working_cloud->points[idx].x;
           p.y = working_cloud->points[idx].y;
           p.z = working_cloud->points[idx].z;
         p.rgb = working_cloud->points[idx].rgb;
    p.normal_x = r_min;
    p.normal_y = r_max;
    p.normal_z = working_cloud->points[idx].normal_z;

    _rsd_cloud_->points.push_back (p);
  }

  _rsd_cloud_->height = 1;
  _rsd_cloud_->width  = rsd_cloud->points.size ();
  pcl::io::savePCDFile ("rsd.pcd", *_rsd_cloud_);

  */

  // Split Pausible From Implausible Radius Values //
  pcl::PointIndices::Ptr plausible_r_min_indices (new pcl::PointIndices ());
  pcl::PointIndices::Ptr implausible_r_min_indices (new pcl::PointIndices ());

  for (int idx = 0; idx < (int) working_cloud->points.size(); idx++)
  {
    double r_min = working_cloud->points.at (idx).r_min;

    if ( (low_r_min < r_min) && (r_min < high_r_min) )
      plausible_r_min_indices->indices.push_back (idx);
    else
      implausible_r_min_indices->indices.push_back (idx);
  }

  // Extract Plausible And Implausible Radius Values //
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr plausible_r_min_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr implausible_r_min_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> r_ei;
  r_ei.setInputCloud (working_cloud);

  r_ei.setIndices (plausible_r_min_indices);
  r_ei.setNegative (false);
  r_ei.filter (*plausible_r_min_cloud);

  r_ei.setIndices (implausible_r_min_indices);
  r_ei.setNegative (false);
  r_ei.filter (*implausible_r_min_cloud);

  if ( verbose ) pcl::console::print_info ("RSD Mapping ! Returned: %d plausibles vs %d implausibles\n", (int) plausible_r_min_cloud->points.size (), (int) implausible_r_min_cloud->points.size ());

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBNormalRSD> plausible_r_min_cloud_color (plausible_r_min_cloud);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZRGBNormalRSD> implausible_r_min_cloud_color (implausible_r_min_cloud);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (plausible_r_min_cloud, plausible_r_min_cloud_color, "PLAUSIBLE_R_MIN_CLOUD");
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (implausible_r_min_cloud, implausible_r_min_cloud_color, "IMPLAUSIBLE_R_MIN_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "PLAUSIBLE_R_MIN_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "IMPLAUSIBLE_R_MIN_CLOUD");
    viewer.spin ();

    viewer.removePointCloud ("PLAUSIBLE_R_MIN_CLOUD");
    viewer.removePointCloud ("IMPLAUSIBLE_R_MIN_CLOUD");
    viewer.spin ();
  }

  }

  if ( !normal_refinement )
  {

  // ---------- Compute 2D Normals ---------- //

  for (int idx = 0; idx < (int) working_cloud->points.size (); idx++)
  {
    // Force Normals From 3D To 2D
    working_cloud->points[idx].normal_z = 0.0;
  }

  if ( verbose ) pcl::console::print_info ("Normal Flattening ! Returned: %d 2D normals\n", (int) working_cloud->points.size ());

  if ( false )
  {
    viewer.addPointCloudNormals<pcl::PointXYZRGBNormalRSD> (working_cloud, 1, 0.025, "NORMAL_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, "NORMAL_CLOUD");
    viewer.spin ();

    viewer.removePointCloud ("NORMAL_CLOUD");
    viewer.spin ();
  }

  // Refine Normals In 2D Space
  for (int idx = 0; idx < (int) working_cloud->points.size (); idx++)
  {
    double nx = working_cloud->points[idx].normal_x;
    double ny = working_cloud->points[idx].normal_y;
    double nz = working_cloud->points[idx].normal_z;

    double nl = sqrt (nx*nx + ny*ny + nz*nz);
    nx = nx / nl;
    ny = ny / nl;
    nz = nz / nl;

    working_cloud->points[idx].normal_x = nx;
    working_cloud->points[idx].normal_y = ny;
    working_cloud->points[idx].normal_z = nz;
  }

  if ( verbose ) pcl::console::print_info ("Normal Refinement ! Returned: %d normals\n", (int) working_cloud->points.size ());

  if ( false )
  {
    viewer.addPointCloudNormals<pcl::PointXYZRGBNormalRSD> (working_cloud, 1, 0.025, "NORMAL_CLOUD");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "NORMAL_CLOUD");
    viewer.spin ();

    viewer.removePointCloud ("NORMAL_CLOUD");
    viewer.spin ();
  }

  }

  //////

  // ---------- Height Of Table ---------- //

  pcl::PointXYZRGBNormalRSD working_cloud_minimum, working_cloud_maximum;
  pcl::getMinMax3D (*working_cloud, working_cloud_minimum, working_cloud_maximum);
  // TODO Lines with identical functionality //
  pcl::PointXYZRGBNormalRSD abs_min, abs_max;
  pcl::getMinMax3D (*working_cloud, abs_min, abs_max);

  //////

  // ---------- For Classification ---------- //

  // ------------------------------------------------- //
  // ---------- Create Marked Working Cloud ---------- //
  // ------------------------------------------------- //
  pcl::PointCloud<pcl::PointXYZRGBI>::Ptr marked_working_cloud (new pcl::PointCloud<pcl::PointXYZRGBI> ());
  pcl::copyPointCloud (*working_cloud, *marked_working_cloud);

  ///*
  //marked_working_cloud->points.resize (working_cloud->points.size ());
  //marked_working_cloud->width        = working_cloud->width;
  //marked_working_cloud->height       = working_cloud->height;
  //if (!working_cloud->is_dense) marked_working_cloud->is_dense = false; else marked_working_cloud->is_dense = true;

  for (size_t i=0; i < working_cloud->points.size (); ++i)
  {
    //marked_working_cloud->points.at (i).x         = working_cloud->points.at (i).x;
    //marked_working_cloud->points.at (i).y         = working_cloud->points.at (i).y;
    //marked_working_cloud->points.at (i).z         = working_cloud->points.at (i).z;
    //marked_working_cloud->points.at (i).rgb       = working_cloud->points.at (i).rgb;

    marked_working_cloud->points.at (i).intensity = -1;
  }
  //*/

  if ( step )
  {
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBI> marked_color (marked_working_cloud);
    viewer.addPointCloud<pcl::PointXYZRGBI> (marked_working_cloud, marked_color, "marked_generic");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "marked_generic");
    viewer.spin ();
    viewer.removePointCloud ("marked_generic");
  }

  //////

  // Adding The Working Cloud //
  viewer.removePointCloud ("generic");
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_color (working_cloud, 0, 0, 0);
  viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_color, "generic");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");

  // Declare Backup Working Cloud //
  pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr backup_working_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

  // Backup Working Cloud //
  *backup_working_cloud = *working_cloud;

  bool rerun = false;

  do
  {

  cyls_per_view.clear();
  cubs_per_view.clear();
  clouds_of_cyls_per_view.clear();
  clouds_of_cubs_per_view.clear();
  *working_cloud = *backup_working_cloud;

  // -------------------------------------------------- //
  // ---------- Take Care of Planars Objects ---------- //
  // -------------------------------------------------- //

  int model = 0;

  if ( sign )
  {
    cerr << endl << endl << endl ;

    pcl::PointIndices::Ptr significant_plane_inliers (new pcl::PointIndices ());
    pcl::ModelCoefficients::Ptr significant_plane_coefficients (new pcl::ModelCoefficients ());

    pcl::SACSegmentation<pcl::PointXYZRGBNormalRSD> sign_sacs;
    sign_sacs.setOptimizeCoefficients (true);
    sign_sacs.setModelType (pcl::SACMODEL_PLANE);
    sign_sacs.setMethodType (pcl::SAC_RANSAC);
    sign_sacs.setDistanceThreshold (significant_plane_threshold);
    sign_sacs.setMaxIterations (maximum_iterations_of_significant_plane);
    sign_sacs.setInputCloud (working_cloud);
    sign_sacs.segment (*significant_plane_inliers, *significant_plane_coefficients);

    if ( verbose ) pcl::console::print_info ("Significant plane has %5d inliers, found in maximum %d iterations\n", (int) significant_plane_inliers->indices.size (), maximum_iterations_of_significant_plane);

    // Check if the fitted circle has enough inliers in order to be accepted
    if ((int) significant_plane_inliers->indices.size () < minimum_inliers_of_significant_plane)
    {
      pcl::console::print_error ("NOT ACCEPTED !\n");
    }
    else
    {
      {
        {
          pcl::console::print_info ("ACCEPTED !\n");

          pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr significant_plane_inliers_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

          pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> sign_ee;
          sign_ee.setInputCloud (working_cloud);
          sign_ee.setIndices (significant_plane_inliers);
          sign_ee.setNegative (false);
          sign_ee.filter (*significant_plane_inliers_cloud);
          //sign_ee.setNegative (true);
          //sign_ee.filter (*working_cloud);

          if ( step )
          //if ( true )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> significant_plane_inliers_color (significant_plane_inliers_cloud, 0, 255, 255);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (significant_plane_inliers_cloud, significant_plane_inliers_color, "plane");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size *3, "plane");
            viewer.spin ();

            viewer.removePointCloud ("plane");
            //viewer.removePointCloud ("generic");
            //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_color (working_cloud, 0, 0, 0);
            //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_color, "generic");
            //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
            //viewer.spin ();
          }

          // XXX // BREAK //

          std::vector<pcl::PointIndices> sign_clusters;
          pcl::search::KdTree<pcl::PointXYZRGBNormalRSD>::Ptr sign_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormalRSD> ());
          sign_tree->setInputCloud (significant_plane_inliers_cloud);

          pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormalRSD> sign_ece;
          sign_ece.setInputCloud (significant_plane_inliers_cloud);
          sign_ece.setClusterTolerance (clustering_tolerance_of_significant_plane_inliers);
          sign_ece.setMinClusterSize (minimum_size_of_significant_plane_cluster);
          sign_ece.setSearchMethod (sign_tree);
          sign_ece.extract (sign_clusters);

          pcl::PointIndices::Ptr biggest_cluster_of_plane (new pcl::PointIndices (sign_clusters.at (0)));
          pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr biggest_cluster_of_plane_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

          pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> sign_ei;
          sign_ei.setInputCloud (significant_plane_inliers_cloud);
          sign_ei.setIndices (biggest_cluster_of_plane);
          sign_ei.setNegative (false);
          sign_ei.filter (*biggest_cluster_of_plane_cloud);

          if ( verbose )
          {
            pcl::console::print_info ("The plane has %d clusters where:\n", (int) sign_clusters.size ());
            for (int clu = 0; clu < (int) sign_clusters.size(); clu++)
              pcl::console::print_info ("  Cluster %d has %d points\n", clu, (int) sign_clusters.at (clu).indices.size());
          }

          if ( step )
          //if ( true )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> big_color (biggest_cluster_of_plane_cloud, 0, 255, 255);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (biggest_cluster_of_plane_cloud, big_color, "big_");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size * 3, "big_");
            viewer.spin ();
            viewer.removePointCloud ("big_");
          }

          // XXX // BREAK //

          pcl::PointCloud<B>::Ptr boundary (new pcl::PointCloud<B> ());
          pcl::search::KdTree<pcl::PointXYZRGBNormalRSD>::Ptr boundary_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormalRSD> ());
          boundary_tree->setInputCloud (biggest_cluster_of_plane_cloud);

          pcl::BoundaryEstimation<pcl::PointXYZRGBNormalRSD, pcl::PointXYZRGBNormalRSD, B> big_sign_be;
          big_sign_be.setSearchMethod (boundary_tree);
          big_sign_be.setInputCloud (biggest_cluster_of_plane_cloud);
          big_sign_be.setInputNormals (biggest_cluster_of_plane_cloud);
          big_sign_be.setRadiusSearch (boundary_search_radius);
          big_sign_be.setAngleThreshold (boundary_angle_threshold);
          big_sign_be.compute (*boundary);

          pcl::PointIndices::Ptr boundary_inliers (new pcl::PointIndices ());

          for (int i = 0; i < boundary->points.size (); i++)
            if (boundary->points.at (i).boundary_point)
              boundary_inliers->indices.push_back (i);

          if ( verbose ) pcl::console::print_info ("There are %4d boundary points.\n", boundary_inliers->indices.size ());

          pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr boundary_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

          pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> big_sign_ee;
          big_sign_ee.setInputCloud (biggest_cluster_of_plane_cloud);
          big_sign_ee.setIndices (boundary_inliers);
          big_sign_ee.setNegative (false);
          big_sign_ee.filter (*boundary_cloud);

          if ( step )
          //if ( true )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> boundary_color (boundary_cloud, 0, 255, 255);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (boundary_cloud, boundary_color, "bou_");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size *3, "bou_");
            viewer.spin ();
            viewer.removePointCloud ("bou_");
          }

          // XXX // BREAK //

          pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());
          pcl::ModelCoefficients::Ptr line_coefficients (new pcl::ModelCoefficients ());
          pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr line_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

          int line_iterations = fitLine (boundary_cloud, line_inliers, line_coefficients, line_threshold, maximum_line_iterations);

          adjustLineInliers (boundary_cloud, line_inliers, line_coefficients, line_threshold);

          pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> f_ei;

          f_ei.setIndices (line_inliers);
          f_ei.setInputCloud (boundary_cloud);
          f_ei.setNegative (false);
          f_ei.filter (*line_cloud);

          adjustLineCoefficients (line_cloud, line_coefficients);

          if ( step )
          //if ( true )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 255, 0, 0);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "LINE_CLOUD_ID");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size *3, "LINE_CLOUD_ID");
            viewer.addLine (*line_coefficients, 0.0, 1.0, 0.0, "LINE_MODEL_ID");
            viewer.spin ();
            viewer.removePointCloud ("LINE_CLOUD_ID");
            viewer.removeShape ("LINE_MODEL_ID");
            viewer.spin ();
          }






          // NOVEL 1 WAY OF GROWING BOXES //

          double P1[2];
          P1[0] = line_coefficients->values.at (0);
          P1[1] = line_coefficients->values.at (1);

          double P2[2];
          P2[0] = line_coefficients->values.at (3) + line_coefficients->values.at (0);
          P2[1] = line_coefficients->values.at (4) + line_coefficients->values.at (1);

          pcl::ModelCoefficients::Ptr test_coeffs (new pcl::ModelCoefficients ());
          test_coeffs->values.push_back (P1[0]);
          test_coeffs->values.push_back (P1[1]);
          test_coeffs->values.push_back (0.0);
          test_coeffs->values.push_back (P2[0] - P1[0]);
          test_coeffs->values.push_back (P2[1] - P1[1]);
          test_coeffs->values.push_back (0.0);

          if ( growing_visualization )
          {
            viewer.addLine (*test_coeffs, 1.0, 0.0, 0.5, "test_COEFFS");
            viewer.spin ();
            viewer.removeShape ("test_COEFFS");
          }

          pcl::PointXYZRGBNormalRSD MiN, MaX;
          pcl::getMinMax3D (*line_cloud, MiN, MaX);

          pcl::PointIndices::Ptr novel_box_inliers (new pcl::PointIndices ());
          pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr novel_box_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

          double M[2];
          M[0] = (P1[0] + P2[0]) / 2;
          M[1] = (P1[1] + P2[1]) / 2;

          //////////////////
          // From M To P1 //
          //////////////////

          bool em1_bool = true;

          double M1[2];
          M1[0] = M[0];
          M1[1] = M[1];

          do
          {

            double P2M1V[2];
            P2M1V[0] = M1[0] - P2[0];
            P2M1V[1] = M1[1] - P2[1];

            double P2M1N = sqrt (P2M1V[0]*P2M1V[0] + P2M1V[1]*P2M1V[1]);
            P2M1V[0] = P2M1V[0] / P2M1N;
            P2M1V[1] = P2M1V[1] / P2M1N;

            double EM1[2];
            EM1[0] = M1[0] + P2M1V[0] * growing_step;
            EM1[1] = M1[1] + P2M1V[1] * growing_step;

            pcl::ModelCoefficients::Ptr em1_coeffs (new pcl::ModelCoefficients ());
            em1_coeffs->values.push_back (EM1[0]);
            em1_coeffs->values.push_back (EM1[1]);
            em1_coeffs->values.push_back (0.0);
            em1_coeffs->values.push_back (M1[0] - EM1[0]);
            em1_coeffs->values.push_back (M1[1] - EM1[1]);
            em1_coeffs->values.push_back (0.0);

            if ( growing_visualization )
            {
              viewer.addLine (*em1_coeffs, 1.0, 0.0, 0.5, "EM1_COEFFS");
              viewer.spin ();
              viewer.removeShape ("EM1_COEFFS");
            }

            //

            pcl::PointIndices::Ptr em1_inliers (new pcl::PointIndices ());

            for (int idx = 0; idx < working_cloud->points.size (); idx++)
            {
              double P0[2];
              P0[0] = working_cloud->points.at (idx).x;
              P0[1] = working_cloud->points.at (idx).y;

              double num = (P0[0] - EM1[0])*(M1[0] - EM1[0]) + (P0[1] - EM1[1])*(M1[1] - EM1[1]);
              double den = _sqr (M1[0] - EM1[0])  +  _sqr (M1[1] - EM1[1]);
              double u = num / den;

              double I[2];
              I[0] = EM1[0] + u * (M1[0] - EM1[0]);
              I[1] = EM1[1] + u * (M1[1] - EM1[1]);

              double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

              if ( d < line_threshold )
              {
                double IEM1D  = sqrt ( _sqr  (I[0] - EM1[0])  +  _sqr  (I[1] - EM1[1]) );
                double IM1D   = sqrt ( _sqr  (I[0] -  M1[0])  +  _sqr  (I[1] -  M1[1]) );
                double EM1M1D = sqrt ( _sqr (M1[0] - EM1[0])  +  _sqr (M1[1] - EM1[1]) );

                if ( (IEM1D + IM1D) < (EM1M1D + 0.001) )
                  em1_inliers->indices.push_back (idx);
              }
            }

            //

            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr em1_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

            pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> em1_ei;
            em1_ei.setInputCloud (working_cloud);
            em1_ei.setIndices (em1_inliers);
            em1_ei.setNegative (false);
            em1_ei.filter (*em1_cloud);

            if ( growing_visualization )
            {
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> em1_cloud_color (em1_cloud, 255, 0, 127);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (em1_cloud, em1_cloud_color, "EM1_CLOUD");
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EM1_CLOUD");
              viewer.spin ();
              viewer.removePointCloud ("EM1_CLOUD");
            }

            //

            pcl::PointXYZRGBNormalRSD em1_MiN, em1_MaX;
            pcl::getMinMax3D (*em1_cloud, em1_MiN, em1_MaX);

            cerr << endl;
            cerr << "          MaX = " << MaX.z << endl;
            cerr << "      em1_MaX = " << em1_MaX.z << endl;
            cerr << "      em1_dif = " << fabs (MaX.z - em1_MaX.z) << endl;
            cerr << endl;

            double em1_dif = fabs (MaX.z - em1_MaX.z);

            if ( em1_dif < growing_height )
            {
              cerr << "      OK !" << endl ;

              M1[0] = EM1[0];
              M1[1] = EM1[1];

              //pcl::copyPointCloud (*em1_cloud, *novel_box_cloud);
              novel_box_cloud->points.insert (  novel_box_cloud->points.end(),        em1_cloud->points.begin(),         em1_cloud->points.end());
              novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     em1_inliers->indices.begin(),     em1_inliers->indices.end());
            }
            else
            {
              cerr << "      NOT OK !" << endl ;

              em1_bool = false;
            }

          } while ( em1_bool );

          cerr << endl << "   EXIT !" << endl << endl ;

          if ( growing_visualization )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
            viewer.spin ();
            viewer.removePointCloud ("NOVEL_BOX_CLOUD");
          }

          //////////////////
          // From M To P2 //
          //////////////////

          bool em2_bool = true;

          double M2[2];
          M2[0] = M[0];
          M2[1] = M[1];

          do
          {

            double P1M2V[2];
            P1M2V[0] = M2[0] - P1[0];
            P1M2V[1] = M2[1] - P1[1];

            double P1M2N = sqrt (P1M2V[0]*P1M2V[0] + P1M2V[1]*P1M2V[1]);
            P1M2V[0] = P1M2V[0] / P1M2N;
            P1M2V[1] = P1M2V[1] / P1M2N;

            double EM2[2];
            EM2[0] = M2[0] + P1M2V[0] * growing_step;
            EM2[1] = M2[1] + P1M2V[1] * growing_step;

            pcl::ModelCoefficients::Ptr em2_coeffs (new pcl::ModelCoefficients ());
            em2_coeffs->values.push_back (EM2[0]);
            em2_coeffs->values.push_back (EM2[1]);
            em2_coeffs->values.push_back (0.0);
            em2_coeffs->values.push_back (M2[0] - EM2[0]);
            em2_coeffs->values.push_back (M2[1] - EM2[1]);
            em2_coeffs->values.push_back (0.0);

            if ( growing_visualization )
            {
              viewer.addLine (*em2_coeffs, 1.0, 0.0, 0.5, "EM2_COEFFS");
              viewer.spin ();
              viewer.removeShape ("EM2_COEFFS");
            }

            //

            pcl::PointIndices::Ptr em2_inliers (new pcl::PointIndices ());

            for (int idx = 0; idx < working_cloud->points.size (); idx++)
            {
              double P0[2];
              P0[0] = working_cloud->points.at (idx).x;
              P0[1] = working_cloud->points.at (idx).y;

              double num = (P0[0] - EM2[0])*(M2[0] - EM2[0]) + (P0[1] - EM2[1])*(M2[1] - EM2[1]);
              double den = _sqr (M2[0] - EM2[0])  +  _sqr (M2[1] - EM2[1]);
              double u = num / den;

              double I[2];
              I[0] = EM2[0] + u * (M2[0] - EM2[0]);
              I[1] = EM2[1] + u * (M2[1] - EM2[1]);

              double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

              if ( d < line_threshold )
              {
                double IEM2D  = sqrt ( _sqr  (I[0] - EM2[0])  +  _sqr  (I[1] - EM2[1]) );
                double IM2D   = sqrt ( _sqr  (I[0] -  M2[0])  +  _sqr  (I[1] -  M2[1]) );
                double EM2M2D = sqrt ( _sqr (M2[0] - EM2[0])  +  _sqr (M2[1] - EM2[1]) );

                if ( (IEM2D + IM2D) < (EM2M2D + 0.001) )
                  em2_inliers->indices.push_back (idx);
              }
            }

            //

            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr em2_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

            pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> em2_ei;
            em2_ei.setInputCloud (working_cloud);
            em2_ei.setIndices (em2_inliers);
            em2_ei.setNegative (false);
            em2_ei.filter (*em2_cloud);

            if ( growing_visualization )
            {
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> em2_cloud_color (em2_cloud, 255, 0, 127);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (em2_cloud, em2_cloud_color, "EM2_CLOUD");
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EM2_CLOUD");
              viewer.spin ();
              viewer.removePointCloud ("EM2_CLOUD");
            }

            //

            pcl::PointXYZRGBNormalRSD em2_MiN, em2_MaX;
            pcl::getMinMax3D (*em2_cloud, em2_MiN, em2_MaX);

            cerr << endl;
            cerr << "          MaX = " << MaX.z << endl;
            cerr << "      em2_MaX = " << em2_MaX.z << endl;
            cerr << "      em2_dif = " << fabs (MaX.z - em2_MaX.z) << endl;
            cerr << endl;

            double em2_dif = fabs (MaX.z - em2_MaX.z);

            if ( em2_dif < growing_height )
            {
              cerr << "      OK !" << endl ;

              M2[0] = EM2[0];
              M2[1] = EM2[1];

              //pcl::copyPointCloud (*em2_cloud, *novel_box_cloud);
              novel_box_cloud->points.insert (  novel_box_cloud->points.end(),        em2_cloud->points.begin(),         em2_cloud->points.end());
              novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     em2_inliers->indices.begin(),     em2_inliers->indices.end());
            }
            else
            {
              cerr << "      NOT OK !" << endl ;

              em2_bool = false;
            }

          } while ( em2_bool );

          cerr << endl << "   EXIT !" << endl << endl ;

          if ( growing_visualization )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
            viewer.spin ();
            viewer.removePointCloud ("NOVEL_BOX_CLOUD");
          }

          /*
             pcl::PointIndices::Ptr g_vransac_line_inliers (new pcl::PointIndices ());
           *g_vransac_line_inliers = *vransac_line_inliers;

           float R = 0.010;
           std::vector<int> pointIdxRadiusSearch;
           std::vector<float> pointRadiusSquaredDistance;

           pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD>::Ptr g_tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD> ());
           g_tree->setInputCloud (working_cloud);

           std::vector<bool> g_visited_points (working_cloud->points.size ());
           for (int vis = 0; vis < g_visited_points.size (); vis++)
           g_visited_points.at (vis) = false;

           size_t gidx = 0;

           do
           {
           pcl::PointXYZRGBNormalRSD searchPoint;
           searchPoint = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx));

           if ( g_visited_points.at (g_vransac_line_inliers->indices.at (gidx)) == false )
           {
           g_visited_points.at (g_vransac_line_inliers->indices.at (gidx)) = true;

           double P0[2];
           P0[0] = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx)).x;
           P0[1] = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx)).y;

           double num = (P0[0] - P1[0])*(P2[0] - P1[0]) + (P0[1] - P1[1])*(P2[1] - P1[1]);
           double den = _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]);
           double u = num / den;

           double I[2];
           I[0] = P1[0] + u * (P2[0] - P1[0]);
           I[1] = P1[1] + u * (P2[1] - P1[1]);

           double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

           if ( d < line_threshold )
           {
           if ( g_tree->radiusSearch (searchPoint, R, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
           {
           for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
           g_vransac_line_inliers->indices.push_back (pointIdxRadiusSearch.at (i));
           }
           }
           }

           gidx++;

           } while ( gidx < g_vransac_line_inliers->indices.size () );

           *vransac_line_inliers = *g_vransac_line_inliers;

           cerr << " vransac_line_inliers = " << vransac_line_inliers->indices.size () << endl;

           sort (g_vransac_line_inliers->indices.begin(), g_vransac_line_inliers->indices.end());
           g_vransac_line_inliers->indices.erase (unique (g_vransac_line_inliers->indices.begin(), g_vransac_line_inliers->indices.end()), g_vransac_line_inliers->indices.end());

           pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr g_vransac_line_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

           pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> g_l_ei;
           g_l_ei.setInputCloud (working_cloud);
           g_l_ei.setIndices (g_vransac_line_inliers);
           g_l_ei.setNegative (false);
           g_l_ei.filter (*g_vransac_line_cloud);

          // Adjust Coefficients Of Line Model //
          adjustLineCoefficients (g_vransac_line_cloud, vransac_line_coefficients);

          if ( space_step )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> g_vransac_line_cloud_color (g_vransac_line_cloud, 0, 255, 255);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (g_vransac_line_cloud, g_vransac_line_cloud_color, "G_VRANSAC_LINE_CLOUD");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "G_VRANSAC_LINE_CLOUD");
            viewer.addLine (*vransac_line_coefficients, 0.0, 1.0, 1.0, "VRANSAC_LINE_MODEL");
            viewer.spin ();
            viewer.removePointCloud ("G_VRANSAC_LINE_CLOUD");
            viewer.removeShape ("VRANSAC_LINE_MODEL");
          }
          */

            // Obtain The Scanning Viewpoint //

            double O[3];
          O[0] = 0.0;
          O[1] = 0.0;
          O[2] = 0.0;

          double VP[3];
          double T[4][4];

          rX = DEG2RAD (rX); rY = DEG2RAD (rY); rZ = DEG2RAD (rZ);
          computeTransformationMatrix (tX, tY, tZ, rX, rY, rZ, T);
          transform (T, O, VP);

          if ( !till_the_end )
            if ( true )
            {
              pcl::PointXYZ CVP;
              CVP.x = VP[0];
              CVP.y = VP[1];
              CVP.z = VP[2];

              viewer.addSphere<pcl::PointXYZ> (CVP, 0.010, 1.0, 1.0, 0.0, "VIEWPOINT_SPHERE");
              viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "VIEWPOINT_SPHERE");
              viewer.spin ();
              viewer.removeShape ("VIEWPOINT_SPHERE");
            }

          // Determine The Normal To Line //

          double dx = P2[0] - P1[0];
          double dy = P2[1] - P1[1];

          double N1[2];
          N1[0] = -dy;
          N1[1] =  dx;

          double PN1[2];
          PN1[0] =  ((P1[0] + P2[0]) / 2) + N1[0]*0.25;
          PN1[1] =  ((P1[1] + P2[1]) / 2) + N1[1]*0.25;

          double N2[2];
          N2[0] =  dy;
          N2[1] = -dx;

          double PN2[2];
          PN2[0] =  ((P1[0] + P2[0]) / 2) + N2[0]*0.25;
          PN2[1] =  ((P1[1] + P2[1]) / 2) + N2[1]*0.25;

          if ( false )
          {
            pcl::ModelCoefficients::Ptr first_normal_coefficients (new pcl::ModelCoefficients ());
            first_normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
            first_normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
            first_normal_coefficients->values.push_back (0.0);
            first_normal_coefficients->values.push_back (PN1[0] - ((P1[0] + P2[0]) / 2));
            first_normal_coefficients->values.push_back (PN1[1] - ((P1[1] + P2[1]) / 2));
            first_normal_coefficients->values.push_back (0.0);

            pcl::ModelCoefficients::Ptr second_normal_coefficients (new pcl::ModelCoefficients ());
            second_normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
            second_normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
            second_normal_coefficients->values.push_back (0.0);
            second_normal_coefficients->values.push_back (PN2[0] - ((P1[0] + P2[0]) / 2));
            second_normal_coefficients->values.push_back (PN2[1] - ((P1[1] + P2[1]) / 2));
            second_normal_coefficients->values.push_back (0.0);

            viewer.addLine (*first_normal_coefficients, 1.0, 0.0, 1.0, "FIRST_NORMAL_COEFFICIENTS");
            viewer.addLine (*second_normal_coefficients, 1.0, 0.0, 1.0, "SECOND_NORMAL_COEFFICIENTS");
            viewer.spin ();
            viewer.removeShape ("FIRST_NORMAL_COEFFICIENTS");
            viewer.removeShape ("SECOND_NORMAL_COEFFICIENTS");
          }

          double N[2];

          double PN1VP = sqrt ( _sqr (PN1[0] - VP[0]) + _sqr (PN1[1] - VP[1]) );
          double PN2VP = sqrt ( _sqr (PN2[0] - VP[0]) + _sqr (PN2[1] - VP[1]) );

          if ( PN1VP < PN2VP )
          {
            N[0] = N2[0];
            N[1] = N2[1];
          }
          else
          {
            N[0] = N1[0];
            N[1] = N1[1];
          }

          if ( !till_the_end )
            if ( true )
            {
              pcl::ModelCoefficients::Ptr normal_coefficients (new pcl::ModelCoefficients ());
              normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
              normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
              normal_coefficients->values.push_back (0.0);
              normal_coefficients->values.push_back (N[0]);
              normal_coefficients->values.push_back (N[1]);
              normal_coefficients->values.push_back (0.0);

              viewer.addLine (*normal_coefficients, 1.0, 0.0, 1.0, "NORMAL_COEFFICIENTS");
              viewer.spin ();
              viewer.removeShape ("NORMAL_COEFFICIENTS");
            }

          double NN = sqrt ( N[0]*N[0] + N[1]*N[1] );
          N[0] = N[0] / NN;
          N[1] = N[1] / NN;

          // NOVEL 2 WAY OF GROWING BOXES //

          double NP1[2];
          double NP2[2];

          /////////////////
          // To The Back //
          /////////////////

          bool en1_bool = true;

          NP1[0] = M1[0];
          NP1[1] = M1[1];

          NP2[0] = M2[0];
          NP2[1] = M2[1];

          do
          {
            double EN1[2];
            EN1[0] = NP1[0] + N[0] * growing_step /2;
            EN1[1] = NP1[1] + N[1] * growing_step /2;

            double EN2[2];
            EN2[0] = NP2[0] + N[0] * growing_step /2;
            EN2[1] = NP2[1] + N[1] * growing_step /2;

            pcl::ModelCoefficients::Ptr en1_coeffs (new pcl::ModelCoefficients ());
            en1_coeffs->values.push_back (EN1[0]);
            en1_coeffs->values.push_back (EN1[1]);
            en1_coeffs->values.push_back (0.0);
            en1_coeffs->values.push_back (EN2[0] - EN1[0]);
            en1_coeffs->values.push_back (EN2[1] - EN1[1]);
            en1_coeffs->values.push_back (0.0);

            if ( growing_visualization )
            {
              viewer.addLine (*en1_coeffs, 1.0, 0.0, 0.5, "EN_COEFFS");
              viewer.spin ();
              viewer.removeShape ("EN_COEFFS");
            }

            //

            pcl::PointIndices::Ptr en1_inliers (new pcl::PointIndices ());

            for (int idx = 0; idx < working_cloud->points.size (); idx++)
            {
              double P0[2];
              P0[0] = working_cloud->points.at (idx).x;
              P0[1] = working_cloud->points.at (idx).y;

              double num = (P0[0] - EN1[0])*(EN2[0] - EN1[0]) + (P0[1] - EN1[1])*(EN2[1] - EN1[1]);
              double den = _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]);
              double u = num / den;

              double I[2];
              I[0] = EN1[0] + u * (EN2[0] - EN1[0]);
              I[1] = EN1[1] + u * (EN2[1] - EN1[1]);

              double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

              if ( d < growing_step )
              {
                double IEN1D  = sqrt ( _sqr  (I[0] - EN1[0])  +  _sqr  (I[1] - EN1[1]) );
                double IEN2D  = sqrt ( _sqr  (I[0] - EN2[0])  +  _sqr  (I[1] - EN2[1]) );
                double EN1EN2D = sqrt ( _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]) );

                if ( (IEN1D + IEN2D) < (EN1EN2D + 0.001) )
                  en1_inliers->indices.push_back (idx);
              }
            }

            //

            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr en1_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

            pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> en1_ei;
            en1_ei.setInputCloud (working_cloud);
            en1_ei.setIndices (en1_inliers);
            en1_ei.setNegative (false);
            en1_ei.filter (*en1_cloud);

            if ( growing_visualization )
            {
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> en1_cloud_color (en1_cloud, 255, 0, 127);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (en1_cloud, en1_cloud_color, "EN_CLOUD");
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EN_CLOUD");
              viewer.spin ();
              viewer.removePointCloud ("EN_CLOUD");
            }

            //

            pcl::PointXYZRGBNormalRSD en1_MiN, en1_MaX;
            pcl::getMinMax3D (*en1_cloud, en1_MiN, en1_MaX);

            cerr << endl;
            cerr << "          MaX = " << MaX.z << endl;
            cerr << "      en1_MaX = " << en1_MaX.z << endl;
            cerr << "      en1_dif = " << fabs (MaX.z - en1_MaX.z) << endl;
            cerr << endl;

            double en1_dif = fabs (MaX.z - en1_MaX.z);

            if ( en1_dif < growing_height )
            {
              cerr << "      OK !" << endl ;

              NP1[0] = EN1[0];
              NP1[1] = EN1[1];

              NP2[0] = EN2[0];
              NP2[1] = EN2[1];

              //pcl::copyPointCloud (*en1_cloud, *novel_box_cloud);
              novel_box_cloud->points.insert (   novel_box_cloud->points.end(),        en1_cloud->points.begin(),        en1_cloud->points.end());
              novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     en1_inliers->indices.begin(),     en1_inliers->indices.end());
            }
            else
            {
              cerr << "      NOT OK !" << endl ;

              en1_bool = false;
            }

          } while ( en1_bool );

          cerr << endl << "   EXIT !" << endl << endl ;

          if ( growing_visualization )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
            viewer.spin ();
            viewer.removePointCloud ("NOVEL_BOX_CLOUD");
          }

          //////////////////
          // To The Front //
          //////////////////

          bool en2_bool = true;

          NP1[0] = M1[0];
          NP1[1] = M1[1];

          NP2[0] = M2[0];
          NP2[1] = M2[1];

          do
          {
            double EN1[2];
            EN1[0] = NP1[0] - N[0] * growing_step /2;
            EN1[1] = NP1[1] - N[1] * growing_step /2;

            double EN2[2];
            EN2[0] = NP2[0] - N[0] * growing_step /2;
            EN2[1] = NP2[1] - N[1] * growing_step /2;

            pcl::ModelCoefficients::Ptr en2_coeffs (new pcl::ModelCoefficients ());
            en2_coeffs->values.push_back (EN1[0]);
            en2_coeffs->values.push_back (EN1[1]);
            en2_coeffs->values.push_back (0.0);
            en2_coeffs->values.push_back (EN2[0] - EN1[0]);
            en2_coeffs->values.push_back (EN2[1] - EN1[1]);
            en2_coeffs->values.push_back (0.0);

            if ( growing_visualization )
            {
              viewer.addLine (*en2_coeffs, 1.0, 0.0, 0.5, "EN_COEFFS");
              viewer.spin ();
              viewer.removeShape ("EN_COEFFS");
            }

            //

            pcl::PointIndices::Ptr en2_inliers (new pcl::PointIndices ());

            for (int idx = 0; idx < working_cloud->points.size (); idx++)
            {
              double P0[2];
              P0[0] = working_cloud->points.at (idx).x;
              P0[1] = working_cloud->points.at (idx).y;

              double num = (P0[0] - EN1[0])*(EN2[0] - EN1[0]) + (P0[1] - EN1[1])*(EN2[1] - EN1[1]);
              double den = _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]);
              double u = num / den;

              double I[2];
              I[0] = EN1[0] + u * (EN2[0] - EN1[0]);
              I[1] = EN1[1] + u * (EN2[1] - EN1[1]);

              double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

              if ( d < growing_step )
              {
                double IEN1D  = sqrt ( _sqr  (I[0] - EN1[0])  +  _sqr  (I[1] - EN1[1]) );
                double IEN2D  = sqrt ( _sqr  (I[0] - EN2[0])  +  _sqr  (I[1] - EN2[1]) );
                double EN1EN2D = sqrt ( _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]) );

                if ( (IEN1D + IEN2D) < (EN1EN2D + 0.001) )
                  en2_inliers->indices.push_back (idx);
              }
            }

            //

            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr en2_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

            pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> en2_ei;
            en2_ei.setInputCloud (working_cloud);
            en2_ei.setIndices (en2_inliers);
            en2_ei.setNegative (false);
            en2_ei.filter (*en2_cloud);

            if ( growing_visualization )
            {
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> en2_cloud_color (en2_cloud, 255, 0, 127);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (en2_cloud, en2_cloud_color, "EN_CLOUD");
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EN_CLOUD");
              viewer.spin ();
              viewer.removePointCloud ("EN_CLOUD");
            }

            //

            pcl::PointXYZRGBNormalRSD en2_MiN, en2_MaX;
            pcl::getMinMax3D (*en2_cloud, en2_MiN, en2_MaX);

            cerr << endl;
            cerr << "          MaX = " << MaX.z << endl;
            cerr << "      en2_MaX = " << en2_MaX.z << endl;
            cerr << "      en2_dif = " << fabs (MaX.z - en2_MaX.z) << endl;
            cerr << endl;

            double en2_dif = fabs (MaX.z - en2_MaX.z);

            if ( en2_dif < growing_height )
            {
              cerr << "      OK !" << endl ;

              NP1[0] = EN1[0];
              NP1[1] = EN1[1];

              NP2[0] = EN2[0];
              NP2[1] = EN2[1];

              //pcl::copyPointCloud (*en2_cloud, *novel_box_cloud);
              novel_box_cloud->points.insert (   novel_box_cloud->points.end(),        en2_cloud->points.begin(),        en2_cloud->points.end());
              novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     en2_inliers->indices.begin(),     en2_inliers->indices.end());
            }
            else
            {
              cerr << "      NOT OK !" << endl ;

              en2_bool = false;
            }

          } while ( en2_bool );

          cerr << endl << "   EXIT !" << endl << endl ;

          if ( growing_visualization )
          {
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
            viewer.spin ();
            viewer.removePointCloud ("NOVEL_BOX_CLOUD");
          }



          /*

          // Obtain Points Of Box //

          pcl::PointIndices::Ptr box_inliers (new pcl::PointIndices ());
           *box_inliers = *vransac_line_inliers;

           int K = 10;
           std::vector<int> pointIdxNKNSearch(K);
           std::vector<float> pointNKNSquaredDistance(K);

           pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD>::Ptr box_tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD> ());
           box_tree->setInputCloud (working_cloud);

           std::vector<bool> visited_points (working_cloud->points.size ());
           for (int vis = 0; vis < visited_points.size (); vis++)
           visited_points.at (vis) = false;

           size_t idx = 0;

           pcl::PointXYZRGBNormalRSD min_of_vlc, max_of_vlc;
           pcl::getMinMax3D (*vransac_line_cloud, min_of_vlc, max_of_vlc);

           do
           {
           pcl::PointXYZRGBNormalRSD searchPoint;
           searchPoint = working_cloud->points.at (box_inliers->indices.at (idx));

           if ( visited_points.at (box_inliers->indices.at (idx)) == false )
           {
           visited_points.at (box_inliers->indices.at (idx)) = true;

           double P0[2];
           P0[0] = working_cloud->points.at (box_inliers->indices.at (idx)).x;
           P0[1] = working_cloud->points.at (box_inliers->indices.at (idx)).y;
           P0[2] = working_cloud->points.at (box_inliers->indices.at (idx)).z;

           if ( ( P0[2] < (max_of_vlc.z + 0.005) ) && ( P0[2] > (min_of_vlc.z - 0.005) ) )
           {
           double num = (P0[0] - P1[0])*(P2[0] - P1[0]) + (P0[1] - P1[1])*(P2[1] - P1[1]);
           double den = _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]);
           double u = num / den;

           double I[2];
           I[0] = P1[0] + u * (P2[0] - P1[0]);
           I[1] = P1[1] + u * (P2[1] - P1[1]);

           double  IP1D = sqrt ( _sqr  (I[0] - P1[0])  +  _sqr  (I[1] - P1[1]) );
           double  IP2D = sqrt ( _sqr  (I[0] - P2[0])  +  _sqr  (I[1] - P2[1]) );
           double P1P2D = sqrt ( _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]) );

           if ( (IP1D + IP2D) < (P1P2D + 0.001) )
           {
           double  IVPD  = sqrt ( _sqr   (I[0] - VP[0])  +  _sqr   (I[1] - VP[1]) );
           double P0VPD  = sqrt ( _sqr  (P0[0] - VP[0])  +  _sqr  (P0[1] - VP[1]) );

           if ( P0VPD > IVPD )
           {
           if ( box_tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
           {
           for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
           box_inliers->indices.push_back (pointIdxNKNSearch.at (i));
           }
           }
           }
           }
           }

           idx++;

           } while ( idx < box_inliers->indices.size() );

          sort (box_inliers->indices.begin(), box_inliers->indices.end());
          box_inliers->indices.erase (unique (box_inliers->indices.begin(), box_inliers->indices.end()), box_inliers->indices.end());

          */

            pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr box_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

          pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> b_ei;
          b_ei.setInputCloud (working_cloud);
          b_ei.setIndices (novel_box_inliers);

          b_ei.setNegative (false);
          b_ei.filter (*box_cloud);

          b_ei.setNegative (true);
          b_ei.filter (*working_cloud);

          //

          if ( working_cloud->points.size () > 0 )
          {
            pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormalRSD> sor;
            sor.setInputCloud (working_cloud);
            sor.setMeanK (mean_k_filter);
            sor.setStddevMulThresh (std_dev_filter *5);
            sor.filter (*working_cloud);
          }

          //

          //////if ( space_step )
          //////{
          //////std::stringstream box_cloud_id;
          //////box_cloud_id << "BOX_CLOUD_" << getTimestamp ();
          //////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> box_cloud_color (box_cloud, 127, 0, 255);
          //////viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (box_cloud, box_cloud_color, box_cloud_id.str ());
          //////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, box_cloud_id.str ());
          //////viewer.spin ();
          //////
          //////viewer.removePointCloud ("generic");
          //////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
          //////viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "generic");
          //////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
          //////viewer.spin ();
          //////}

          // ADD BOX SHAPE //

          double p1[3];
          p1[0] = M1[0];
          p1[1] = M1[1];
          p1[2] = 0.0;

          double p2[3];
          p2[0] = M2[0];
          p2[1] = M2[1];
          p2[2] = 0.0;

          /// Vector of Line ///
          double l[3];
          l[0] = p2[0] - p1[0];
          l[1] = p2[1] - p1[1];
          l[2] = 0.0;

          /// Normalize Vector of Line ///
          double nl = sqrt ( _sqr(l[0]) + _sqr(l[1]) + _sqr(l[2]) );
          l[0] = l[0] / nl;
          l[1] = l[1] / nl;
          l[2] = l[2] / nl;

          /// Unit Vector of Z Axis ///
          double n[3];
          n[0] = 0.0;
          n[1] = 0.0;
          n[2] = 1.0;

          /// Dot Product ///
          double m[3];
          m[0] = l[1]*n[2] - l[2]*n[1];
          m[1] = l[2]*n[0] - l[0]*n[2];
          m[2] = l[0]*n[1] - l[1]*n[0];

          // Length- and Cross- Vectors //
          double vectors[2][2];
          vectors[0][0] = l[0];
          vectors[0][1] = l[1];
          vectors[1][0] = m[0];
          vectors[1][1] = m[1];

          Eigen::Vector4f centroid;
          pcl::compute3DCentroid (*box_cloud, centroid);

          double max_u = -DBL_MAX;
          double min_u =  DBL_MAX;
          double max_v = -DBL_MAX;
          double min_v =  DBL_MAX;

          /*

          // Bounding box only for line inliers //

          for ( int inl = 0; inl < (int) shapes_lines_inliers.at (clu)->indices.size(); inl++ )
          {
          int idx = shapes_lines_inliers.at (clu)->indices.at (inl);

          double c2p[2];
          c2p[0] = shapes_lines_clusters.at (clu)->points.at (idx).x - centroid[0];
          c2p[1] = shapes_lines_clusters.at (clu)->points.at (idx).y - centroid[1];

          double width = vectors[0][0]*c2p[0] + vectors[0][1]*c2p[1];
          if (width > max_u) max_u = width;
          if (width < min_u) min_u = width;

          double length = vectors[1][0]*c2p[0] + vectors[1][1]*c2p[1];
          if (length > max_v) max_v = length;
          if (length < min_v) min_v = length;
          }

*/

          // Bounding box for whole cluster //

          for ( int idx = 0; idx < (int) box_cloud->points.size(); idx++ )
          {
            double c2p[2];
            c2p[0] = box_cloud->points.at (idx).x - centroid[0];
            c2p[1] = box_cloud->points.at (idx).y - centroid[1];

            double width = vectors[0][0]*c2p[0] + vectors[0][1]*c2p[1];
            if (width > max_u) max_u = width;
            if (width < min_u) min_u = width;

            double length = vectors[1][0]*c2p[0] + vectors[1][1]*c2p[1];
            if (length > max_v) max_v = length;
            if (length < min_v) min_v = length;
          }

          double minimus = +DBL_MAX;
          double maximus = -DBL_MAX;

          /*

          // Bounding box only for line inliers //

          for ( int inl = 0; inl < (int) shapes_lines_inliers.at (clu)->indices.size(); inl++ )
          {
          int idx = shapes_lines_inliers.at (clu)->indices.at (inl);

          double Z = shapes_lines_clusters.at (clu)->points.at (idx).z;

          if ( minimus > Z ) minimus = Z;
          if ( maximus < Z ) maximus = Z;
          }

*/

          // Bounding box for whole cluster //

          for ( int idx = 0; idx < (int) box_cloud->points.size(); idx++ )
          {
            double Z = box_cloud->points.at (idx).z;

            if ( minimus > Z ) minimus = Z;
            if ( maximus < Z ) maximus = Z;
          }

          // The edges //

          double edges[4][2];

          edges[0][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*max_v;
          edges[0][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*max_v;

          edges[1][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*min_v;
          edges[1][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*min_v;

          edges[2][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*min_v;
          edges[2][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*min_v;

          edges[3][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*max_v;
          edges[3][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*max_v;

          //////    minimus = working_cloud_minimum.z;

          if ( shapes_from_table_plane )
            minimus = abs_min.z;

          pcl::ModelCoefficients e0, e1, e2, e3;

          e0.values.push_back (edges[0][0]);
          e0.values.push_back (edges[0][1]);
          e0.values.push_back (minimus);
          e0.values.push_back (edges[1][0] - edges[0][0]);
          e0.values.push_back (edges[1][1] - edges[0][1]);
          e0.values.push_back (minimus - minimus);

          e1.values.push_back (edges[1][0]);
          e1.values.push_back (edges[1][1]);
          e1.values.push_back (minimus);
          e1.values.push_back (edges[2][0] - edges[1][0]);
          e1.values.push_back (edges[2][1] - edges[1][1]);
          e1.values.push_back (minimus - minimus);

          e2.values.push_back (edges[2][0]);
          e2.values.push_back (edges[2][1]);
          e2.values.push_back (minimus);
          e2.values.push_back (edges[3][0] - edges[2][0]);
          e2.values.push_back (edges[3][1] - edges[2][1]);
          e2.values.push_back (minimus - minimus);

          e3.values.push_back (edges[3][0]);
          e3.values.push_back (edges[3][1]);
          e3.values.push_back (minimus);
          e3.values.push_back (edges[0][0] - edges[3][0]);
          e3.values.push_back (edges[0][1] - edges[3][1]);
          e3.values.push_back (minimus - minimus);

          /*

             std::stringstream line_0;
             line_0 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e0);
          line_viewer.addLine (e0, line_0.str ());
          line_viewer.spin ();

          std::stringstream line_1;
          line_1 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e1);
          line_viewer.addLine (e1, line_1.str ());
          line_viewer.spin ();

          std::stringstream line_2;
          line_2 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e2);
          line_viewer.addLine (e2, line_2.str ());
          line_viewer.spin ();

          std::stringstream line_3;
          line_3 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e3);
          line_viewer.addLine (e3, line_3.str ());
          line_viewer.spin ();

*/

          pcl::ModelCoefficients e4, e5, e6, e7;

          e4.values.push_back (edges[0][0]);
          e4.values.push_back (edges[0][1]);
          e4.values.push_back (maximus);
          e4.values.push_back (edges[1][0] - edges[0][0]);
          e4.values.push_back (edges[1][1] - edges[0][1]);
          e4.values.push_back (maximus - maximus);

          e5.values.push_back (edges[1][0]);
          e5.values.push_back (edges[1][1]);
          e5.values.push_back (maximus);
          e5.values.push_back (edges[2][0] - edges[1][0]);
          e5.values.push_back (edges[2][1] - edges[1][1]);
          e5.values.push_back (maximus - maximus);

          e6.values.push_back (edges[2][0]);
          e6.values.push_back (edges[2][1]);
          e6.values.push_back (maximus);
          e6.values.push_back (edges[3][0] - edges[2][0]);
          e6.values.push_back (edges[3][1] - edges[2][1]);
          e6.values.push_back (maximus - maximus);

          e7.values.push_back (edges[3][0]);
          e7.values.push_back (edges[3][1]);
          e7.values.push_back (maximus);
          e7.values.push_back (edges[0][0] - edges[3][0]);
          e7.values.push_back (edges[0][1] - edges[3][1]);
          e7.values.push_back (maximus - maximus);

          /*

             std::stringstream line_4;
             line_4 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e4);
          line_viewer.addLine (e4, line_4.str ());
          line_viewer.spin ();

          std::stringstream line_5;
          line_5 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e5);
          line_viewer.addLine (e5, line_5.str ());
          line_viewer.spin ();

          std::stringstream line_6;
          line_6 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e6);
          line_viewer.addLine (e6, line_6.str ());
          line_viewer.spin ();

          std::stringstream line_7;
          line_7 << "LINE_" << ros::Time::now();
          //adjustLine (shapes_lines_clusters.at (clu), e7);
          line_viewer.addLine (e7, line_7.str ());
          line_viewer.spin ();

*/

          std::vector<pcl::ModelCoefficients> pla;

          pla.push_back (e0);
          pla.push_back (e1);
          pla.push_back (e2);
          pla.push_back (e3);
          pla.push_back (e4);
          pla.push_back (e5);
          pla.push_back (e6);
          pla.push_back (e7);

          cerr << e0.values.at(0) << " " << e0.values.at(1) << " " << e0.values.at(2) << " " << e0.values.at(3) << " " << e0.values.at(4) << " " << e0.values.at(5) << endl ;
          cerr << e1.values.at(0) << " " << e1.values.at(1) << " " << e1.values.at(2) << " " << e1.values.at(3) << " " << e1.values.at(4) << " " << e1.values.at(5) << endl ;
          cerr << e2.values.at(0) << " " << e2.values.at(1) << " " << e2.values.at(2) << " " << e2.values.at(3) << " " << e2.values.at(4) << " " << e2.values.at(5) << endl ;
          cerr << e3.values.at(0) << " " << e3.values.at(1) << " " << e3.values.at(2) << " " << e3.values.at(3) << " " << e3.values.at(4) << " " << e3.values.at(5) << endl ;
          cerr << e4.values.at(0) << " " << e4.values.at(1) << " " << e4.values.at(2) << " " << e4.values.at(3) << " " << e4.values.at(4) << " " << e4.values.at(5) << endl ;
          cerr << e5.values.at(0) << " " << e5.values.at(1) << " " << e5.values.at(2) << " " << e5.values.at(3) << " " << e5.values.at(4) << " " << e5.values.at(5) << endl ;
          cerr << e6.values.at(0) << " " << e6.values.at(1) << " " << e6.values.at(2) << " " << e6.values.at(3) << " " << e6.values.at(4) << " " << e6.values.at(5) << endl ;
          cerr << e7.values.at(0) << " " << e7.values.at(1) << " " << e7.values.at(2) << " " << e7.values.at(3) << " " << e7.values.at(4) << " " << e7.values.at(5) << endl ;

          // NOVEL 3 WAY OF GROWING BOXES //

          //////std::stringstream cub_id;
          //////cub_id << "CUB_" << getTimestamp ();
          //////viewer.addCuboid (pla, 0.5, 0.0, 1.0, 0.5, cub_id.str ());

          model++;
          cerr << endl << " MODEL " << model << endl << endl ;

          if ( space_step ) viewer.spin ();

            //                //
           // Classification //
          //                //

          if ( classification )
          {

            //double x_surface = sqrt (_sqr (e0.values[0]-e1.values[0]) + _sqr (e0.values[1]-e1.values[1]) + _sqr (e0.values[2]-e1.values[2])) * sqrt (_sqr (e1.values[0]-e5.values[0]) + _sqr (e1.values[1]-e5.values[1]) + _sqr (e1.values[2]-e5.values[2])) ;
            //double y_surface = sqrt (_sqr (e1.values[0]-e2.values[0]) + _sqr (e1.values[1]-e2.values[1]) + _sqr (e1.values[2]-e2.values[2])) * sqrt (_sqr (e2.values[0]-e6.values[0]) + _sqr (e2.values[1]-e6.values[1]) + _sqr (e2.values[2]-e6.values[2])) ;
            //double z_surface = sqrt (_sqr (e0.values[0]-e1.values[0]) + _sqr (e0.values[1]-e1.values[1]) + _sqr (e0.values[2]-e1.values[2])) * sqrt (_sqr (e1.values[0]-e2.values[0]) + _sqr (e1.values[1]-e2.values[1]) + _sqr (e1.values[2]-e2.values[2])) ;

            double x_dist = sqrt (_sqr (e0.values[0]-e1.values[0]) + _sqr (e0.values[1]-e1.values[1]) + _sqr (e0.values[2]-e1.values[2])) ;
            double y_dist = sqrt (_sqr (e1.values[0]-e2.values[0]) + _sqr (e1.values[1]-e2.values[1]) + _sqr (e1.values[2]-e2.values[2])) ;
            double z_dist = sqrt (_sqr (e2.values[0]-e6.values[0]) + _sqr (e2.values[1]-e6.values[1]) + _sqr (e2.values[2]-e6.values[2])) ;

            // Create file name for saving
            std::stringstream object_filename;

            //double min_surface = std::min (x_surface, std::min (y_surface, z_surface));

            //cerr << " x_surface " << x_surface << endl ;
            //cerr << " y_surface " << y_surface << endl ;
            //cerr << " z_surface " << z_surface << endl ;

            double smallest_dimension = std::min (x_dist, std::min (y_dist, z_dist));
            double  biggest_dimension = std::max (x_dist, std::max (y_dist, z_dist));
            double   medium_dimension = (smallest_dimension==x_dist) ? (biggest_dimension==z_dist ? y_dist:z_dist) : (biggest_dimension==z_dist ? y_dist:x_dist);
            cerr << smallest_dimension << " < " << medium_dimension << " < " << biggest_dimension << endl;

            cerr << " x_dist " << x_dist << endl ;
            cerr << " y_dist " << y_dist << endl ;
            cerr << " z_dist " << z_dist << endl ;

            double min_dist = std::min (x_dist, y_dist);
            double max_dist = std::max (x_dist, y_dist);

            cerr << " max / min " << max_dist / min_dist << endl ;
            cerr << " max / z " << max_dist / z_dist << endl ;

            cerr << "flat_value = " << flat_value << endl ;
            cerr << "a/(b+c) = " << smallest_dimension / (medium_dimension + biggest_dimension) << endl ;

            // TODO This is NOT the optim way, just a way // Begin //
            ///*
            pcl::PointIndices::Ptr backup_novel_box_inliers (new pcl::PointIndices ());

            for (int idx = 0; idx < (int) backup_working_cloud->points.size(); idx++)
            {
              //double x = backup_working_cloud->points.at (idx).x;
              //double y = backup_working_cloud->points.at (idx).y;
              //
              //double sqr_dist_to_e0 = _sqr (x - edges[0][0]) + _sqr (y - edges[0][1]);
              //double sqr_dist_to_e1 = _sqr (x - edges[1][0]) + _sqr (y - edges[1][1]);
              //double sqr_dist_to_e2 = _sqr (x - edges[2][0]) + _sqr (y - edges[2][1]);
              //double sqr_dist_to_e3 = _sqr (x - edges[3][0]) + _sqr (y - edges[3][1]);
              //double sum_of_sqr_dist = sqr_dist_to_e0 + sqr_dist_to_e1 + sqr_dist_to_e2 + sqr_dist_to_e3;
              //
              //double sqr_dist_from_e0_to_e1 = _sqr (edges[0][0] - edges[1][0]) + _sqr (edges[0][1] - edges[1][1]);
              //double sqr_dist_from_e1_to_e2 = _sqr (edges[1][0] - edges[2][0]) + _sqr (edges[1][1] - edges[2][1]);
              //double sqr_dist_from_e2_to_e3 = _sqr (edges[2][0] - edges[3][0]) + _sqr (edges[2][1] - edges[3][1]);
              //double sqr_dist_from_e3_to_e0 = _sqr (edges[3][0] - edges[0][0]) + _sqr (edges[3][1] - edges[0][1]);
              //double sum_of_sqr_rect = sqr_dist_from_e0_to_e1 + sqr_dist_from_e1_to_e2 + sqr_dist_from_e2_to_e3 + sqr_dist_from_e3_to_e0;
              //
              //if ( sum_of_sqr_dist < sum_of_sqr_rect )
              //{
              //// Save only the right indices
              //backup_novel_box_inliers->indices.push_back (idx);
              //}

              pcl::PointXYZ B;
              B.x = backup_working_cloud->points.at (idx).x;
              B.y = backup_working_cloud->points.at (idx).y;

              pcl::PointXYZ E0;  E0.x = edges[0][0];  E0.y = edges[0][1];
              pcl::PointXYZ E1;  E1.x = edges[1][0];  E1.y = edges[1][1];
              pcl::PointXYZ E2;  E2.x = edges[2][0];  E2.y = edges[2][1];
              pcl::PointXYZ E3;  E3.x = edges[3][0];  E3.y = edges[3][1];

              double A0 = whatAngle (E0, B, E1);
              double A1 = whatAngle (E1, B, E2);
              double A2 = whatAngle (E2, B, E3);
              double A3 = whatAngle (E3, B, E0);
              double SA = abs(A0) + abs(A1) + abs(A2) + abs(A3);

              //cerr << A0 << endl ;
              //cerr << A1 << endl ;
              //cerr << A2 << endl ;
              //cerr << A3 << endl ;
              //cerr << " = " << abs(A0) + abs(A1) + abs(A2) + abs(A3) << endl ;
              //viewer.spin ();

              if ( abs(SA - 360) <= sat )
              {
                B.z = backup_working_cloud->points.at (idx).z;

                if ( (minimus < B.z) && (B.z < maximus) )
                  backup_novel_box_inliers->indices.push_back (idx);

                //cerr << SA << endl ;
              }

            }
            //*/
            // TODO This is NOT the optim way, just a way // End //

            //if ( ((max_dist / min_dist) > flat_value) || ((max_dist / z_dist) > flat_value ))
            //if ( smallest_dimension < flat_value )
            if ( (smallest_dimension / (medium_dimension + biggest_dimension)) < flat_value )
              //if ((max_dist / min_dist) > flat_value)
            {
              if ( number_of_flat < 10 )
                object_filename << directory << "---" << "object" << "-" << "flat" << "_" << "0" << number_of_flat << ".pcd" ;
              else
                object_filename << directory << "---" << "object" << "-" << "flat" << "_" << number_of_flat << ".pcd" ;

              cerr << "      FLAT      " << endl ;

              std::stringstream box_cloud_id;
              box_cloud_id << "BOX_CLOUD_" << getTimestamp ();
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> box_cloud_color (box_cloud, 0, 127, 255);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (box_cloud, box_cloud_color, box_cloud_id.str ());
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, box_cloud_id.str ());
              if ( space_step ) viewer.spin ();

              viewer.removePointCloud ("generic");
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "generic");
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
              if ( space_step ) viewer.spin ();

              std::stringstream cub_id;
              cub_id << "CUB_" << getTimestamp ();
              viewer.addCuboid (pla, 0.0, 0.5, 1.0, 0.5, cub_id.str ());
              if ( space_step ) viewer.spin ();

              cubs_per_view.push_back (pla);

              clouds_of_cubs_per_view.push_back (box_cloud);

              number_of_flat++;
              /*
                 pcl::PointCloud<pcl::PointXYZRGBI>::Ptr marked_box_cloud (new pcl::PointCloud<pcl::PointXYZRGBI> ());
                 pcl::copyFields (*box_cloud, *marked_box_cloud);
                 for (int idx=0; idx < (int) marked_box_cloud->points.size (); idx++)
                 marked_box_cloud->points.at (idx).label = 3;
               *marked_working_cloud += *marked_box_cloud;
               */
              ///*
              for (int idx=0; idx < (int) backup_novel_box_inliers->indices.size (); idx++)
                marked_working_cloud->points.at (backup_novel_box_inliers->indices.at (idx)).intensity = 3;
              //*/
            }
            else
            {
              if ( number_of_box < 10 )
                object_filename << directory << "---" << "object" << "-" << "box" << "_" << "0" << number_of_box << ".pcd" ;
              else
                object_filename << directory << "---" << "object" << "-" << "box" << "_" << number_of_box << ".pcd" ;

              cerr << "      BOX      " << endl ;

              std::stringstream box_cloud_id;
              box_cloud_id << "BOX_CLOUD_" << getTimestamp ();
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> box_cloud_color (box_cloud, 127, 0, 255);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (box_cloud, box_cloud_color, box_cloud_id.str ());
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, box_cloud_id.str ());
              if ( space_step ) viewer.spin ();

              viewer.removePointCloud ("generic");
              pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
              viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "generic");
              viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
              if ( space_step ) viewer.spin ();

              std::stringstream cub_id;
              cub_id << "CUB_" << getTimestamp ();
              viewer.addCuboid (pla, 0.5, 0.0, 1.0, 0.5, cub_id.str ());
              if ( space_step ) viewer.spin ();

              cubs_per_view.push_back (pla);

              clouds_of_cubs_per_view.push_back (box_cloud);

              number_of_box++;
              /*
                 pcl::PointCloud<pcl::PointXYZRGBI>::Ptr marked_box_cloud (new pcl::PointCloud<pcl::PointXYZRGBI> ());
                 pcl::copyFields (*box_cloud, *marked_box_cloud);
                 for (int idx=0; idx < (int) marked_box_cloud->points.size (); idx++)
                 marked_box_cloud->points.at (idx).label = 2;
               *marked_working_cloud += *marked_box_cloud;
               */
              ///*
              for (int idx=0; idx < (int) backup_novel_box_inliers->indices.size (); idx++)
                marked_working_cloud->points.at (backup_novel_box_inliers->indices.at (idx)).intensity = 2;
              //*/
            }

            cerr << object_filename.str () << endl;
            cerr << object_filename.str () << endl;
            cerr << object_filename.str () << endl;

            pcl::io::savePCDFile (object_filename.str (), *box_cloud, 10);

            if ( !till_the_end ) viewer.spin ();

          }
        }
      }
    }

    cerr << endl << endl << endl ;
  }

  // ---------- Fitting Of Models ---------- //

  int fit = 0;

  //int model = 0;

  bool continue_hough = true;

  do
  {
    pcl::PointCloud<pcl::PointNormal>::Ptr line_parameters_space (new pcl::PointCloud<pcl::PointNormal> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_parameters_space (new pcl::PointCloud<pcl::PointXYZ> ());

    //if ( fit > 9 )
    //{
      ////std::cout << " fitting_step = ";
      ////std::cin >> fitting_step;

      //fitting_step = 1;
      //space_step = 1;
      //growing_visualization = 1;

    //}

    for (int ite = 0; ite < vransac_iterations; ite++)
    {
      bool valid_line = true;
      bool valid_circle = true;

      std::vector<std::string> ids;

      pcl::console::print_info ("----------------------------------------------------------------------------------------------------\n");
      pcl::console::print_info ("AT FIT = %d AT ITERATION = %d\n", fit, ite);

      pcl::PointIndices::Ptr line_inliers (new pcl::PointIndices ());
      pcl::ModelCoefficients::Ptr line_coefficients (new pcl::ModelCoefficients ());
      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr line_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      pcl::PointIndices::Ptr circle_inliers (new pcl::PointIndices ());
      pcl::ModelCoefficients::Ptr circle_coefficients (new pcl::ModelCoefficients ());
      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr circle_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      //pcl::SACSegmentation<pcl::PointXYZRGBNormalRSD> sacs;
      //sacs.setInputCloud (working_cloud);
      //sacs.setMethodType (pcl::SAC_RANSAC);
      //sacs.setOptimizeCoefficients (false);

      int line_iterations = fitLine (working_cloud, line_inliers, line_coefficients, line_threshold, maximum_line_iterations);

      adjustLineInliers (working_cloud, line_inliers, line_coefficients, line_threshold);

      /*
      // THIS SHOULD BE ONLY TEMPORARY HERE //
      valid_line = false;
      */

      //sacs.setModelType (pcl::SACMODEL_LINE);
      //sacs.setMaxIterations (maximum_line_iterations);
      //sacs.setDistanceThreshold (line_threshold);
      //sacs.segment (*line_inliers, *line_coefficients);


      int circle_iterations = fitCircle (working_cloud, circle_inliers, circle_coefficients, circle_threshold, minimum_circle_radius, maximum_circle_radius, maximum_circle_iterations);

      /*
      // THIS SHOULD BE ONLY TEMPORARY HERE //
      valid_circle = false;
      */

      //sacs.setModelType (pcl::SACMODEL_CIRCLE2D);
      //sacs.setMaxIterations (maximum_circle_iterations);
      //sacs.setDistanceThreshold (circle_threshold);
      //sacs.setRadiusLimits (minimum_circle_radius, maximum_circle_radius);
      //sacs.segment (*circle_inliers, *circle_coefficients);



      pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> f_ei;

      f_ei.setIndices (line_inliers);
      f_ei.setInputCloud (working_cloud);
      f_ei.setNegative (false);
      f_ei.filter (*line_cloud);

      f_ei.setIndices (circle_inliers);
      f_ei.setInputCloud (working_cloud);
      f_ei.setNegative (false);
      f_ei.filter (*circle_cloud);



      ///*
      adjustLineCoefficients (line_cloud, line_coefficients);

      if ( fitting_step )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "LINE_CLOUD_ID");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "LINE_CLOUD_ID");
        viewer.addLine (*line_coefficients, 1.0, 0.0, 0.0, "LINE_MODEL_ID");
        viewer.spin ();
        viewer.removePointCloud ("LINE_CLOUD_ID");
        viewer.removeShape ("LINE_MODEL_ID");
        viewer.spin ();
      }

      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr adjust_using_curvature (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      // Clustering Feature For Lines //
      if ( clustering_feature_for_lines )
      {
        if ( (int) line_inliers->indices.size() < minimum_line_inliers )
        {
          valid_line = false;
          if ( verbose ) pcl::console::print_error ("  [MINIMUM LINE INLIERS] Line Rejected !\n");
        }
        else
          ClusteringFeatureForLines (valid_line, working_cloud, line_cloud, line_inliers, line_coefficients, viewer, adjust_using_curvature, verbose, fitting_step);
      }
      //*/



      ///*
      //// Curvature Feature For Lines //
      //if ( curvature_feature_for_lines )
      //{
      //if ( (int) line_inliers->indices.size() < minimum_line_inliers )
      //{
      //valid_line = false;
      //if ( verbose ) pcl::console::print_error ("  [MINIMUM LINE INLIERS] Line Rejected !\n");
      //}
      //else
      //CurvatureFeatureForLines (valid_line, working_cloud, line_cloud, line_inliers, line_coefficients, viewer, verbose, fitting_step);
      //}
      //*/



      ///*
      if ( (int) line_inliers->indices.size() < minimum_line_inliers )
      {
        valid_line = false;
        if ( verbose ) pcl::console::print_error ("  [MINIMUM LINE INLIERS] Line Rejected !\n");
      }
      else
      {
        // Adjust Model Of Line //
        double half_of_line_threshold = line_threshold / 2;
        //adjustLineModel (line_cloud, line_inliers, line_coefficients, half_of_line_threshold, maximum_line_iterations);
        adjustLineModel (adjust_using_curvature, line_inliers, line_coefficients, half_of_line_threshold, maximum_line_iterations);



              if ( fitting_step )
              {
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> adjust_using_curvature_color (adjust_using_curvature, 64, 255, 127);
                viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (adjust_using_curvature, adjust_using_curvature_color, "id1");
                viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "id1");
                /*
                      viewer.addLine (*line_coefficients, 0.25, 1.0, 0.5, "id2");
                */
                viewer.addTube (*line_coefficients, 0.25, 1.0, 0.5, 0.005, "id2");
                viewer.spin ();
                viewer.removePointCloud ("id1");
                viewer.removeShape ("id2");
                viewer.spin ();
              }



        double pt1[2];
        pt1[0] = line_coefficients->values.at (0);
        pt1[1] = line_coefficients->values.at (1);

        double pt2[2];
        pt2[0] = line_coefficients->values.at (3) + line_coefficients->values.at (0);
        pt2[1] = line_coefficients->values.at (4) + line_coefficients->values.at (1);

        // A.K.A. length of line //
        float lol = sqrt (_sqr (pt2[0] - pt1[0]) + _sqr (pt2[1] - pt1[1]));

        pcl::console::print_warn ("  (b) length of line = %.3f | where minimum length of line = %.3f \n", lol, minimum_line_length);

        if ( lol < minimum_line_length )
        {
          valid_line = false;
          if ( verbose ) pcl::console::print_error ("  [LENGTH OF LINE] Line Rejected ! \n");
        }

/*

        // Adjust Coefficients Of Line Model //
        adjustLineCoefficients (line_cloud, line_coefficients);

*/

        pt1[0] = line_coefficients->values.at (0);
        pt1[1] = line_coefficients->values.at (1);

        pt2[0] = line_coefficients->values.at (3) + line_coefficients->values.at (0);
        pt2[1] = line_coefficients->values.at (4) + line_coefficients->values.at (1);

        // A.K.A. length of line //
        lol = sqrt (_sqr (pt2[0] - pt1[0]) + _sqr (pt2[1] - pt1[1]));

        pcl::console::print_warn ("  (c) length of line = %.3f | where minimum length of line = %.3f \n", lol, minimum_line_length);

        if ( fitting_step )
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 255, 127, 0);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "LINE_CLOUD_ID");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "LINE_CLOUD_ID");
          viewer.addLine (*line_coefficients, 1.0, 0.5, 0.0, "LINE_MODEL_ID");
          viewer.spin ();
          viewer.removePointCloud ("LINE_CLOUD_ID");
          viewer.removeShape ("LINE_MODEL_ID");
          viewer.spin ();
        }
      }
      //*/



      ///*
      //// Normal Feature For Lines //
      //if ( normal_feature_for_lines )
      //{
      //if ( (int) line_inliers->indices.size() < minimum_line_inliers )
      //{
      //valid_line = false;
      //if ( verbose ) pcl::console::print_error ("  [MINIMUM LINE INLIERS] Line Rejected !\n");
      //}
      //else
      //NormalFeatureForLines (valid_line, working_cloud, line_cloud, line_inliers, line_coefficients, viewer, verbose, fitting_step);
      //}
      //*/
      //
      //
      //
      ///*
      //if ( fitting_step )
      //{
      //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> line_cloud_color (line_cloud, 255, 0, 0);
      //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (line_cloud, line_cloud_color, "LINE_CLOUD_ID");
      //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "LINE_CLOUD_ID");
      //viewer.addLine (*line_coefficients, 1.0, 0.0, 0.0, "LINE_MODEL_ID");
      //viewer.spin ();
      //viewer.removePointCloud ("LINE_CLOUD_ID");
      //viewer.removeShape ("LINE_MODEL_ID");
      //viewer.spin ();
      //}
      //*/


      ///*
      if ( fitting_step )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 255, 0, 0);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "CIRCLE_CLOUD_ID");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "CIRCLE_CLOUD_ID");
        viewer.addCircle (*circle_coefficients, 1.0, 0.0, 0.0, "CIRCLE_MODEL_ID");
        viewer.spin ();
        viewer.removePointCloud ("CIRCLE_CLOUD_ID");
        viewer.removeShape ("CIRCLE_MODEL_ID");
        viewer.spin ();
      }

      // Clustering Feature For Circles //
      if ( clustering_feature_for_circles )
      {
        if ( (int) circle_inliers->indices.size() < minimum_circle_inliers )
        {
          valid_circle = false;
          if ( verbose ) pcl::console::print_error ("  [MINIMUM CIRCLE INLIERS] Circle Rejected !\n");
        }
        else
          ClusteringFeatureForCircles (valid_circle, working_cloud, circle_cloud, circle_inliers, circle_coefficients, viewer, verbose, fitting_step);
      }

      // Adjusting Feature For Circles //
      if ( (int) circle_inliers->indices.size() < minimum_circle_inliers )
      {
        valid_circle = false;
        if ( verbose ) pcl::console::print_error ("  [MINIMUM CIRCLE INLIERS] Circle Rejected !\n");
      }
      else
      {
        // Adjust Model Of Circle //
        double half_of_circle_threshold = circle_threshold / 2;
        adjustCircleModel (circle_cloud, circle_inliers, circle_coefficients, half_of_circle_threshold, minimum_circle_radius, maximum_circle_radius, maximum_circle_iterations);

        double mcx = circle_coefficients->values.at (0);
        double mcy = circle_coefficients->values.at (1);
        double  mr = circle_coefficients->values.at (2);

        pcl::PointIndices::Ptr planar_cylinder_inliers (new pcl::PointIndices ());

        for (int idx = 0; idx < (int) planar_curvature_cloud->points.size(); idx++)
        {
          double xx = planar_curvature_cloud->points.at (idx).x;
          double yy = planar_curvature_cloud->points.at (idx).y;

          double dd = sqrt ( _sqr (mcx - xx) + _sqr (mcy - yy) );

          if ( dd < (mr + circle_threshold) )
            planar_cylinder_inliers->indices.push_back (idx);
        }

        pcl::console::print_value ("  # PLANAR POINTS OF CYLINDER = %d \n", planar_cylinder_inliers->indices.size ());

        //if ( 1025 < planar_cylinder_inliers->indices.size () )
        //if ( 525 < planar_cylinder_inliers->indices.size () )
        if ( too_many_planar_curvatures < planar_cylinder_inliers->indices.size () )
        {
          pcl::console::print_value ("  Skiping this cylinder model ! Too Many Planar Curvatures.\n");
          valid_circle = false;
        }

        if ( fitting_step )
        {
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 255, 127, 0);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "CIRCLE_CLOUD_ID");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "CIRCLE_CLOUD_ID");
          viewer.addCircle (*circle_coefficients, 1.0, 0.5, 0.0, "CIRCLE_MODEL_ID");
          viewer.spin ();
          viewer.removePointCloud ("CIRCLE_CLOUD_ID");
          viewer.removeShape ("CIRCLE_MODEL_ID");
          viewer.spin ();
        }
      }
      //*/


      ///*
      //// Curvature Feature For Circles //
      //if ( curvature_feature_for_circles )
      //{
      //if ( (int) circle_inliers->indices.size() < minimum_circle_inliers )
      //{
      //valid_circle = false;
      //if ( verbose ) pcl::console::print_error ("  [MINIMUM CIRCLE INLIERS] Circle Rejected !\n");
      //}
      //else
      //CurvatureFeatureForCircles (valid_circle, working_cloud, circle_cloud, circle_inliers, circle_coefficients, viewer, verbose, fitting_step);
      //}
      //
      //
      //
      //// Normal Feature For Circles //
      //if ( normal_feature_for_circles )
      //{
      //if ( (int) circle_inliers->indices.size() < minimum_circle_inliers )
      //{
      //valid_circle = false;
      //if ( verbose ) pcl::console::print_error ("  [MINIMUM CIRCLE INLIERS] Circle Rejected !\n");
      //}
      //else
      //NormalFeatureForCircles (valid_circle, working_cloud, circle_cloud, circle_inliers, circle_coefficients, viewer, verbose, fitting_step);
      //}
      //
      //
      //
      //if ( fitting_step )
      //{
      //pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> circle_cloud_color (circle_cloud, 255, 0, 0);
      //viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (circle_cloud, circle_cloud_color, "CIRCLE_CLOUD_ID");
      //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "CIRCLE_CLOUD_ID");
      //viewer.addCircle (*circle_coefficients, 1.0, 0.0, 0.0, "CIRCLE_MODEL_ID");
      //viewer.spin ();
      //viewer.removePointCloud ("CIRCLE_CLOUD_ID");
      //viewer.removeShape ("CIRCLE_MODEL_ID");
      //viewer.spin ();
      //}
      //*/

      if ( valid_line == false ) { line_inliers->indices.clear (); line_cloud->points.clear (); }
      if ( valid_circle == false ) { circle_inliers->indices.clear (); circle_cloud->points.clear (); }

      if ( verbose ) pcl::console::print_warn ("  %5d line inliers vs. %5d circle inliers\n", line_inliers->indices.size (), circle_inliers->indices.size ());
      if ( verbose ) pcl::console::print_info ("  %5d line cloudsz vs. %5d circle cloudsz\n", line_cloud->points.size (), circle_cloud->points.size ());

      // Parameters //
      double FP1[2];
      FP1[0] = line_coefficients->values.at (0);
      FP1[1] = line_coefficients->values.at (1);

      double FP2[2];
      FP2[0] = line_coefficients->values.at (3) + line_coefficients->values.at (0);
      FP2[1] = line_coefficients->values.at (4) + line_coefficients->values.at (1);

      double FCX = circle_coefficients->values.at (0);
      double FCY = circle_coefficients->values.at (1);
      double  FR = circle_coefficients->values.at (2);

      // Decide Between Line or Circle //
      if ( ( valid_line ) || ( valid_circle ) )
      {

        // THIS SHOULD BE ONLY TEMPORARY HERE //
//        if (line_inliers->indices.size () > circle_inliers->indices.size ())
        if (line_cloud->points.size () > circle_cloud->points.size ())
        {
        if ( valid_line )
        {
          if (line_inliers->indices.size () > 0 )
          {
            if (line_inliers->indices.size () > minimum_line_inliers )
            {
              if ( verbose ) pcl::console::print_value ("  Line Accepted ! Has %4d inliers with FP1 = [%6.3f,%6.3f] and FP2 = [%6.3f,%6.3f]\n", (int) line_inliers->indices.size (), FP1[0], FP1[1], FP2[0], FP2[1] );

              // Cast Vote For Lines //
              pcl::PointNormal line_vote;
              //line_vote.x = (FP1[0] + FP2[0]) / 2;
              //line_vote.y = (FP1[1] + FP2[1]) / 2;
              //line_vote.z = sqrt ( _sqr (FP2[0] - FP1[0]) + _sqr (FP2[1] - FP1[1]) );
              //
              //line_vote.normal_x  = FP1[0];
              //line_vote.normal_y  = FP1[1];
              //line_vote.normal_z  = FP2[0];
              //line_vote.curvature = FP2[1];

              line_vote.x = FP1[0];
              line_vote.y = FP1[1];
              line_vote.z = 0.0;

              line_vote.normal_x  = FP2[0];
              line_vote.normal_y  = FP2[1];
              line_vote.normal_z  = 0.0;
              line_vote.curvature = 0.0;

              line_parameters_space->points.push_back (line_vote);
            }
            else
              if ( verbose ) pcl::console::print_error ("  Line Rejected !\n");
          }
          else
            if ( verbose ) pcl::console::print_error ("  Still Rejecting The Line !\n");
        }
        }
        // THIS SHOULD BE ONLY TEMPORARY HERE //
        else
        {

        if ( valid_circle )
        {
          if (circle_inliers->indices.size () > 0 )
          {
          if (circle_inliers->indices.size () > minimum_circle_inliers )
          {
          if ( verbose ) pcl::console::print_value ("  Circle Accepted ! Has %4d inliers with FC = (%6.3f,%6.3f) and FR = %5.3f in [%5.3f, %5.3f]\n", (int) circle_inliers->indices.size (), FCX, FCY, FR, minimum_circle_radius, maximum_circle_radius);

          // Cast Vote For Circles //
          pcl::PointXYZ circle_vote;
          circle_vote.x = FCX;
          circle_vote.y = FCY;
          circle_vote.z =  FR;

          circle_parameters_space->points.push_back (circle_vote);

          }
          else
            if ( verbose ) pcl::console::print_error ("  Circle No Good !\n");
          }
          else
            if ( verbose ) pcl::console::print_error ("  Circle No Good !\n");
        }
        else
          if ( verbose ) pcl::console::print_error ("  Still Rejecting The Circle !\n");

        // THIS SHOULD BE ONLY TEMPORARY HERE //
        }

      }
      else
        if ( verbose ) pcl::console::print_error ("  NO MODEL ACCEPTED !\n");
    }

    // ---------- Parameter Space of Lines ---------- //

    if ( false )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> line_parameters_space_color (line_parameters_space, 0, 255, 255);
      viewer.addPointCloud<pcl::PointNormal> (line_parameters_space, line_parameters_space_color, "LINE_PARAMETER_SPACE");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size * 3, "LINE_PARAMETER_SPACE");
      viewer.spin ();
      viewer.removePointCloud ("LINE_PARAMETER_SPACE");
    }

    pcl::PointCloud<pcl::PointNormal>::Ptr line_parameters_cloud (new pcl::PointCloud<pcl::PointNormal> ());

    if ( line_parameters_space->points.size () > 0 )
    {
      std::vector<pcl::PointIndices> line_parameters_clusters;
      pcl::search::KdTree<pcl::PointNormal>::Ptr line_parameters_tree (new pcl::search::KdTree<pcl::PointNormal> ());
      line_parameters_tree->setInputCloud (line_parameters_space);

      // XXX SEGFAULT //
      //cerr << endl << "line_parameters_space->points.size () = " << line_parameters_space->points.size () << endl ;
      //cerr << "clustering_tolerance_of_line_parameters_space = " << clustering_tolerance_of_line_parameters_space << endl ;
      //cerr << "minimum_size_of_line_parameters_clusters = " << minimum_size_of_line_parameters_clusters << endl << endl ;
      //for (int sf=0; sf < (int) line_parameters_space->points.size(); sf++)
      //cerr << sf << " - " << line_parameters_space->points.at (sf) << endl ;
      // XXX SEGFAULT //

      pcl::EuclideanClusterExtraction<pcl::PointNormal> lps_ece;
      lps_ece.setInputCloud (line_parameters_space);
      lps_ece.setClusterTolerance (clustering_tolerance_of_line_parameters_space);
      lps_ece.setMinClusterSize (minimum_size_of_line_parameters_clusters);
      lps_ece.setSearchMethod (line_parameters_tree);
      lps_ece.extract (line_parameters_clusters);

      pcl::PointIndices::Ptr biggest_cluster_of_line_parameters (new pcl::PointIndices (line_parameters_clusters.at (0)));

      pcl::ExtractIndices<pcl::PointNormal> lps_ei;
      lps_ei.setInputCloud (line_parameters_space);
      lps_ei.setIndices (biggest_cluster_of_line_parameters);
      lps_ei.setNegative (false);
      lps_ei.filter (*line_parameters_cloud);

      if ( verbose )
      {
        pcl::console::print_info ("The parameters space of lines has %d votes and %d clusters where:\n", (int) line_parameters_space->points.size (), (int) line_parameters_clusters.size ());
        for (int clu = 0; clu < (int) line_parameters_clusters.size(); clu++)
          pcl::console::print_info ("  Cluster %d has %d points\n", clu, (int) line_parameters_clusters.at (clu).indices.size());
      }

      if ( space_step )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> line_parameters_cloud_color (line_parameters_cloud, 0, 255, 255);
        viewer.addPointCloud<pcl::PointNormal> (line_parameters_cloud, line_parameters_cloud_color, "LINE_PARAMETER_CLOUD");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size * 3, "LINE_PARAMETER_CLOUD");
        viewer.spin ();
        viewer.removePointCloud ("LINE_PARAMETER_CLOUD");
      }
    }
    else
      pcl::console::print_error ("No votes in parameters space of lines.\n");

    // ---------- Parameter Space of Circles ---------- //

    if ( !till_the_end )
    if ( true )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> circle_parameters_space_color (circle_parameters_space, 0, 255, 255);
      viewer.addPointCloud<pcl::PointXYZ> (circle_parameters_space, circle_parameters_space_color, "CIRCLE_PARAMETER_SPACE");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size * 3, "CIRCLE_PARAMETER_SPACE");
      viewer.spin ();
      viewer.removePointCloud ("CIRCLE_PARAMETER_SPACE");
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_parameters_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    if ( circle_parameters_space->points.size () > 0 )
    {
      std::vector<pcl::PointIndices> circle_parameters_clusters;
      pcl::search::KdTree<pcl::PointXYZ>::Ptr circle_parameters_tree (new pcl::search::KdTree<pcl::PointXYZ> ());
      circle_parameters_tree->setInputCloud (circle_parameters_space);

      // XXX SEGFAULT //
      //cerr << endl << "circle_parameters_space->points.size () = " << circle_parameters_space->points.size () << endl ;
      //cerr << "clustering_tolerance_of_circle_parameters_space = " << clustering_tolerance_of_circle_parameters_space << endl ;
      //cerr << "minimum_size_of_circle_parameters_clusters = " << minimum_size_of_circle_parameters_clusters << endl << endl ;
      //for (int sf=0; sf < (int) circle_parameters_space->points.size(); sf++)
      //cerr << sf << " - " << circle_parameters_space->points.at (sf) << endl ;
      // XXX SEGFAULT //

      pcl::EuclideanClusterExtraction<pcl::PointXYZ> cps_ece;
      cps_ece.setInputCloud (circle_parameters_space);
      cps_ece.setClusterTolerance (clustering_tolerance_of_circle_parameters_space);
      cps_ece.setMinClusterSize (minimum_size_of_circle_parameters_clusters);
      cps_ece.setSearchMethod (circle_parameters_tree);
      cps_ece.extract (circle_parameters_clusters);

      pcl::PointIndices::Ptr biggest_cluster_of_circle_parameters (new pcl::PointIndices (circle_parameters_clusters.at (0)));

      pcl::ExtractIndices<pcl::PointXYZ> cps_ei;
      cps_ei.setInputCloud (circle_parameters_space);
      cps_ei.setIndices (biggest_cluster_of_circle_parameters);
      cps_ei.setNegative (false);
      cps_ei.filter (*circle_parameters_cloud);

      if ( verbose )
      {
        pcl::console::print_info ("The parameters space of circles has %d votes and %d clusters where:\n", (int) circle_parameters_space->points.size (), (int) circle_parameters_clusters.size ());
        for (int clu = 0; clu < (int) circle_parameters_clusters.size(); clu++)
          pcl::console::print_info ("  Cluster %d has %d points\n", clu, (int) circle_parameters_clusters.at (clu).indices.size());
      }

      if ( space_step )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> circle_parameters_cloud_color (circle_parameters_cloud, 0, 255, 255);
        viewer.addPointCloud<pcl::PointXYZ> (circle_parameters_cloud, circle_parameters_cloud_color, "CIRCLE_PARAMETER_CLOUD");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size * 3, "CIRCLE_PARAMETER_CLOUD");
        viewer.spin ();
        viewer.removePointCloud ("CIRCLE_PARAMETER_CLOUD");
      }
    }
    else
      pcl::console::print_error ("No votes in parameters space of circles.\n");

    // ---------- Decide Which Model Type Has More Votes ---------- //

    bool more_votes_for_lines = false;
    bool more_votes_for_circles = false;

    if ( line_parameters_cloud->points.size () > circle_parameters_cloud->points.size () )
    {
      more_votes_for_lines = true;
      if ( verbose ) pcl::console::print_value ("More votes for lines, i.e. %d \n", line_parameters_cloud->points.size ());
    }
    else if ( line_parameters_cloud->points.size () < circle_parameters_cloud->points.size () )
    {
      more_votes_for_circles = true;
      if ( verbose ) pcl::console::print_value ("More votes for circles, i.e. %d \n", circle_parameters_cloud->points.size ());
    }
    else if ( line_parameters_cloud->points.size () == circle_parameters_cloud->points.size () )
    {
      if ( (line_parameters_cloud->points.size () > 0) && (circle_parameters_cloud->points.size () > 0) )
      {
        more_votes_for_lines = true;
        more_votes_for_circles = true;
        if ( verbose ) pcl::console::print_value ("The same number of votes.\n");
      }
      else if ( (line_parameters_cloud->points.size () == 0) && (circle_parameters_cloud->points.size () == 0) )
      {
        more_votes_for_lines = false;
        more_votes_for_circles = false;
        if ( verbose ) pcl::console::print_value ("No votes at all.\n");
      }
    }

    if ( !till_the_end ) viewer.spin ();

    // THIS SHOULD BE ONLY TEMPORARY HERE //
//    more_votes_for_lines = false;
//    more_votes_for_circles = true;



    // ---------- Estimate The Box Like Object ---------- //



    if ( more_votes_for_lines )
    {
    float sxm = 0.0;
    float sym = 0.0;
    float  sl = 0.0;

    float sx1 = 0.0;
    float sy1 = 0.0;
    float sx2 = 0.0;
    float sy2 = 0.0;

    int votes = line_parameters_cloud->size ();

    for (int vot = 0; vot < votes; vot++)
    {
      float xm = line_parameters_cloud->points.at (vot).x;
      float ym = line_parameters_cloud->points.at (vot).y;
      float  l = line_parameters_cloud->points.at (vot).z;

      float x1 = line_parameters_cloud->points.at (vot).normal_x;
      float y1 = line_parameters_cloud->points.at (vot).normal_y;
      float x2 = line_parameters_cloud->points.at (vot).normal_z;
      float y2 = line_parameters_cloud->points.at (vot).curvature;

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

    double vec[2];
    vec[0] = mx2 - mx1;
    vec[1] = my2 - my1;

    double nor = sqrt ( _sqr (vec[0]) + _sqr (vec[1]) );
    vec[0] = vec[0] / nor;
    vec[1] = vec[1] / nor;

    double P1[2];
    //P1[0] = mxm + vec[0] * ml / 2;
    //P1[1] = mym + vec[1] * ml / 2;
    P1[0] = mxm;
    P1[1] = mym;

    double P2[2];
    //P2[0] = mxm - vec[0] * ml / 2;
    //P2[1] = mym - vec[1] * ml / 2;
    P2[0] = mx1;
    P2[1] = my1;

    pcl::ModelCoefficients::Ptr vransac_line_coefficients (new pcl::ModelCoefficients ());
    vransac_line_coefficients->values.push_back (P1[0]);
    vransac_line_coefficients->values.push_back (P1[1]);
    vransac_line_coefficients->values.push_back (0.0);
    vransac_line_coefficients->values.push_back (P2[0] - P1[0]);
    vransac_line_coefficients->values.push_back (P2[1] - P1[1]);
    vransac_line_coefficients->values.push_back (0.0);

    if ( verbose )
    {
      pcl::console::print_info ("These 3 distances need to be appproximately equal:\n");
      pcl::console::print_info ("  %7.5f meters\n", ml);
      pcl::console::print_info ("  %7.5f meters\n", sqrt (_sqr (P2[0] - P1[0]) + _sqr (P2[1] - P1[1])));
      pcl::console::print_info ("  %7.5f meters\n", sqrt (_sqr (mx2 - mx1) + _sqr (my2 - my1)));
    }

    pcl::PointIndices::Ptr vransac_line_inliers (new pcl::PointIndices ());

    // THIS IS AN EXHAUSTIVE SEARCH OF VRANSAC LINE INLIERS //

    ///*
    for (int idx = 0; idx < working_cloud->points.size (); idx++)
    {
      double P0[2];
      P0[0] = working_cloud->points.at (idx).x;
      P0[1] = working_cloud->points.at (idx).y;

      double num = (P0[0] - P1[0])*(P2[0] - P1[0]) + (P0[1] - P1[1])*(P2[1] - P1[1]);
      double den = _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]);
      double u = num / den;

      double I[2];
      I[0] = P1[0] + u * (P2[0] - P1[0]);
      I[1] = P1[1] + u * (P2[1] - P1[1]);

      double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

      if ( d < line_threshold )
      {
        double IP1D  = sqrt ( _sqr  (I[0] - P1[0])  +  _sqr  (I[1] - P1[1]) );
        double IP2D  = sqrt ( _sqr  (I[0] - P2[0])  +  _sqr  (I[1] - P2[1]) );
        double P1P2D = sqrt ( _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]) );

        if ( (IP1D + IP2D) < (P1P2D + 0.001) )
          vransac_line_inliers->indices.push_back (idx);
      }
    }

    cerr << " vransac_line_inliers = " << vransac_line_inliers->indices.size () << endl;

    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr vransac_line_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> l_ei;
    l_ei.setInputCloud (working_cloud);
    l_ei.setIndices (vransac_line_inliers);
    l_ei.setNegative (false);
    l_ei.filter (*vransac_line_cloud);

    if ( space_step )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> vransac_line_cloud_color (vransac_line_cloud, 0, 255, 255);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (vransac_line_cloud, vransac_line_cloud_color, "VRANSAC_LINE_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "VRANSAC_LINE_CLOUD");
      viewer.addLine (*vransac_line_coefficients, 0.0, 1.0, 1.0, "VRANSAC_LINE_MODEL");
      viewer.spin ();
      viewer.removePointCloud ("VRANSAC_LINE_CLOUD");
      viewer.removeShape ("VRANSAC_LINE_MODEL");
    }
    //*/



    pcl::ModelCoefficients::Ptr test_coeffs (new pcl::ModelCoefficients ());
    test_coeffs->values.push_back (P1[0]);
    test_coeffs->values.push_back (P1[1]);
    test_coeffs->values.push_back (0.0);
    test_coeffs->values.push_back (P2[0] - P1[0]);
    test_coeffs->values.push_back (P2[1] - P1[1]);
    test_coeffs->values.push_back (0.0);

    if ( growing_visualization )
    {
      viewer.addLine (*test_coeffs, 1.0, 0.0, 0.5, "test_COEFFS");
      viewer.spin ();
      viewer.removeShape ("test_COEFFS");
    }



    // NOVEL 1 WAY OF GROWING BOXES //

    pcl::PointXYZRGBNormalRSD MiN, MaX;
    pcl::getMinMax3D (*vransac_line_cloud, MiN, MaX);

    pcl::PointIndices::Ptr novel_box_inliers (new pcl::PointIndices ());
    pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr novel_box_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    double M[2];
    M[0] = (P1[0] + P2[0]) / 2;
    M[1] = (P1[1] + P2[1]) / 2;

    //////////////////
    // From M To P1 //
    //////////////////

    bool em1_bool = true;

    double M1[2];
    M1[0] = M[0];
    M1[1] = M[1];

    do
    {

      double P2M1V[2];
      P2M1V[0] = M1[0] - P2[0];
      P2M1V[1] = M1[1] - P2[1];

      double P2M1N = sqrt (P2M1V[0]*P2M1V[0] + P2M1V[1]*P2M1V[1]);
      P2M1V[0] = P2M1V[0] / P2M1N;
      P2M1V[1] = P2M1V[1] / P2M1N;

      double EM1[2];
      EM1[0] = M1[0] + P2M1V[0] * growing_step;
      EM1[1] = M1[1] + P2M1V[1] * growing_step;

      pcl::ModelCoefficients::Ptr em1_coeffs (new pcl::ModelCoefficients ());
      em1_coeffs->values.push_back (EM1[0]);
      em1_coeffs->values.push_back (EM1[1]);
      em1_coeffs->values.push_back (0.0);
      em1_coeffs->values.push_back (M1[0] - EM1[0]);
      em1_coeffs->values.push_back (M1[1] - EM1[1]);
      em1_coeffs->values.push_back (0.0);

      if ( growing_visualization )
      {
        viewer.addLine (*em1_coeffs, 1.0, 0.0, 0.5, "EM1_COEFFS");
        viewer.spin ();
        viewer.removeShape ("EM1_COEFFS");
      }

      //

      pcl::PointIndices::Ptr em1_inliers (new pcl::PointIndices ());

      for (int idx = 0; idx < working_cloud->points.size (); idx++)
      {
        double P0[2];
        P0[0] = working_cloud->points.at (idx).x;
        P0[1] = working_cloud->points.at (idx).y;

        double num = (P0[0] - EM1[0])*(M1[0] - EM1[0]) + (P0[1] - EM1[1])*(M1[1] - EM1[1]);
        double den = _sqr (M1[0] - EM1[0])  +  _sqr (M1[1] - EM1[1]);
        double u = num / den;

        double I[2];
        I[0] = EM1[0] + u * (M1[0] - EM1[0]);
        I[1] = EM1[1] + u * (M1[1] - EM1[1]);

        double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

        if ( d < line_threshold )
        {
          double IEM1D  = sqrt ( _sqr  (I[0] - EM1[0])  +  _sqr  (I[1] - EM1[1]) );
          double IM1D   = sqrt ( _sqr  (I[0] -  M1[0])  +  _sqr  (I[1] -  M1[1]) );
          double EM1M1D = sqrt ( _sqr (M1[0] - EM1[0])  +  _sqr (M1[1] - EM1[1]) );

          if ( (IEM1D + IM1D) < (EM1M1D + 0.001) )
            em1_inliers->indices.push_back (idx);
        }
      }

      //

      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr em1_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> em1_ei;
      em1_ei.setInputCloud (working_cloud);
      em1_ei.setIndices (em1_inliers);
      em1_ei.setNegative (false);
      em1_ei.filter (*em1_cloud);

      if ( growing_visualization )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> em1_cloud_color (em1_cloud, 255, 0, 127);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (em1_cloud, em1_cloud_color, "EM1_CLOUD");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EM1_CLOUD");
        viewer.spin ();
        viewer.removePointCloud ("EM1_CLOUD");
      }

      //

      pcl::PointXYZRGBNormalRSD em1_MiN, em1_MaX;
      pcl::getMinMax3D (*em1_cloud, em1_MiN, em1_MaX);

      cerr << endl;
      cerr << "          MaX = " << MaX.z << endl;
      cerr << "      em1_MaX = " << em1_MaX.z << endl;
      cerr << "      em1_dif = " << fabs (MaX.z - em1_MaX.z) << endl;
      cerr << endl;

      double em1_dif = fabs (MaX.z - em1_MaX.z);

      if ( em1_dif < growing_height )
      {
        cerr << "      OK !" << endl ;

        M1[0] = EM1[0];
        M1[1] = EM1[1];

        //pcl::copyPointCloud (*em1_cloud, *novel_box_cloud);
        novel_box_cloud->points.insert (  novel_box_cloud->points.end(),        em1_cloud->points.begin(),         em1_cloud->points.end());
        novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     em1_inliers->indices.begin(),     em1_inliers->indices.end());
      }
      else
      {
        cerr << "      NOT OK !" << endl ;

        em1_bool = false;
      }

    } while ( em1_bool );

    cerr << endl << "   EXIT !" << endl << endl ;

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("NOVEL_BOX_CLOUD");
    }

    //////////////////
    // From M To P2 //
    //////////////////

    bool em2_bool = true;

    double M2[2];
    M2[0] = M[0];
    M2[1] = M[1];

    do
    {

      double P1M2V[2];
      P1M2V[0] = M2[0] - P1[0];
      P1M2V[1] = M2[1] - P1[1];

      double P1M2N = sqrt (P1M2V[0]*P1M2V[0] + P1M2V[1]*P1M2V[1]);
      P1M2V[0] = P1M2V[0] / P1M2N;
      P1M2V[1] = P1M2V[1] / P1M2N;

      double EM2[2];
      EM2[0] = M2[0] + P1M2V[0] * growing_step;
      EM2[1] = M2[1] + P1M2V[1] * growing_step;

      pcl::ModelCoefficients::Ptr em2_coeffs (new pcl::ModelCoefficients ());
      em2_coeffs->values.push_back (EM2[0]);
      em2_coeffs->values.push_back (EM2[1]);
      em2_coeffs->values.push_back (0.0);
      em2_coeffs->values.push_back (M2[0] - EM2[0]);
      em2_coeffs->values.push_back (M2[1] - EM2[1]);
      em2_coeffs->values.push_back (0.0);

      if ( growing_visualization )
      {
        viewer.addLine (*em2_coeffs, 1.0, 0.0, 0.5, "EM2_COEFFS");
        viewer.spin ();
        viewer.removeShape ("EM2_COEFFS");
      }

      //

      pcl::PointIndices::Ptr em2_inliers (new pcl::PointIndices ());

      for (int idx = 0; idx < working_cloud->points.size (); idx++)
      {
        double P0[2];
        P0[0] = working_cloud->points.at (idx).x;
        P0[1] = working_cloud->points.at (idx).y;

        double num = (P0[0] - EM2[0])*(M2[0] - EM2[0]) + (P0[1] - EM2[1])*(M2[1] - EM2[1]);
        double den = _sqr (M2[0] - EM2[0])  +  _sqr (M2[1] - EM2[1]);
        double u = num / den;

        double I[2];
        I[0] = EM2[0] + u * (M2[0] - EM2[0]);
        I[1] = EM2[1] + u * (M2[1] - EM2[1]);

        double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

        if ( d < line_threshold )
        {
          double IEM2D  = sqrt ( _sqr  (I[0] - EM2[0])  +  _sqr  (I[1] - EM2[1]) );
          double IM2D   = sqrt ( _sqr  (I[0] -  M2[0])  +  _sqr  (I[1] -  M2[1]) );
          double EM2M2D = sqrt ( _sqr (M2[0] - EM2[0])  +  _sqr (M2[1] - EM2[1]) );

          if ( (IEM2D + IM2D) < (EM2M2D + 0.001) )
            em2_inliers->indices.push_back (idx);
        }
      }

      //

      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr em2_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> em2_ei;
      em2_ei.setInputCloud (working_cloud);
      em2_ei.setIndices (em2_inliers);
      em2_ei.setNegative (false);
      em2_ei.filter (*em2_cloud);

      if ( growing_visualization )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> em2_cloud_color (em2_cloud, 255, 0, 127);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (em2_cloud, em2_cloud_color, "EM2_CLOUD");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EM2_CLOUD");
        viewer.spin ();
        viewer.removePointCloud ("EM2_CLOUD");
      }

      //

      pcl::PointXYZRGBNormalRSD em2_MiN, em2_MaX;
      pcl::getMinMax3D (*em2_cloud, em2_MiN, em2_MaX);

      cerr << endl;
      cerr << "          MaX = " << MaX.z << endl;
      cerr << "      em2_MaX = " << em2_MaX.z << endl;
      cerr << "      em2_dif = " << fabs (MaX.z - em2_MaX.z) << endl;
      cerr << endl;

      double em2_dif = fabs (MaX.z - em2_MaX.z);

      if ( em2_dif < growing_height )
      {
        cerr << "      OK !" << endl ;

        M2[0] = EM2[0];
        M2[1] = EM2[1];

        //pcl::copyPointCloud (*em2_cloud, *novel_box_cloud);
        novel_box_cloud->points.insert (  novel_box_cloud->points.end(),        em2_cloud->points.begin(),         em2_cloud->points.end());
        novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     em2_inliers->indices.begin(),     em2_inliers->indices.end());
      }
      else
      {
        cerr << "      NOT OK !" << endl ;

        em2_bool = false;
      }

    } while ( em2_bool );

    cerr << endl << "   EXIT !" << endl << endl ;

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("NOVEL_BOX_CLOUD");
    }

    /*
       pcl::PointIndices::Ptr g_vransac_line_inliers (new pcl::PointIndices ());
     *g_vransac_line_inliers = *vransac_line_inliers;

     float R = 0.010;
     std::vector<int> pointIdxRadiusSearch;
     std::vector<float> pointRadiusSquaredDistance;

     pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD>::Ptr g_tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD> ());
     g_tree->setInputCloud (working_cloud);

     std::vector<bool> g_visited_points (working_cloud->points.size ());
     for (int vis = 0; vis < g_visited_points.size (); vis++)
     g_visited_points.at (vis) = false;

     size_t gidx = 0;

     do
     {
     pcl::PointXYZRGBNormalRSD searchPoint;
     searchPoint = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx));

     if ( g_visited_points.at (g_vransac_line_inliers->indices.at (gidx)) == false )
     {
     g_visited_points.at (g_vransac_line_inliers->indices.at (gidx)) = true;

     double P0[2];
     P0[0] = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx)).x;
     P0[1] = working_cloud->points.at (g_vransac_line_inliers->indices.at (gidx)).y;

     double num = (P0[0] - P1[0])*(P2[0] - P1[0]) + (P0[1] - P1[1])*(P2[1] - P1[1]);
     double den = _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]);
     double u = num / den;

     double I[2];
     I[0] = P1[0] + u * (P2[0] - P1[0]);
     I[1] = P1[1] + u * (P2[1] - P1[1]);

     double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

     if ( d < line_threshold )
     {
     if ( g_tree->radiusSearch (searchPoint, R, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
     {
     for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
     g_vransac_line_inliers->indices.push_back (pointIdxRadiusSearch.at (i));
     }
     }
     }

     gidx++;

     } while ( gidx < g_vransac_line_inliers->indices.size () );

     *vransac_line_inliers = *g_vransac_line_inliers;

     cerr << " vransac_line_inliers = " << vransac_line_inliers->indices.size () << endl;

     sort (g_vransac_line_inliers->indices.begin(), g_vransac_line_inliers->indices.end());
     g_vransac_line_inliers->indices.erase (unique (g_vransac_line_inliers->indices.begin(), g_vransac_line_inliers->indices.end()), g_vransac_line_inliers->indices.end());

     pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr g_vransac_line_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

     pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> g_l_ei;
     g_l_ei.setInputCloud (working_cloud);
     g_l_ei.setIndices (g_vransac_line_inliers);
     g_l_ei.setNegative (false);
     g_l_ei.filter (*g_vransac_line_cloud);

    // Adjust Coefficients Of Line Model //
    adjustLineCoefficients (g_vransac_line_cloud, vransac_line_coefficients);

    if ( space_step )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> g_vransac_line_cloud_color (g_vransac_line_cloud, 0, 255, 255);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (g_vransac_line_cloud, g_vransac_line_cloud_color, "G_VRANSAC_LINE_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "G_VRANSAC_LINE_CLOUD");
      viewer.addLine (*vransac_line_coefficients, 0.0, 1.0, 1.0, "VRANSAC_LINE_MODEL");
      viewer.spin ();
      viewer.removePointCloud ("G_VRANSAC_LINE_CLOUD");
      viewer.removeShape ("VRANSAC_LINE_MODEL");
    }
    */

      // Obtain The Scanning Viewpoint //

      double O[3];
    O[0] = 0.0;
    O[1] = 0.0;
    O[2] = 0.0;

    double VP[3];
    double T[4][4];

    rX = DEG2RAD (rX); rY = DEG2RAD (rY); rZ = DEG2RAD (rZ);
    computeTransformationMatrix (tX, tY, tZ, rX, rY, rZ, T);
    transform (T, O, VP);

    if ( !till_the_end )
      if ( true )
      {
        pcl::PointXYZ CVP;
        CVP.x = VP[0];
        CVP.y = VP[1];
        CVP.z = VP[2];

        viewer.addSphere<pcl::PointXYZ> (CVP, 0.010, 1.0, 1.0, 0.0, "VIEWPOINT_SPHERE");
        viewer.setShapeRenderingProperties (pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "VIEWPOINT_SPHERE");
        viewer.spin ();
        viewer.removeShape ("VIEWPOINT_SPHERE");
      }

    // Determine The Normal To Line //

    double dx = P2[0] - P1[0];
    double dy = P2[1] - P1[1];

    double N1[2];
    N1[0] = -dy;
    N1[1] =  dx;

    double PN1[2];
    PN1[0] =  ((P1[0] + P2[0]) / 2) + N1[0]*0.25;
    PN1[1] =  ((P1[1] + P2[1]) / 2) + N1[1]*0.25;

    double N2[2];
    N2[0] =  dy;
    N2[1] = -dx;

    double PN2[2];
    PN2[0] =  ((P1[0] + P2[0]) / 2) + N2[0]*0.25;
    PN2[1] =  ((P1[1] + P2[1]) / 2) + N2[1]*0.25;

    if ( false )
    {
      pcl::ModelCoefficients::Ptr first_normal_coefficients (new pcl::ModelCoefficients ());
      first_normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
      first_normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
      first_normal_coefficients->values.push_back (0.0);
      first_normal_coefficients->values.push_back (PN1[0] - ((P1[0] + P2[0]) / 2));
      first_normal_coefficients->values.push_back (PN1[1] - ((P1[1] + P2[1]) / 2));
      first_normal_coefficients->values.push_back (0.0);

      pcl::ModelCoefficients::Ptr second_normal_coefficients (new pcl::ModelCoefficients ());
      second_normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
      second_normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
      second_normal_coefficients->values.push_back (0.0);
      second_normal_coefficients->values.push_back (PN2[0] - ((P1[0] + P2[0]) / 2));
      second_normal_coefficients->values.push_back (PN2[1] - ((P1[1] + P2[1]) / 2));
      second_normal_coefficients->values.push_back (0.0);

      viewer.addLine (*first_normal_coefficients, 1.0, 0.0, 1.0, "FIRST_NORMAL_COEFFICIENTS");
      viewer.addLine (*second_normal_coefficients, 1.0, 0.0, 1.0, "SECOND_NORMAL_COEFFICIENTS");
      viewer.spin ();
      viewer.removeShape ("FIRST_NORMAL_COEFFICIENTS");
      viewer.removeShape ("SECOND_NORMAL_COEFFICIENTS");
    }

    double N[2];

    double PN1VP = sqrt ( _sqr (PN1[0] - VP[0]) + _sqr (PN1[1] - VP[1]) );
    double PN2VP = sqrt ( _sqr (PN2[0] - VP[0]) + _sqr (PN2[1] - VP[1]) );

    if ( PN1VP < PN2VP )
    {
      N[0] = N2[0];
      N[1] = N2[1];
    }
    else
    {
      N[0] = N1[0];
      N[1] = N1[1];
    }

    if ( !till_the_end )
      if ( true )
      {
        pcl::ModelCoefficients::Ptr normal_coefficients (new pcl::ModelCoefficients ());
        normal_coefficients->values.push_back ((P1[0] + P2[0]) / 2);
        normal_coefficients->values.push_back ((P1[1] + P2[1]) / 2);
        normal_coefficients->values.push_back (0.0);
        normal_coefficients->values.push_back (N[0]);
        normal_coefficients->values.push_back (N[1]);
        normal_coefficients->values.push_back (0.0);

        viewer.addLine (*normal_coefficients, 1.0, 0.0, 1.0, "NORMAL_COEFFICIENTS");
        viewer.spin ();
        viewer.removeShape ("NORMAL_COEFFICIENTS");
      }

    double NN = sqrt ( N[0]*N[0] + N[1]*N[1] );
    N[0] = N[0] / NN;
    N[1] = N[1] / NN;

    // NOVEL 2 WAY OF GROWING BOXES //

    double NP1[2];
    double NP2[2];

    /////////////////
    // To The Back //
    /////////////////

    bool en1_bool = true;

    NP1[0] = M1[0];
    NP1[1] = M1[1];

    NP2[0] = M2[0];
    NP2[1] = M2[1];

    do
    {
      double EN1[2];
      EN1[0] = NP1[0] + N[0] * growing_step /2;
      EN1[1] = NP1[1] + N[1] * growing_step /2;

      double EN2[2];
      EN2[0] = NP2[0] + N[0] * growing_step /2;
      EN2[1] = NP2[1] + N[1] * growing_step /2;

      pcl::ModelCoefficients::Ptr en1_coeffs (new pcl::ModelCoefficients ());
      en1_coeffs->values.push_back (EN1[0]);
      en1_coeffs->values.push_back (EN1[1]);
      en1_coeffs->values.push_back (0.0);
      en1_coeffs->values.push_back (EN2[0] - EN1[0]);
      en1_coeffs->values.push_back (EN2[1] - EN1[1]);
      en1_coeffs->values.push_back (0.0);

      if ( growing_visualization )
      {
        viewer.addLine (*en1_coeffs, 1.0, 0.0, 0.5, "EN_COEFFS");
        viewer.spin ();
        viewer.removeShape ("EN_COEFFS");
      }

      //

      pcl::PointIndices::Ptr en1_inliers (new pcl::PointIndices ());

      for (int idx = 0; idx < working_cloud->points.size (); idx++)
      {
        double P0[2];
        P0[0] = working_cloud->points.at (idx).x;
        P0[1] = working_cloud->points.at (idx).y;

        double num = (P0[0] - EN1[0])*(EN2[0] - EN1[0]) + (P0[1] - EN1[1])*(EN2[1] - EN1[1]);
        double den = _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]);
        double u = num / den;

        double I[2];
        I[0] = EN1[0] + u * (EN2[0] - EN1[0]);
        I[1] = EN1[1] + u * (EN2[1] - EN1[1]);

        double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

        if ( d < growing_step )
        {
          double IEN1D  = sqrt ( _sqr  (I[0] - EN1[0])  +  _sqr  (I[1] - EN1[1]) );
          double IEN2D  = sqrt ( _sqr  (I[0] - EN2[0])  +  _sqr  (I[1] - EN2[1]) );
          double EN1EN2D = sqrt ( _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]) );

          if ( (IEN1D + IEN2D) < (EN1EN2D + 0.001) )
            en1_inliers->indices.push_back (idx);
        }
      }

      //

      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr en1_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> en1_ei;
      en1_ei.setInputCloud (working_cloud);
      en1_ei.setIndices (en1_inliers);
      en1_ei.setNegative (false);
      en1_ei.filter (*en1_cloud);

      if ( growing_visualization )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> en1_cloud_color (en1_cloud, 255, 0, 127);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (en1_cloud, en1_cloud_color, "EN_CLOUD");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EN_CLOUD");
        viewer.spin ();
        viewer.removePointCloud ("EN_CLOUD");
      }

      //

      pcl::PointXYZRGBNormalRSD en1_MiN, en1_MaX;
      pcl::getMinMax3D (*en1_cloud, en1_MiN, en1_MaX);

      cerr << endl;
      cerr << "          MaX = " << MaX.z << endl;
      cerr << "      en1_MaX = " << en1_MaX.z << endl;
      cerr << "      en1_dif = " << fabs (MaX.z - en1_MaX.z) << endl;
      cerr << endl;

      double en1_dif = fabs (MaX.z - en1_MaX.z);

      if ( en1_dif < growing_height )
      {
        cerr << "      OK !" << endl ;

        NP1[0] = EN1[0];
        NP1[1] = EN1[1];

        NP2[0] = EN2[0];
        NP2[1] = EN2[1];

        //pcl::copyPointCloud (*en1_cloud, *novel_box_cloud);
        novel_box_cloud->points.insert (   novel_box_cloud->points.end(),        en1_cloud->points.begin(),        en1_cloud->points.end());
        novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     en1_inliers->indices.begin(),     en1_inliers->indices.end());
      }
      else
      {
        cerr << "      NOT OK !" << endl ;

        en1_bool = false;
      }

    } while ( en1_bool );

    cerr << endl << "   EXIT !" << endl << endl ;

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("NOVEL_BOX_CLOUD");
    }

    //////////////////
    // To The Front //
    //////////////////

    bool en2_bool = true;

    NP1[0] = M1[0];
    NP1[1] = M1[1];

    NP2[0] = M2[0];
    NP2[1] = M2[1];

    do
    {
      double EN1[2];
      EN1[0] = NP1[0] - N[0] * growing_step /2;
      EN1[1] = NP1[1] - N[1] * growing_step /2;

      double EN2[2];
      EN2[0] = NP2[0] - N[0] * growing_step /2;
      EN2[1] = NP2[1] - N[1] * growing_step /2;

      pcl::ModelCoefficients::Ptr en2_coeffs (new pcl::ModelCoefficients ());
      en2_coeffs->values.push_back (EN1[0]);
      en2_coeffs->values.push_back (EN1[1]);
      en2_coeffs->values.push_back (0.0);
      en2_coeffs->values.push_back (EN2[0] - EN1[0]);
      en2_coeffs->values.push_back (EN2[1] - EN1[1]);
      en2_coeffs->values.push_back (0.0);

      if ( growing_visualization )
      {
        viewer.addLine (*en2_coeffs, 1.0, 0.0, 0.5, "EN_COEFFS");
        viewer.spin ();
        viewer.removeShape ("EN_COEFFS");
      }

      //

      pcl::PointIndices::Ptr en2_inliers (new pcl::PointIndices ());

      for (int idx = 0; idx < working_cloud->points.size (); idx++)
      {
        double P0[2];
        P0[0] = working_cloud->points.at (idx).x;
        P0[1] = working_cloud->points.at (idx).y;

        double num = (P0[0] - EN1[0])*(EN2[0] - EN1[0]) + (P0[1] - EN1[1])*(EN2[1] - EN1[1]);
        double den = _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]);
        double u = num / den;

        double I[2];
        I[0] = EN1[0] + u * (EN2[0] - EN1[0]);
        I[1] = EN1[1] + u * (EN2[1] - EN1[1]);

        double d = sqrt ( _sqr (I[0] - P0[0])  +  _sqr (I[1] - P0[1]) );

        if ( d < growing_step )
        {
          double IEN1D  = sqrt ( _sqr  (I[0] - EN1[0])  +  _sqr  (I[1] - EN1[1]) );
          double IEN2D  = sqrt ( _sqr  (I[0] - EN2[0])  +  _sqr  (I[1] - EN2[1]) );
          double EN1EN2D = sqrt ( _sqr (EN2[0] - EN1[0])  +  _sqr (EN2[1] - EN1[1]) );

          if ( (IEN1D + IEN2D) < (EN1EN2D + 0.001) )
            en2_inliers->indices.push_back (idx);
        }
      }

      //

      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr en2_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> en2_ei;
      en2_ei.setInputCloud (working_cloud);
      en2_ei.setIndices (en2_inliers);
      en2_ei.setNegative (false);
      en2_ei.filter (*en2_cloud);

      if ( growing_visualization )
      {
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> en2_cloud_color (en2_cloud, 255, 0, 127);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (en2_cloud, en2_cloud_color, "EN_CLOUD");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "EN_CLOUD");
        viewer.spin ();
        viewer.removePointCloud ("EN_CLOUD");
      }

      //

      pcl::PointXYZRGBNormalRSD en2_MiN, en2_MaX;
      pcl::getMinMax3D (*en2_cloud, en2_MiN, en2_MaX);

      cerr << endl;
      cerr << "          MaX = " << MaX.z << endl;
      cerr << "      en2_MaX = " << en2_MaX.z << endl;
      cerr << "      en2_dif = " << fabs (MaX.z - en2_MaX.z) << endl;
      cerr << endl;

      double en2_dif = fabs (MaX.z - en2_MaX.z);

      if ( en2_dif < growing_height )
      {
        cerr << "      OK !" << endl ;

        NP1[0] = EN1[0];
        NP1[1] = EN1[1];

        NP2[0] = EN2[0];
        NP2[1] = EN2[1];

        //pcl::copyPointCloud (*en2_cloud, *novel_box_cloud);
        novel_box_cloud->points.insert (   novel_box_cloud->points.end(),        en2_cloud->points.begin(),        en2_cloud->points.end());
        novel_box_inliers->indices.insert (novel_box_inliers->indices.end(),     en2_inliers->indices.begin(),     en2_inliers->indices.end());
      }
      else
      {
        cerr << "      NOT OK !" << endl ;

        en2_bool = false;
      }

    } while ( en2_bool );

    cerr << endl << "   EXIT !" << endl << endl ;

    if ( growing_visualization )
    {
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> novel_box_cloud_color (novel_box_cloud, 255, 0, 127);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (novel_box_cloud, novel_box_cloud_color, "NOVEL_BOX_CLOUD");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "NOVEL_BOX_CLOUD");
      viewer.spin ();
      viewer.removePointCloud ("NOVEL_BOX_CLOUD");
    }



    /*

    // Obtain Points Of Box //

    pcl::PointIndices::Ptr box_inliers (new pcl::PointIndices ());
     *box_inliers = *vransac_line_inliers;

     int K = 10;
     std::vector<int> pointIdxNKNSearch(K);
     std::vector<float> pointNKNSquaredDistance(K);

     pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD>::Ptr box_tree (new pcl::KdTreeFLANN<pcl::PointXYZRGBNormalRSD> ());
     box_tree->setInputCloud (working_cloud);

     std::vector<bool> visited_points (working_cloud->points.size ());
     for (int vis = 0; vis < visited_points.size (); vis++)
     visited_points.at (vis) = false;

     size_t idx = 0;

     pcl::PointXYZRGBNormalRSD min_of_vlc, max_of_vlc;
     pcl::getMinMax3D (*vransac_line_cloud, min_of_vlc, max_of_vlc);

     do
     {
     pcl::PointXYZRGBNormalRSD searchPoint;
     searchPoint = working_cloud->points.at (box_inliers->indices.at (idx));

     if ( visited_points.at (box_inliers->indices.at (idx)) == false )
     {
     visited_points.at (box_inliers->indices.at (idx)) = true;

     double P0[2];
     P0[0] = working_cloud->points.at (box_inliers->indices.at (idx)).x;
     P0[1] = working_cloud->points.at (box_inliers->indices.at (idx)).y;
     P0[2] = working_cloud->points.at (box_inliers->indices.at (idx)).z;

     if ( ( P0[2] < (max_of_vlc.z + 0.005) ) && ( P0[2] > (min_of_vlc.z - 0.005) ) )
     {
     double num = (P0[0] - P1[0])*(P2[0] - P1[0]) + (P0[1] - P1[1])*(P2[1] - P1[1]);
     double den = _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]);
     double u = num / den;

     double I[2];
     I[0] = P1[0] + u * (P2[0] - P1[0]);
     I[1] = P1[1] + u * (P2[1] - P1[1]);

     double  IP1D = sqrt ( _sqr  (I[0] - P1[0])  +  _sqr  (I[1] - P1[1]) );
     double  IP2D = sqrt ( _sqr  (I[0] - P2[0])  +  _sqr  (I[1] - P2[1]) );
     double P1P2D = sqrt ( _sqr (P2[0] - P1[0])  +  _sqr (P2[1] - P1[1]) );

     if ( (IP1D + IP2D) < (P1P2D + 0.001) )
     {
     double  IVPD  = sqrt ( _sqr   (I[0] - VP[0])  +  _sqr   (I[1] - VP[1]) );
     double P0VPD  = sqrt ( _sqr  (P0[0] - VP[0])  +  _sqr  (P0[1] - VP[1]) );

     if ( P0VPD > IVPD )
     {
     if ( box_tree->nearestKSearch (searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
     {
     for (size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
     box_inliers->indices.push_back (pointIdxNKNSearch.at (i));
     }
     }
     }
     }
     }

     idx++;

     } while ( idx < box_inliers->indices.size() );

    sort (box_inliers->indices.begin(), box_inliers->indices.end());
    box_inliers->indices.erase (unique (box_inliers->indices.begin(), box_inliers->indices.end()), box_inliers->indices.end());

    */

      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr box_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

    pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> b_ei;
    b_ei.setInputCloud (working_cloud);
    b_ei.setIndices (novel_box_inliers);

    b_ei.setNegative (false);
    b_ei.filter (*box_cloud);

    b_ei.setNegative (true);
    b_ei.filter (*working_cloud);

    //

    if ( working_cloud->points.size () > 0 )
    {
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormalRSD> sor;
      sor.setInputCloud (working_cloud);
      sor.setMeanK (mean_k_filter);
      sor.setStddevMulThresh (std_dev_filter *5);
      sor.filter (*working_cloud);
    }

    //

    //////if ( space_step )
    //////{
    //////std::stringstream box_cloud_id;
    //////box_cloud_id << "BOX_CLOUD_" << getTimestamp ();
    //////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> box_cloud_color (box_cloud, 127, 0, 255);
    //////viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (box_cloud, box_cloud_color, box_cloud_id.str ());
    //////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, box_cloud_id.str ());
    //////viewer.spin ();
    //////
    //////viewer.removePointCloud ("generic");
    //////pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
    //////viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "generic");
    //////viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
    //////viewer.spin ();
    //////}

    // ADD BOX SHAPE //

    double p1[3];
    p1[0] = M1[0];
    p1[1] = M1[1];
    p1[2] = 0.0;

    double p2[3];
    p2[0] = M2[0];
    p2[1] = M2[1];
    p2[2] = 0.0;

    /// Vector of Line ///
    double l[3];
    l[0] = p2[0] - p1[0];
    l[1] = p2[1] - p1[1];
    l[2] = 0.0;

    /// Normalize Vector of Line ///
    double nl = sqrt ( _sqr(l[0]) + _sqr(l[1]) + _sqr(l[2]) );
    l[0] = l[0] / nl;
    l[1] = l[1] / nl;
    l[2] = l[2] / nl;

    /// Unit Vector of Z Axis ///
    double n[3];
    n[0] = 0.0;
    n[1] = 0.0;
    n[2] = 1.0;

    /// Dot Product ///
    double m[3];
    m[0] = l[1]*n[2] - l[2]*n[1];
    m[1] = l[2]*n[0] - l[0]*n[2];
    m[2] = l[0]*n[1] - l[1]*n[0];

    // Length- and Cross- Vectors //
    double vectors[2][2];
    vectors[0][0] = l[0];
    vectors[0][1] = l[1];
    vectors[1][0] = m[0];
    vectors[1][1] = m[1];

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid (*box_cloud, centroid);

    double max_u = -DBL_MAX;
    double min_u =  DBL_MAX;
    double max_v = -DBL_MAX;
    double min_v =  DBL_MAX;

    /*

    // Bounding box only for line inliers //

    for ( int inl = 0; inl < (int) shapes_lines_inliers.at (clu)->indices.size(); inl++ )
    {
    int idx = shapes_lines_inliers.at (clu)->indices.at (inl);

    double c2p[2];
    c2p[0] = shapes_lines_clusters.at (clu)->points.at (idx).x - centroid[0];
    c2p[1] = shapes_lines_clusters.at (clu)->points.at (idx).y - centroid[1];

    double width = vectors[0][0]*c2p[0] + vectors[0][1]*c2p[1];
    if (width > max_u) max_u = width;
    if (width < min_u) min_u = width;

    double length = vectors[1][0]*c2p[0] + vectors[1][1]*c2p[1];
    if (length > max_v) max_v = length;
    if (length < min_v) min_v = length;
    }

*/

    // Bounding box for whole cluster //

    for ( int idx = 0; idx < (int) box_cloud->points.size(); idx++ )
    {
      double c2p[2];
      c2p[0] = box_cloud->points.at (idx).x - centroid[0];
      c2p[1] = box_cloud->points.at (idx).y - centroid[1];

      double width = vectors[0][0]*c2p[0] + vectors[0][1]*c2p[1];
      if (width > max_u) max_u = width;
      if (width < min_u) min_u = width;

      double length = vectors[1][0]*c2p[0] + vectors[1][1]*c2p[1];
      if (length > max_v) max_v = length;
      if (length < min_v) min_v = length;
    }

    double minimus = +DBL_MAX;
    double maximus = -DBL_MAX;

    /*

    // Bounding box only for line inliers //

    for ( int inl = 0; inl < (int) shapes_lines_inliers.at (clu)->indices.size(); inl++ )
    {
    int idx = shapes_lines_inliers.at (clu)->indices.at (inl);

    double Z = shapes_lines_clusters.at (clu)->points.at (idx).z;

    if ( minimus > Z ) minimus = Z;
    if ( maximus < Z ) maximus = Z;
    }

*/

    // Bounding box for whole cluster //

    for ( int idx = 0; idx < (int) box_cloud->points.size(); idx++ )
    {
      double Z = box_cloud->points.at (idx).z;

      if ( minimus > Z ) minimus = Z;
      if ( maximus < Z ) maximus = Z;
    }

    // The edges //

    double edges[4][2];

    edges[0][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*max_v;
    edges[0][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*max_v;

    edges[1][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*min_v;
    edges[1][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*min_v;

    edges[2][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*min_v;
    edges[2][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*min_v;

    edges[3][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*max_v;
    edges[3][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*max_v;

    //////    minimus = working_cloud_minimum.z;

    if ( shapes_from_table_plane )
      minimus = abs_min.z;

    pcl::ModelCoefficients e0, e1, e2, e3;

    e0.values.push_back (edges[0][0]);
    e0.values.push_back (edges[0][1]);
    e0.values.push_back (minimus);
    e0.values.push_back (edges[1][0] - edges[0][0]);
    e0.values.push_back (edges[1][1] - edges[0][1]);
    e0.values.push_back (minimus - minimus);

    e1.values.push_back (edges[1][0]);
    e1.values.push_back (edges[1][1]);
    e1.values.push_back (minimus);
    e1.values.push_back (edges[2][0] - edges[1][0]);
    e1.values.push_back (edges[2][1] - edges[1][1]);
    e1.values.push_back (minimus - minimus);

    e2.values.push_back (edges[2][0]);
    e2.values.push_back (edges[2][1]);
    e2.values.push_back (minimus);
    e2.values.push_back (edges[3][0] - edges[2][0]);
    e2.values.push_back (edges[3][1] - edges[2][1]);
    e2.values.push_back (minimus - minimus);

    e3.values.push_back (edges[3][0]);
    e3.values.push_back (edges[3][1]);
    e3.values.push_back (minimus);
    e3.values.push_back (edges[0][0] - edges[3][0]);
    e3.values.push_back (edges[0][1] - edges[3][1]);
    e3.values.push_back (minimus - minimus);

    /*

       std::stringstream line_0;
       line_0 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e0);
    line_viewer.addLine (e0, line_0.str ());
    line_viewer.spin ();

    std::stringstream line_1;
    line_1 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e1);
    line_viewer.addLine (e1, line_1.str ());
    line_viewer.spin ();

    std::stringstream line_2;
    line_2 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e2);
    line_viewer.addLine (e2, line_2.str ());
    line_viewer.spin ();

    std::stringstream line_3;
    line_3 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e3);
    line_viewer.addLine (e3, line_3.str ());
    line_viewer.spin ();

*/

    pcl::ModelCoefficients e4, e5, e6, e7;

    e4.values.push_back (edges[0][0]);
    e4.values.push_back (edges[0][1]);
    e4.values.push_back (maximus);
    e4.values.push_back (edges[1][0] - edges[0][0]);
    e4.values.push_back (edges[1][1] - edges[0][1]);
    e4.values.push_back (maximus - maximus);

    e5.values.push_back (edges[1][0]);
    e5.values.push_back (edges[1][1]);
    e5.values.push_back (maximus);
    e5.values.push_back (edges[2][0] - edges[1][0]);
    e5.values.push_back (edges[2][1] - edges[1][1]);
    e5.values.push_back (maximus - maximus);

    e6.values.push_back (edges[2][0]);
    e6.values.push_back (edges[2][1]);
    e6.values.push_back (maximus);
    e6.values.push_back (edges[3][0] - edges[2][0]);
    e6.values.push_back (edges[3][1] - edges[2][1]);
    e6.values.push_back (maximus - maximus);

    e7.values.push_back (edges[3][0]);
    e7.values.push_back (edges[3][1]);
    e7.values.push_back (maximus);
    e7.values.push_back (edges[0][0] - edges[3][0]);
    e7.values.push_back (edges[0][1] - edges[3][1]);
    e7.values.push_back (maximus - maximus);

    /*

       std::stringstream line_4;
       line_4 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e4);
    line_viewer.addLine (e4, line_4.str ());
    line_viewer.spin ();

    std::stringstream line_5;
    line_5 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e5);
    line_viewer.addLine (e5, line_5.str ());
    line_viewer.spin ();

    std::stringstream line_6;
    line_6 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e6);
    line_viewer.addLine (e6, line_6.str ());
    line_viewer.spin ();

    std::stringstream line_7;
    line_7 << "LINE_" << ros::Time::now();
    //adjustLine (shapes_lines_clusters.at (clu), e7);
    line_viewer.addLine (e7, line_7.str ());
    line_viewer.spin ();

*/

    std::vector<pcl::ModelCoefficients> cub;

    cub.push_back (e0);
    cub.push_back (e1);
    cub.push_back (e2);
    cub.push_back (e3);
    cub.push_back (e4);
    cub.push_back (e5);
    cub.push_back (e6);
    cub.push_back (e7);

    cerr << e0.values.at(0) << " " << e0.values.at(1) << " " << e0.values.at(2) << " " << e0.values.at(3) << " " << e0.values.at(4) << " " << e0.values.at(5) << endl ;
    cerr << e1.values.at(0) << " " << e1.values.at(1) << " " << e1.values.at(2) << " " << e1.values.at(3) << " " << e1.values.at(4) << " " << e1.values.at(5) << endl ;
    cerr << e2.values.at(0) << " " << e2.values.at(1) << " " << e2.values.at(2) << " " << e2.values.at(3) << " " << e2.values.at(4) << " " << e2.values.at(5) << endl ;
    cerr << e3.values.at(0) << " " << e3.values.at(1) << " " << e3.values.at(2) << " " << e3.values.at(3) << " " << e3.values.at(4) << " " << e3.values.at(5) << endl ;
    cerr << e4.values.at(0) << " " << e4.values.at(1) << " " << e4.values.at(2) << " " << e4.values.at(3) << " " << e4.values.at(4) << " " << e4.values.at(5) << endl ;
    cerr << e5.values.at(0) << " " << e5.values.at(1) << " " << e5.values.at(2) << " " << e5.values.at(3) << " " << e5.values.at(4) << " " << e5.values.at(5) << endl ;
    cerr << e6.values.at(0) << " " << e6.values.at(1) << " " << e6.values.at(2) << " " << e6.values.at(3) << " " << e6.values.at(4) << " " << e6.values.at(5) << endl ;
    cerr << e7.values.at(0) << " " << e7.values.at(1) << " " << e7.values.at(2) << " " << e7.values.at(3) << " " << e7.values.at(4) << " " << e7.values.at(5) << endl ;

    // NOVEL 3 WAY OF GROWING BOXES //

    //////std::stringstream cub_id;
    //////cub_id << "CUB_" << getTimestamp ();
    //////viewer.addCuboid (cub, 0.5, 0.0, 1.0, 0.5, cub_id.str ());

    model++;
    cerr << endl << " MODEL " << model << endl << endl ;

    if ( space_step ) viewer.spin ();

     //                //
    // Classification //
   //                //

    if ( classification )
    {

      //double x_surface = sqrt (_sqr (e0.values[0]-e1.values[0]) + _sqr (e0.values[1]-e1.values[1]) + _sqr (e0.values[2]-e1.values[2])) * sqrt (_sqr (e1.values[0]-e5.values[0]) + _sqr (e1.values[1]-e5.values[1]) + _sqr (e1.values[2]-e5.values[2])) ;
      //double y_surface = sqrt (_sqr (e1.values[0]-e2.values[0]) + _sqr (e1.values[1]-e2.values[1]) + _sqr (e1.values[2]-e2.values[2])) * sqrt (_sqr (e2.values[0]-e6.values[0]) + _sqr (e2.values[1]-e6.values[1]) + _sqr (e2.values[2]-e6.values[2])) ;
      //double z_surface = sqrt (_sqr (e0.values[0]-e1.values[0]) + _sqr (e0.values[1]-e1.values[1]) + _sqr (e0.values[2]-e1.values[2])) * sqrt (_sqr (e1.values[0]-e2.values[0]) + _sqr (e1.values[1]-e2.values[1]) + _sqr (e1.values[2]-e2.values[2])) ;

      double x_dist = sqrt (_sqr (e0.values[0]-e1.values[0]) + _sqr (e0.values[1]-e1.values[1]) + _sqr (e0.values[2]-e1.values[2])) ;
      double y_dist = sqrt (_sqr (e1.values[0]-e2.values[0]) + _sqr (e1.values[1]-e2.values[1]) + _sqr (e1.values[2]-e2.values[2])) ;
      double z_dist = sqrt (_sqr (e2.values[0]-e6.values[0]) + _sqr (e2.values[1]-e6.values[1]) + _sqr (e2.values[2]-e6.values[2])) ;



              double x_cen = (e0.values[0] + e6.values[0]) / 2;
              double y_cen = (e0.values[1] + e6.values[1]) / 2;
              double z_cen = (e0.values[2] + e6.values[2]) / 2;

              std::cerr << "          center " << "(" << x_cen << "," << y_cen << "," << z_cen << ")" << std::endl;



      // Create file name for saving
      std::stringstream object_filename;

      //double min_surface = std::min (x_surface, std::min (y_surface, z_surface));

      //cerr << " x_surface " << x_surface << endl ;
      //cerr << " y_surface " << y_surface << endl ;
      //cerr << " z_surface " << z_surface << endl ;

      double smallest_dimension = std::min (x_dist, std::min (y_dist, z_dist));
      double  biggest_dimension = std::max (x_dist, std::max (y_dist, z_dist));
      double   medium_dimension = (smallest_dimension==x_dist) ? (biggest_dimension==z_dist ? y_dist:z_dist) : (biggest_dimension==z_dist ? y_dist:x_dist);
      cerr << smallest_dimension << " < " << medium_dimension << " < " << biggest_dimension << endl;

      cerr << " x_dist " << x_dist << endl ;
      cerr << " y_dist " << y_dist << endl ;
      cerr << " z_dist " << z_dist << endl ;

      double min_dist = std::min (x_dist, y_dist);
      double max_dist = std::max (x_dist, y_dist);

      cerr << " max / min " << max_dist / min_dist << endl ;
      cerr << " max / z " << max_dist / z_dist << endl ;

      cerr << "flat_value = " << flat_value << endl ;
      cerr << "a/(b+c) = " << smallest_dimension / (medium_dimension + biggest_dimension) << endl ;

      // TODO This is NOT the optim way, just a way // Begin //
      ///*
      pcl::PointIndices::Ptr backup_novel_box_inliers (new pcl::PointIndices ());

      for (int idx = 0; idx < (int) backup_working_cloud->points.size(); idx++)
      {
        //double x = backup_working_cloud->points.at (idx).x;
        //double y = backup_working_cloud->points.at (idx).y;
        //
        //double sqr_dist_to_e0 = _sqr (x - edges[0][0]) + _sqr (y - edges[0][1]);
        //double sqr_dist_to_e1 = _sqr (x - edges[1][0]) + _sqr (y - edges[1][1]);
        //double sqr_dist_to_e2 = _sqr (x - edges[2][0]) + _sqr (y - edges[2][1]);
        //double sqr_dist_to_e3 = _sqr (x - edges[3][0]) + _sqr (y - edges[3][1]);
        //double sum_of_sqr_dist = sqr_dist_to_e0 + sqr_dist_to_e1 + sqr_dist_to_e2 + sqr_dist_to_e3;
        //
        //double sqr_dist_from_e0_to_e1 = _sqr (edges[0][0] - edges[1][0]) + _sqr (edges[0][1] - edges[1][1]);
        //double sqr_dist_from_e1_to_e2 = _sqr (edges[1][0] - edges[2][0]) + _sqr (edges[1][1] - edges[2][1]);
        //double sqr_dist_from_e2_to_e3 = _sqr (edges[2][0] - edges[3][0]) + _sqr (edges[2][1] - edges[3][1]);
        //double sqr_dist_from_e3_to_e0 = _sqr (edges[3][0] - edges[0][0]) + _sqr (edges[3][1] - edges[0][1]);
        //double sum_of_sqr_rect = sqr_dist_from_e0_to_e1 + sqr_dist_from_e1_to_e2 + sqr_dist_from_e2_to_e3 + sqr_dist_from_e3_to_e0;
        //
        //if ( sum_of_sqr_dist < sum_of_sqr_rect )
        //{
        //// Save only the right indices
        //backup_novel_box_inliers->indices.push_back (idx);
        //}

        pcl::PointXYZ B;
        B.x = backup_working_cloud->points.at (idx).x;
        B.y = backup_working_cloud->points.at (idx).y;

        pcl::PointXYZ E0;  E0.x = edges[0][0];  E0.y = edges[0][1];
        pcl::PointXYZ E1;  E1.x = edges[1][0];  E1.y = edges[1][1];
        pcl::PointXYZ E2;  E2.x = edges[2][0];  E2.y = edges[2][1];
        pcl::PointXYZ E3;  E3.x = edges[3][0];  E3.y = edges[3][1];

        double A0 = whatAngle (E0, B, E1);
        double A1 = whatAngle (E1, B, E2);
        double A2 = whatAngle (E2, B, E3);
        double A3 = whatAngle (E3, B, E0);
        double SA = abs(A0) + abs(A1) + abs(A2) + abs(A3);

        //cerr << A0 << endl ;
        //cerr << A1 << endl ;
        //cerr << A2 << endl ;
        //cerr << A3 << endl ;
        //cerr << " = " << abs(A0) + abs(A1) + abs(A2) + abs(A3) << endl ;
        //viewer.spin ();

        if ( abs(SA - 360) <= sat )
        {
          B.z = backup_working_cloud->points.at (idx).z;

          if ( (minimus < B.z) && (B.z < maximus) )
            backup_novel_box_inliers->indices.push_back (idx);

          //cerr << SA << endl ;
        }

      }
      //*/
      // TODO This is NOT the optim way, just a way // End //

      //if ( ((max_dist / min_dist) > flat_value) || ((max_dist / z_dist) > flat_value ))
      //if ( smallest_dimension < flat_value )
      if ( (smallest_dimension / (medium_dimension + biggest_dimension)) < flat_value )
      //if ((max_dist / min_dist) > flat_value)
      {
        if ( number_of_flat < 10 )
          object_filename << directory << "---" << "object" << "-" << "flat" << "_" << "0" << number_of_flat << ".pcd" ;
        else
          object_filename << directory << "---" << "object" << "-" << "flat" << "_" << number_of_flat << ".pcd" ;

        cerr << "      FLAT      " << endl ;

          std::stringstream box_cloud_id;
          box_cloud_id << "BOX_CLOUD_" << getTimestamp ();
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> box_cloud_color (box_cloud, 0, 127, 255);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (box_cloud, box_cloud_color, box_cloud_id.str ());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, box_cloud_id.str ());
          if ( space_step ) viewer.spin ();

          viewer.removePointCloud ("generic");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "generic");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
          if ( space_step ) viewer.spin ();

        std::stringstream cub_id;
        cub_id << "CUB_" << getTimestamp ();
        viewer.addCuboid (cub, 0.0, 0.5, 1.0, 0.5, cub_id.str ());
        if ( space_step ) viewer.spin ();

        cubs_per_view.push_back (cub);

        clouds_of_cubs_per_view.push_back (box_cloud);

        number_of_flat++;
        /*
        pcl::PointCloud<pcl::PointXYZRGBI>::Ptr marked_box_cloud (new pcl::PointCloud<pcl::PointXYZRGBI> ());
        pcl::copyFields (*box_cloud, *marked_box_cloud);
        for (int idx=0; idx < (int) marked_box_cloud->points.size (); idx++)
          marked_box_cloud->points.at (idx).label = 3;
        *marked_working_cloud += *marked_box_cloud;
        */
        ///*
        for (int idx=0; idx < (int) backup_novel_box_inliers->indices.size (); idx++)
          marked_working_cloud->points.at (backup_novel_box_inliers->indices.at (idx)).intensity = 3;
        //*/
      }
      else
      {
        if ( number_of_box < 10 )
          object_filename << directory << "--" << "box" << "_" << "0" << number_of_box << ".pcd" ;
        else
          object_filename << directory << "--" << "box" << "_" << number_of_box << ".pcd" ;

        cerr << "      BOX      " << endl ;

          std::stringstream box_cloud_id;
          box_cloud_id << "BOX_CLOUD_" << getTimestamp ();
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> box_cloud_color (box_cloud, 127, 0, 255);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (box_cloud, box_cloud_color, box_cloud_id.str ());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, box_cloud_id.str ());
          if ( space_step ) viewer.spin ();

          viewer.removePointCloud ("generic");
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "generic");
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
          if ( space_step ) viewer.spin ();

        std::stringstream cub_id;
        // cub_id << "CUB_" << getTimestamp (); //
        cub_id << "cad_box" << "_" << number_of_box << ".vtk";
        viewer.addCuboidVTK (cub, 0.5, 0.0, 1.0, 0.5, cub_id.str ());



                double abs_cen[3];
                abs_cen[0] = x_cen;
                abs_cen[1] = y_cen;
                abs_cen[2] = z_cen;

                Eigen::Vector4f rel_cen;
                pcl::compute3DCentroid (*box_cloud, rel_cen);

                fprintf (cad_data, "  box %d | absolute center (%12.10f,%12.10f,%12.10f) | relative center (%12.10f,%12.10f,%12.10f) | length %12.10f width %12.10f height %12.10f \n",
                         number_of_box, abs_cen[0], abs_cen[1], abs_cen[2], rel_cen[0], rel_cen[1], rel_cen[2], x_dist, y_dist, z_dist);



        if ( space_step ) viewer.spin ();

        cubs_per_view.push_back (cub);

        clouds_of_cubs_per_view.push_back (box_cloud);

        number_of_box++;
        /*
        pcl::PointCloud<pcl::PointXYZRGBI>::Ptr marked_box_cloud (new pcl::PointCloud<pcl::PointXYZRGBI> ());
        pcl::copyFields (*box_cloud, *marked_box_cloud);
        for (int idx=0; idx < (int) marked_box_cloud->points.size (); idx++)
          marked_box_cloud->points.at (idx).label = 2;
        *marked_working_cloud += *marked_box_cloud;
        */
        ///*
        for (int idx=0; idx < (int) backup_novel_box_inliers->indices.size (); idx++)
          marked_working_cloud->points.at (backup_novel_box_inliers->indices.at (idx)).intensity = 2;
        //*/
      }

      cerr << object_filename.str () << endl;
      cerr << object_filename.str () << endl;
      cerr << object_filename.str () << endl;

      pcl::io::savePCDFile (object_filename.str (), *box_cloud, 10);

      if ( !till_the_end ) viewer.spin ();

    }

    }



    // ---------- Estimate The Cylinder Like Object ---------- //



    if ( more_votes_for_circles )
    {
      float scx = 0.0;
      float scy = 0.0;
      float  sr = 0.0;

      int votes = circle_parameters_cloud->size ();

      for (int vot = 0; vot < votes; vot++)
      {
        float cx = circle_parameters_cloud->points.at (vot).x;
        float cy = circle_parameters_cloud->points.at (vot).y;
        float  r = circle_parameters_cloud->points.at (vot).z;

        scx = scx + cx;
        scy = scy + cy;
        sr =  sr +  r;
      }

      float mcx = scx / votes;
      float mcy = scy / votes;
      float  mr =  sr / votes;

      pcl::PointIndices::Ptr cylinder_inliers (new pcl::PointIndices ());

      for (int idx = 0; idx < (int) working_cloud->points.size(); idx++)
      {
        double x = working_cloud->points.at (idx).x;
        double y = working_cloud->points.at (idx).y;

        double d = sqrt ( _sqr (mcx - x) + _sqr (mcy - y) );

        if ( d < (mr + circle_threshold) )
        {
          // Save only the right indices
          cylinder_inliers->indices.push_back (idx);
        }
      }

      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr cylinder_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> cyl_ei;
      cyl_ei.setInputCloud (working_cloud);
      cyl_ei.setIndices (cylinder_inliers);

      cyl_ei.setNegative (false);
      cyl_ei.filter (*cylinder_cloud);

      cyl_ei.setNegative (true);
      cyl_ei.filter (*working_cloud);

      //

      if ( working_cloud->points.size () > 0 )
      {
        pcl::StatisticalOutlierRemoval<pcl::PointXYZRGBNormalRSD> sor;
        sor.setInputCloud (working_cloud);
        sor.setMeanK (mean_k_filter);
        sor.setStddevMulThresh (std_dev_filter *5);
        sor.filter (*working_cloud);
      }

      //

        std::stringstream cyl_cloud_id;
        cyl_cloud_id << "CYL_CLOUD_" << getTimestamp ();
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> cylinder_cloud_color (cylinder_cloud, 127, 255, 0);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (cylinder_cloud, cylinder_cloud_color, cyl_cloud_id.str ());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cyl_cloud_id.str ());
        if ( space_step ) viewer.spin ();

        viewer.removePointCloud ("generic");
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> working_cloud_color (working_cloud, 0, 0, 0);
        viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (working_cloud, working_cloud_color, "generic");
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, "generic");
        if ( space_step ) viewer.spin ();

      // ADD CYLINDER SHAPE //

      pcl::PointXYZRGBNormalRSD cyl_min, cyl_max;
      pcl::getMinMax3D<pcl::PointXYZRGBNormalRSD> (*cylinder_cloud, cyl_min, cyl_max);

//////      cyl_min.z = working_cloud_minimum.z;

      if ( shapes_from_table_plane )
        cyl_min.z = abs_min.z;

      pcl::ModelCoefficients cyl;

      cyl.values.push_back (mcx);
      cyl.values.push_back (mcy);
      cyl.values.push_back (cyl_min.z);
      cyl.values.push_back (0.0);
      cyl.values.push_back (0.0);
      cyl.values.push_back (cyl_max.z - cyl_min.z);
      cyl.values.push_back (mr);

      cerr << mcx << " " << mcy << " " << cyl_min.z << endl ;
      cerr << cyl_max.z - cyl_min.z << endl ;
      cerr << mr << endl ;

      std::stringstream cyl_id;
      // cyl_id << "CYL" << getTimestamp (); //
      cyl_id << "cad_cylinder" << "_" << number_of_tall << ".vtk";
      viewer.addCylinderVTK (cyl, 0.5, 1.0, 0.0, 0.5, cyl_id.str ());



            double abs_cen[3];
            abs_cen[0] = mcx;
            abs_cen[1] = mcy;
            abs_cen[2] = cyl_min.z + ((cyl_max.z - cyl_min.z) / 2);

            Eigen::Vector4f rel_cen;
            pcl::compute3DCentroid (*cylinder_cloud, rel_cen);

            fprintf (cad_data, "  cylinder %d | absolute center (%12.10f,%12.10f,%12.10f) | relative center (%12.10f,%12.10f,%12.10f) | radius %12.10f height %12.10f \n",
                     number_of_tall, abs_cen[0], abs_cen[1], abs_cen[2], rel_cen[0], rel_cen[1], rel_cen[2], mr, (cyl_max.z - cyl_min.z));



      cyls_per_view.push_back (cyl);

      clouds_of_cyls_per_view.push_back (cylinder_cloud);

      model++;
      cerr << endl << " MODEL " << model << endl << endl ;

      if ( space_step ) viewer.spin ();

       //                //
      // Classification //
     //                //

      if ( classification )
      {

        // Create file name for saving
        std::stringstream object_filename;

        double height_of_cylinder;
        if ( consider_height_from_table_plane )
          height_of_cylinder = cyl_max.z;
        else
          height_of_cylinder = cyl_max.z - cyl_min.z;

        cerr << height_of_cylinder << endl ;

        // Select Tall Cylinders //

        if ( height_of_cylinder > tall_value )
        {
//          if ( number_of_tall < 10 )
//            object_filename << directory << "-" << "tall" << "_" << "0" << number_of_tall << ".pcd" ;
//          else
//            object_filename << directory << "-" << "tall" << "_" << number_of_tall << ".pcd" ;
//
//          cerr << "      TALL      " << endl ;

          if ( number_of_tall < 10 )
            object_filename << directory << "--" << "cylinder" << "_" << "0" << number_of_tall << ".pcd" ;
          else
            object_filename << directory << "--" << "cylinder" << "_" << number_of_tall << ".pcd" ;

          cerr << "      CYLINDER      " << endl ;

          //

          number_of_tall++;
        }
        else
        {

          // Select Short Cylinders //

          if ( height_of_cylinder < short_value )
          {
            if ( number_of_short < 10 )
              object_filename << directory << "-" << "short" << "_" << "0" << number_of_short << ".pcd" ;
            else
              object_filename << directory << "-" << "short" << "_" << number_of_short << ".pcd" ;

            cerr << "      SHORT      " << endl ;

            number_of_short++;
          }
          else
          {

            // Select Medium Cylinders //

            //if ( height_of_cylinder > medium_value )
            //{
            if ( number_of_medium < 10 )
              object_filename << directory << "-" << "medium" << "_" << "0" << number_of_medium << ".pcd" ;
            else
              object_filename << directory << "-" << "medium" << "_" << number_of_medium << ".pcd" ;

            cerr << "      MEDIUM      " << endl ;

            number_of_medium++;
            //}
          }
        }

        // TODO This is NOT the optim way, just a way // Begin //
        ///*
        pcl::PointIndices::Ptr backup_cylinder_inliers (new pcl::PointIndices ());

        for (int idx = 0; idx < (int) backup_working_cloud->points.size(); idx++)
        {
          double x = backup_working_cloud->points.at (idx).x;
          double y = backup_working_cloud->points.at (idx).y;

          double d = sqrt ( _sqr (mcx - x) + _sqr (mcy - y) );

          if ( d < (mr + circle_threshold) )
          {
            // Save only the right indices
            backup_cylinder_inliers->indices.push_back (idx);
          }
        }

        // TODO This is NOT the optim way, just a way // End //

        for (int idx=0; idx < (int) backup_cylinder_inliers->indices.size (); idx++)
          marked_working_cloud->points.at (backup_cylinder_inliers->indices.at (idx)).intensity = 4;
        //*/
        cerr << object_filename.str () << endl;
        cerr << object_filename.str () << endl;
        cerr << object_filename.str () << endl;

        pcl::io::savePCDFile (object_filename.str (), *cylinder_cloud, 10);
        /*
        pcl::PointCloud<pcl::PointXYZRGBI>::Ptr marked_cylinder_cloud (new pcl::PointCloud<pcl::PointXYZRGBI> ());
        pcl::copyFields (*cylinder_cloud, *marked_cylinder_cloud);
        for (int idx=0; idx < (int) marked_cylinder_cloud->points.size (); idx++)
          marked_cylinder_cloud->points.at (idx).label = 4;
        *marked_working_cloud += *marked_cylinder_cloud;
        */
        if ( !till_the_end ) viewer.spin ();

      }

    }

    std::stringstream marked_output_filename;
    marked_output_filename << directory << "--" << "marked.pcd" ;
    pcl::io::savePCDFileASCII (marked_output_filename.str (), *marked_working_cloud);

    // ---------- Deal With The Rest Of The Points ---------- //

    if ( deal_with_the_rest_of_the_points )
    {

    if ( !more_votes_for_lines && !more_votes_for_circles )
    {
      viewer.spin ();

      std::vector<pcl::PointIndices> r_clusterS;
      pcl::search::KdTree<pcl::PointXYZRGBNormalRSD>::Ptr r_tree (new pcl::search::KdTree<pcl::PointXYZRGBNormalRSD> ());
      r_tree->setInputCloud (working_cloud);

      pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormalRSD> r_ece;
      r_ece.setInputCloud (working_cloud);
      r_ece.setClusterTolerance (r_clustering_tolerance);
      r_ece.setMinClusterSize (minimum_size_of_r_clusters);
      r_ece.setSearchMethod (r_tree);
      r_ece.extract (r_clusterS);

      if ( verbose ) pcl::console::print_info ("Euclidean Cluster Extraction ! Returned: %d rest clusters\n", (int) r_clusterS.size ());

      std::vector<pcl::PointIndices::Ptr> r_clusterS_indices;
      std::vector<pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr> r_clusterS_clouds;

      for (int clu = 0; clu < (int) r_clusterS.size(); clu++)
      {
        pcl::PointIndices::Ptr r_cluster_indices (new pcl::PointIndices (r_clusterS.at (clu)));
        pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr r_cluster_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

        pcl::ExtractIndices<pcl::PointXYZRGBNormalRSD> r_ei;
        r_ei.setInputCloud (working_cloud);
        r_ei.setIndices (r_cluster_indices);
        r_ei.setNegative (false);
        r_ei.filter (*r_cluster_cloud);

        if ( verbose ) pcl::console::print_info ("  Object cluster %2d has %4d points\n", clu, (int) r_cluster_cloud->points.size());

        // Placeholder for the 3x3 covariance matrix at each surface patch
        EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
        // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
        Eigen::Vector4f xyz_centroid;

        // Estimate the XYZ centroid
        pcl::compute3DCentroid (*r_cluster_cloud, xyz_centroid);

        // Compute the 3x3 covariance matrix
        pcl::computeCovarianceMatrix (*r_cluster_cloud, xyz_centroid, covariance_matrix);

        // Extract the eigenvalues and eigenvectors
        EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
        EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
        pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);

        cerr << " eigen_values [0] = " << eigen_values [0] << endl;
        cerr << " eigen_values [1] = " << eigen_values [1] << endl;
        cerr << " eigen_values [2] = " << eigen_values [2] << endl;

        // Length- and Cross- Vectors //
        double vectors[3][3];
        vectors[0][0] = eigen_vectors (0, 0);
        vectors[0][1] = eigen_vectors (0, 1);
        vectors[0][2] = eigen_vectors (0, 2);

        vectors[1][0] = eigen_vectors (1, 0);
        vectors[1][1] = eigen_vectors (1, 1);
        vectors[1][2] = eigen_vectors (1, 2);

        vectors[2][0] = eigen_vectors (2, 0);
        vectors[2][1] = eigen_vectors (2, 1);
        vectors[2][2] = eigen_vectors (2, 2);

        Eigen::Vector4f centroid;
        pcl::compute3DCentroid (*r_cluster_cloud, centroid);

        double max_u = -DBL_MAX;
        double min_u =  DBL_MAX;
        double max_v = -DBL_MAX;
        double min_v =  DBL_MAX;
        double max_w = -DBL_MAX;
        double min_w =  DBL_MAX;

        for ( int idx = 0; idx < (int) r_cluster_cloud->points.size(); idx++ )
        {
          double c2p[3];
          c2p[0] = r_cluster_cloud->points.at (idx).x - centroid[0];
          c2p[1] = r_cluster_cloud->points.at (idx).y - centroid[1];
          c2p[2] = r_cluster_cloud->points.at (idx).z - centroid[2];

          double  width = vectors[0][0]*c2p[0] + vectors[0][1]*c2p[1] + vectors[0][2]*c2p[2];
          if ( width > max_u) max_u =  width;
          if ( width < min_u) min_u =  width;

          double length = vectors[1][0]*c2p[0] + vectors[1][1]*c2p[1] + vectors[1][2]*c2p[2];
          if (length > max_v) max_v = length;
          if (length < min_v) min_v = length;

          double height = vectors[2][0]*c2p[0] + vectors[2][1]*c2p[1] + vectors[2][2]*c2p[2];
          if (height > max_w) max_w = height;
          if (height < min_w) min_w = height;
        }

        double E[8][3];

        E[0][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*max_v + vectors[2][0]*min_w;
        E[0][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*max_v + vectors[2][1]*min_w;
        E[0][2] = centroid[2] + vectors[0][2]*max_u + vectors[1][2]*max_v + vectors[2][2]*min_w;

        E[1][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*min_v + vectors[2][0]*min_w;
        E[1][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*min_v + vectors[2][1]*min_w;
        E[1][2] = centroid[2] + vectors[0][2]*max_u + vectors[1][2]*min_v + vectors[2][2]*min_w;

        E[2][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*min_v + vectors[2][0]*min_w;
        E[2][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*min_v + vectors[2][1]*min_w;
        E[2][2] = centroid[2] + vectors[0][2]*min_u + vectors[1][2]*min_v + vectors[2][2]*min_w;

        E[3][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*max_v + vectors[2][0]*min_w;
        E[3][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*max_v + vectors[2][1]*min_w;
        E[3][2] = centroid[2] + vectors[0][2]*min_u + vectors[1][2]*max_v + vectors[2][2]*min_w;

        //

        E[4][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*max_v + vectors[2][0]*max_w;
        E[4][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*max_v + vectors[2][1]*max_w;
        E[4][2] = centroid[2] + vectors[0][2]*max_u + vectors[1][2]*max_v + vectors[2][2]*max_w;

        E[5][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*min_v + vectors[2][0]*max_w;
        E[5][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*min_v + vectors[2][1]*max_w;
        E[5][2] = centroid[2] + vectors[0][2]*max_u + vectors[1][2]*min_v + vectors[2][2]*max_w;

        E[6][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*min_v + vectors[2][0]*max_w;
        E[6][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*min_v + vectors[2][1]*max_w;
        E[6][2] = centroid[2] + vectors[0][2]*min_u + vectors[1][2]*min_v + vectors[2][2]*max_w;

        E[7][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*max_v + vectors[2][0]*max_w;
        E[7][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*max_v + vectors[2][1]*max_w;
        E[7][2] = centroid[2] + vectors[0][2]*min_u + vectors[1][2]*max_v + vectors[2][2]*max_w;

        pcl::ModelCoefficients C0, C1, C2, C3, C4, C5, C6, C7;

        C0.values.push_back (E[0][0]);
        C0.values.push_back (E[0][1]);
        C0.values.push_back (E[0][2]);

        C1.values.push_back (E[1][0]);
        C1.values.push_back (E[1][1]);
        C1.values.push_back (E[1][2]);

        C2.values.push_back (E[2][0]);
        C2.values.push_back (E[2][1]);
        C2.values.push_back (E[2][2]);

        C3.values.push_back (E[3][0]);
        C3.values.push_back (E[3][1]);
        C3.values.push_back (E[3][2]);

        //

        C4.values.push_back (E[4][0]);
        C4.values.push_back (E[4][1]);
        C4.values.push_back (E[4][2]);

        C5.values.push_back (E[5][0]);
        C5.values.push_back (E[5][1]);
        C5.values.push_back (E[5][2]);

        C6.values.push_back (E[6][0]);
        C6.values.push_back (E[6][1]);
        C6.values.push_back (E[6][2]);

        C7.values.push_back (E[7][0]);
        C7.values.push_back (E[7][1]);
        C7.values.push_back (E[7][2]);

        std::vector<pcl::ModelCoefficients> C;

        C.push_back (C0);
        C.push_back (C1);
        C.push_back (C2);
        C.push_back (C3);

        //

        C.push_back (C4);
        C.push_back (C5);
        C.push_back (C6);
        C.push_back (C7);

        double minimus = +DBL_MAX;
        double maximus = -DBL_MAX;

        /*

        // Bounding box only for line inliers //

        for ( int inl = 0; inl < (int) shapes_lines_inliers.at (clu)->indices.size(); inl++ )
        {
        int idx = shapes_lines_inliers.at (clu)->indices.at (inl);

        double Z = shapes_lines_clusters.at (clu)->points.at (idx).z;

        if ( minimus > Z ) minimus = Z;
        if ( maximus < Z ) maximus = Z;
        }

        */

        // Bounding box for whole cluster //

        for ( int idx = 0; idx < (int) r_cluster_cloud->points.size(); idx++ )
        {
          double Z = r_cluster_cloud->points.at (idx).z;

          if ( minimus > Z ) minimus = Z;
          if ( maximus < Z ) maximus = Z;
        }

        // The edges //

        double edges[4][2];

        edges[0][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*max_v;
        edges[0][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*max_v;

        edges[1][0] = centroid[0] + vectors[0][0]*max_u + vectors[1][0]*min_v;
        edges[1][1] = centroid[1] + vectors[0][1]*max_u + vectors[1][1]*min_v;

        edges[2][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*min_v;
        edges[2][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*min_v;

        edges[3][0] = centroid[0] + vectors[0][0]*min_u + vectors[1][0]*max_v;
        edges[3][1] = centroid[1] + vectors[0][1]*min_u + vectors[1][1]*max_v;

        minimus = working_cloud_minimum.z;

        pcl::ModelCoefficients e0, e1, e2, e3;

        e0.values.push_back (edges[0][0]);
        e0.values.push_back (edges[0][1]);
        e0.values.push_back (minimus);
        e0.values.push_back (edges[1][0] - edges[0][0]);
        e0.values.push_back (edges[1][1] - edges[0][1]);
        e0.values.push_back (minimus - minimus);

        e1.values.push_back (edges[1][0]);
        e1.values.push_back (edges[1][1]);
        e1.values.push_back (minimus);
        e1.values.push_back (edges[2][0] - edges[1][0]);
        e1.values.push_back (edges[2][1] - edges[1][1]);
        e1.values.push_back (minimus - minimus);

        e2.values.push_back (edges[2][0]);
        e2.values.push_back (edges[2][1]);
        e2.values.push_back (minimus);
        e2.values.push_back (edges[3][0] - edges[2][0]);
        e2.values.push_back (edges[3][1] - edges[2][1]);
        e2.values.push_back (minimus - minimus);

        e3.values.push_back (edges[3][0]);
        e3.values.push_back (edges[3][1]);
        e3.values.push_back (minimus);
        e3.values.push_back (edges[0][0] - edges[3][0]);
        e3.values.push_back (edges[0][1] - edges[3][1]);
        e3.values.push_back (minimus - minimus);

        /*

        std::stringstream line_0;
        line_0 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e0);
        line_viewer.addLine (e0, line_0.str ());
        line_viewer.spin ();

        std::stringstream line_1;
        line_1 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e1);
        line_viewer.addLine (e1, line_1.str ());
        line_viewer.spin ();

        std::stringstream line_2;
        line_2 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e2);
        line_viewer.addLine (e2, line_2.str ());
        line_viewer.spin ();

        std::stringstream line_3;
        line_3 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e3);
        line_viewer.addLine (e3, line_3.str ());
        line_viewer.spin ();

        */

        pcl::ModelCoefficients e4, e5, e6, e7;

        e4.values.push_back (edges[0][0]);
        e4.values.push_back (edges[0][1]);
        e4.values.push_back (maximus);
        e4.values.push_back (edges[1][0] - edges[0][0]);
        e4.values.push_back (edges[1][1] - edges[0][1]);
        e4.values.push_back (maximus - maximus);

        e5.values.push_back (edges[1][0]);
        e5.values.push_back (edges[1][1]);
        e5.values.push_back (maximus);
        e5.values.push_back (edges[2][0] - edges[1][0]);
        e5.values.push_back (edges[2][1] - edges[1][1]);
        e5.values.push_back (maximus - maximus);

        e6.values.push_back (edges[2][0]);
        e6.values.push_back (edges[2][1]);
        e6.values.push_back (maximus);
        e6.values.push_back (edges[3][0] - edges[2][0]);
        e6.values.push_back (edges[3][1] - edges[2][1]);
        e6.values.push_back (maximus - maximus);

        e7.values.push_back (edges[3][0]);
        e7.values.push_back (edges[3][1]);
        e7.values.push_back (maximus);
        e7.values.push_back (edges[0][0] - edges[3][0]);
        e7.values.push_back (edges[0][1] - edges[3][1]);
        e7.values.push_back (maximus - maximus);

        /*

        std::stringstream line_4;
        line_4 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e4);
        line_viewer.addLine (e4, line_4.str ());
        line_viewer.spin ();

        std::stringstream line_5;
        line_5 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e5);
        line_viewer.addLine (e5, line_5.str ());
        line_viewer.spin ();

        std::stringstream line_6;
        line_6 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e6);
        line_viewer.addLine (e6, line_6.str ());
        line_viewer.spin ();

        std::stringstream line_7;
        line_7 << "LINE_" << ros::Time::now();
        //adjustLine (shapes_lines_clusters.at (clu), e7);
        line_viewer.addLine (e7, line_7.str ());
        line_viewer.spin ();

*/

        std::vector<pcl::ModelCoefficients> cub;

        cub.push_back (e0);
        cub.push_back (e1);
        cub.push_back (e2);
        cub.push_back (e3);
        cub.push_back (e4);
        cub.push_back (e5);
        cub.push_back (e6);
        cub.push_back (e7);

        std::stringstream cub_id;
        cub_id << "CUB_" << getTimestamp ();
        viewer.addCuboid (C, 0.5, 0.0, 1.0, 0.5, cub_id.str ());
        viewer.spin ();

        //////cubs_per_view.push_back (C);
        //////
        //////clouds_of_cubs_per_view.push_back (r_cluster_cloud);

        // Remove These Point From The Cloud //
        r_ei.setNegative (true);
        r_ei.filter (*working_cloud);

        r_clusterS_indices.push_back (r_cluster_indices);
        r_clusterS_clouds.push_back (r_cluster_cloud);
      }

      viewer.spin ();
    }

    }

    if (!more_votes_for_lines && !more_votes_for_circles) continue_hough = false;

    fit++;

    // Print And Check If Either To Continue Or Stop //
    if ( (int) working_cloud->points.size () < minimum_line_inliers )
      pcl::console::print_warn ("    %d < %d | Stop !\n", (int) working_cloud->points.size (), minimum_line_inliers);
    else
      if ( (int) working_cloud->points.size () > minimum_line_inliers )
        pcl::console::print_warn ("    %d > %d | Continue... \n", (int) working_cloud->points.size (), minimum_line_inliers);
      else
        pcl::console::print_warn ("    %d = %d | Continue... \n", (int) working_cloud->points.size (), minimum_line_inliers);

  } while ( ((int) working_cloud->points.size () > minimum_line_inliers) && ((int) working_cloud->points.size () > minimum_circle_inliers) && (continue_hough) );

  viewer.spin ();

//  std::cerr << endl ;
//  std::cout << " RE-RUN = ";
//  std::cin >> rerun;
//  std::cerr << endl ;
//
//  pcl::console::print_value ("\n re-Processing %s\n\n", argv [pcd_file_indices [fff]]);
//
//  if (rerun)
//    std::cerr << " YES " << endl << endl ;
//  else
//    std::cerr << " NO " << endl << endl ;

  // ! Clean Up Time ! //
  viewer.removeAllShapes ();
  viewer.removeAllPointClouds ();

  //viewer.spin ();

  } while (rerun);

//  cerr << " cubs per view = " << cubs_per_view.size () << endl ;
//  cerr << " cyls per view = " << cyls_per_view.size () << endl ;

  cubs.push_back (cubs_per_view);
  cyls.push_back (cyls_per_view);

//  cerr << " clouds of cubs per view = " << clouds_of_cubs_per_view.size () << endl ;
//  cerr << " clouds of cyls per view = " << clouds_of_cyls_per_view.size () << endl ;

  clouds_of_cubs.push_back (clouds_of_cubs_per_view);
  clouds_of_cyls.push_back (clouds_of_cyls_per_view);

  }

  }



  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //

  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //

  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //



        fprintf (cad_data, "\n");
        fclose (cad_data);



  /*

  FILE * file;

  file = fopen ("characteristics-of-hough-voted-ransac-models.txt", "a");

*/

  /*

  fprintf (file, "\n----------------------------------------------------------------------------------------------------\n");

  fprintf (file, "\n  run at %s with \n", getTimestamp ().c_str ());

  for (int fff = 0; fff < pcd_file_indices.size (); fff++)
  {

    fprintf (file, "\n    for view %d \n", fff);

    for (int cu = 0; cu < cubs.at (fff).size (); cu++)
    {

      double d1 = sqrt (_sqr (cubs.at (fff).at (cu).at (0).values.at (0) - cubs.at (fff).at (cu).at (1).values.at (0)) + _sqr (cubs.at (fff).at (cu).at (0).values.at (1) - cubs.at (fff).at (cu).at (1).values.at (1)));
      double d2 = sqrt (_sqr (cubs.at (fff).at (cu).at (1).values.at (0) - cubs.at (fff).at (cu).at (2).values.at (0)) + _sqr (cubs.at (fff).at (cu).at (1).values.at (1) - cubs.at (fff).at (cu).at (2).values.at (1)));
      double d3 = cubs.at (fff).at (cu).at (4).values.at (2) - cubs.at (fff).at (cu).at (0).values.at (2);

      double v = d1 * d2 * d3;

      Eigen::Vector4f cen;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cubs.at (fff).at (cu), cen);

      fprintf (file, "\n      %12.10f x %12.10f x %12.10f = %12.10f \n", d1, d2, d3, v);

    }

    for (int cy = 0; cy < cyls.at (fff).size (); cy++)
    {

      double h = cyls.at (fff).at (cy).values.at (5);
      double r = cyls.at (fff).at (cy).values.at (6);

      double v = M_PI * _sqr(r) * h;

      Eigen::Vector4f cen;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cyls.at (fff).at (cy), cen);

      fprintf (file, "\n      pi x %12.10f^2 x %12.10f = %12.10f \n", r, h, v);

    }

  }

  */



  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //

  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //

  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //



  /*

  fprintf (file, "----------------------------------------------------------------------------------------------------\n\n");

  fprintf (file, "  run %s \n\n", getTimestamp ().c_str ());

  {

    int v1 = 0;
    int v2 = 1;

    bool match_not_found;

    cerr << endl << endl << endl << " threshold between centroids of cuboids = " << threshold_between_centroids_of_cuboids << endl ;

    for (int cu1 = 0; cu1 < clouds_of_cubs.at (v1).size (); cu1++)
    {

      match_not_found = true;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cubs.at (v1).at (cu1), cen1);

      for (int cu2 = 0; cu2 < clouds_of_cubs.at (v2).size (); cu2++)
      {

        Eigen::Vector4f cen2;
        pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cubs.at (v2).at (cu2), cen2);

        double cen1_to_cen2 = sqrt( _sqr(cen1[0]-cen2[0]) + _sqr(cen1[1]-cen2[1]) + _sqr(cen1[2]-cen2[2]) );

        printf (" %7.5f", cen1_to_cen2);

        if ( cen1_to_cen2 < threshold_between_centroids_of_cuboids )
        {

          match_not_found = false;

          cerr << " match found" << endl ;

          if ( true )
          {
            std::stringstream cu1_id; cu1_id << "cu1_id_" << getTimestamp ();
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> cu1_color (clouds_of_cubs.at (v1).at (cu1), 85, 170, 255);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cubs.at (v1).at (cu1), cu1_color, cu1_id.str ());
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cu1_id.str ());

            std::stringstream cu2_id; cu2_id << "cu2_id_" << getTimestamp ();
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> cu2_color (clouds_of_cubs.at (v2).at (cu2), 255, 170, 85);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cubs.at (v2).at (cu2), cu2_color, cu2_id.str ());
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cu2_id.str ());

            //viewer.spin ();
            //viewer.removePointCloud (cu1_id. str());
            //viewer.removePointCloud (cu2_id. str());
          }

          fprintf (file, "    box %d \n\n", cu1);

          double cu1_d1 = sqrt (_sqr (cubs.at (v1).at (cu1).at (0).values.at (0) - cubs.at (v1).at (cu1).at (1).values.at (0)) + _sqr (cubs.at (v1).at (cu1).at (0).values.at (1) - cubs.at (v1).at (cu1).at (1).values.at (1)));
          double cu1_d2 = sqrt (_sqr (cubs.at (v1).at (cu1).at (1).values.at (0) - cubs.at (v1).at (cu1).at (2).values.at (0)) + _sqr (cubs.at (v1).at (cu1).at (1).values.at (1) - cubs.at (v1).at (cu1).at (2).values.at (1)));
          double cu1_d3 =             cubs.at (v1).at (cu1).at (4).values.at (2) - cubs.at (v1).at (cu1).at (0).values.at (2);
          double cu1_v  = cu1_d1 * cu1_d2 * cu1_d3;

          fprintf (file, "      view 1 %12.10f x %12.10f x %12.10f = %12.10f \n", cu1_d1, cu1_d2, cu1_d3, cu1_v);

          double cu2_d1 = sqrt (_sqr (cubs.at (v2).at (cu2).at (0).values.at (0) - cubs.at (v2).at (cu2).at (1).values.at (0)) + _sqr (cubs.at (v2).at (cu2).at (0).values.at (1) - cubs.at (v2).at (cu2).at (1).values.at (1)));
          double cu2_d2 = sqrt (_sqr (cubs.at (v2).at (cu2).at (1).values.at (0) - cubs.at (v2).at (cu2).at (2).values.at (0)) + _sqr (cubs.at (v2).at (cu2).at (1).values.at (1) - cubs.at (v2).at (cu2).at (2).values.at (1)));
          double cu2_d3 =             cubs.at (v2).at (cu2).at (4).values.at (2) - cubs.at (v2).at (cu2).at (0).values.at (2);
          double cu2_v  = cu2_d1 * cu2_d2 * cu2_d3;

          fprintf (file, "      view 2 %12.10f x %12.10f x %12.10f = %12.10f \n\n", cu2_d1, cu2_d2, cu2_d3, cu2_v);

        }
        else
          cerr << endl ;

        viewer.spin ();

      }

      if ( match_not_found )
      {

        cerr << "         match not found ! but model added. " << endl ;

        if ( true )
        {
          std::stringstream cu1_id; cu1_id << "cu1_id_" << getTimestamp ();
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> cu1_color (clouds_of_cubs.at (v1).at (cu1), 85, 170, 255);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cubs.at (v1).at (cu1), cu1_color, cu1_id.str ());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cu1_id.str ());

          //viewer.spin ();
          //viewer.removePointCloud (cu1_id. str());
        }

        fprintf (file, "    box %d \n\n", cu1);

        double cu1_d1 = sqrt (_sqr (cubs.at (v1).at (cu1).at (0).values.at (0) - cubs.at (v1).at (cu1).at (1).values.at (0)) + _sqr (cubs.at (v1).at (cu1).at (0).values.at (1) - cubs.at (v1).at (cu1).at (1).values.at (1)));
        double cu1_d2 = sqrt (_sqr (cubs.at (v1).at (cu1).at (1).values.at (0) - cubs.at (v1).at (cu1).at (2).values.at (0)) + _sqr (cubs.at (v1).at (cu1).at (1).values.at (1) - cubs.at (v1).at (cu1).at (2).values.at (1)));
        double cu1_d3 =             cubs.at (v1).at (cu1).at (4).values.at (2) - cubs.at (v1).at (cu1).at (0).values.at (2);
        double cu1_v  = cu1_d1 * cu1_d2 * cu1_d3;

        fprintf (file, "      view 1 %12.10f x %12.10f x %12.10f = %12.10f \n\n", cu1_d1, cu1_d2, cu1_d3, cu1_v);

        viewer.spin ();

      }

    }

  }

  {

    int v1 = 0;
    int v2 = 1;

    bool match_not_found;

    cerr << endl << endl << endl << " threshold between centroids of cylinders = " << threshold_between_centroids_of_cylinders << endl ;

    for (int cy1 = 0; cy1 < clouds_of_cyls.at (v1).size (); cy1++)
    {

      match_not_found = true;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cyls.at (v1).at (cy1), cen1);

      for (int cy2 = 0; cy2 < clouds_of_cyls.at (v2).size (); cy2++)
      {

        Eigen::Vector4f cen2;
        pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cyls.at (v2).at (cy2), cen2);

        double cen1_to_cen2 = sqrt( _sqr(cen1[0]-cen2[0]) + _sqr(cen1[1]-cen2[1]) + _sqr(cen1[2]-cen2[2]) );

        printf (" %7.5f", cen1_to_cen2);

        if ( cen1_to_cen2 < threshold_between_centroids_of_cylinders )
        {

          match_not_found = false;

          cerr << " match found" << endl ;

          if ( true )
          {
            std::stringstream cy1_id; cy1_id << "cy1_id_" << getTimestamp ();
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> cy1_color (clouds_of_cyls.at (v1).at (cy1), 85, 170, 255);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cyls.at (v1).at (cy1), cy1_color, cy1_id. str());
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cy1_id. str());

            std::stringstream cy2_id; cy2_id << "cy2_id_" << getTimestamp ();
            pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> cy2_color (clouds_of_cyls.at (v2).at (cy2), 255, 170, 85);
            viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cyls.at (v2).at (cy2), cy2_color, cy2_id. str());
            viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cy2_id. str());

            //viewer.spin ();
            //viewer.removePointCloud (cy1_id. str());
            //viewer.removePointCloud (cy2_id. str());
          }

          fprintf (file, "    cylinder %d \n\n", cy1);

          double cy1_h = cyls.at (v1).at (cy1).values.at (5);
          double cy1_r = cyls.at (v1).at (cy1).values.at (6);
          double cy1_v = M_PI * _sqr(cy1_r) * cy1_h;

          fprintf (file, "      view 1 pi x %12.10f ^2 x %12.10f = %12.10f \n", cy1_r, cy1_h, cy1_v);

          double cy2_h = cyls.at (v2).at (cy2).values.at (5);
          double cy2_r = cyls.at (v2).at (cy2).values.at (6);
          double cy2_v = M_PI * _sqr(cy2_r) * cy2_h;

          fprintf (file, "      view 2 pi x %12.10f ^2 x %12.10f = %12.10f \n\n", cy2_r, cy2_h, cy2_v);

        }
        else
          cerr << endl ;

        viewer.spin ();

      }

      if ( match_not_found )
      {

        cerr << "         match not found ! but model added. " << endl ;

        if ( true )
        {
          std::stringstream cy1_id; cy1_id << "cy1_id_" << getTimestamp ();
          pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> cy1_color (clouds_of_cyls.at (v1).at (cy1), 85, 170, 255);
          viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cyls.at (v1).at (cy1), cy1_color, cy1_id. str());
          viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, cy1_id. str());

          //viewer.spin ();
          //viewer.removePointCloud (cy1_id. str());
        }

        fprintf (file, "    cylinder %d \n\n", cy1);

        double cy1_h = cyls.at (v1).at (cy1).values.at (5);
        double cy1_r = cyls.at (v1).at (cy1).values.at (6);
        double cy1_v = M_PI * _sqr(cy1_r) * cy1_h;

        fprintf (file, "      view 1 pi x %12.10f ^2 x %12.10f = %12.10f \n\n", cy1_r, cy1_h, cy1_v);

      }

    }

  }

  */



  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //

  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //

  // -- // -- // -- // -- // -- // -- // -- // -- // -- // -- //



  /*

  {
    FILE * file1;

    file1 = fopen ("view-1-hough-voted-ransac-models.txt", "a");

    int v1 = 0;

    for (int cu1 = 0; cu1 < clouds_of_cubs.at (v1).size (); cu1++)
    {
      double cu1_d1 = sqrt (_sqr (cubs.at (v1).at (cu1).at (0).values.at (0) - cubs.at (v1).at (cu1).at (1).values.at (0)) + _sqr (cubs.at (v1).at (cu1).at (0).values.at (1) - cubs.at (v1).at (cu1).at (1).values.at (1)));
      double cu1_d2 = sqrt (_sqr (cubs.at (v1).at (cu1).at (1).values.at (0) - cubs.at (v1).at (cu1).at (2).values.at (0)) + _sqr (cubs.at (v1).at (cu1).at (1).values.at (1) - cubs.at (v1).at (cu1).at (2).values.at (1)));
      double cu1_d3 =             cubs.at (v1).at (cu1).at (4).values.at (2) - cubs.at (v1).at (cu1).at (0).values.at (2);
      double cu1_v  = cu1_d1 * cu1_d2 * cu1_d3;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cubs.at (v1).at (cu1), cen1);

      fprintf (file1, " 1 | %12.10f %12.10f %12.10f %12.10f | %12.10f %12.10f %12.10f \n", cu1_d1, cu1_d2, cu1_d3, cu1_v, cen1[0], cen1[1], cen1[2]);
    }

    for (int cy1 = 0; cy1 < clouds_of_cyls.at (v1).size (); cy1++)
    {
      double cy1_h = cyls.at (v1).at (cy1).values.at (5);
      double cy1_r = cyls.at (v1).at (cy1).values.at (6);
      double cy1_v = M_PI * _sqr(cy1_r) * cy1_h;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cyls.at (v1).at (cy1), cen1);

      fprintf (file1, " 2 | %12.10f %12.10f %12.10f | %12.10f %12.10f %12.10f \n", cy1_r, cy1_h, cy1_v, cen1[0], cen1[1], cen1[2]);
    }

    fprintf (file1, " 0 \n");

    fclose (file1);
  }



  {
    FILE * file2;

    file2 = fopen ("view-2-hough-voted-ransac-models.txt", "a");

    int v2 = 1;

    for (int cu2 = 0; cu2 < clouds_of_cubs.at (v2).size (); cu2++)
    {
      double cu2_d1 = sqrt (_sqr (cubs.at (v2).at (cu2).at (0).values.at (0) - cubs.at (v2).at (cu2).at (1).values.at (0)) + _sqr (cubs.at (v2).at (cu2).at (0).values.at (1) - cubs.at (v2).at (cu2).at (1).values.at (1)));
      double cu2_d2 = sqrt (_sqr (cubs.at (v2).at (cu2).at (1).values.at (0) - cubs.at (v2).at (cu2).at (2).values.at (0)) + _sqr (cubs.at (v2).at (cu2).at (1).values.at (1) - cubs.at (v2).at (cu2).at (2).values.at (1)));
      double cu2_d3 =             cubs.at (v2).at (cu2).at (4).values.at (2) - cubs.at (v2).at (cu2).at (0).values.at (2);
      double cu2_v  = cu2_d1 * cu2_d2 * cu2_d3;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cubs.at (v2).at (cu2), cen1);

      fprintf (file2, " 1 | %12.10f %12.10f %12.10f %12.10f | %12.10f %12.10f %12.10f \n", cu2_d1, cu2_d2, cu2_d3, cu2_v, cen1[0], cen1[1], cen1[2]);
    }

    for (int cy2 = 0; cy2 < clouds_of_cyls.at (v2).size (); cy2++)
    {
      double cy2_h = cyls.at (v2).at (cy2).values.at (5);
      double cy2_r = cyls.at (v2).at (cy2).values.at (6);
      double cy2_v = M_PI * _sqr(cy2_r) * cy2_h;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cyls.at (v2).at (cy2), cen1);

      fprintf (file2, " 2 | %12.10f %12.10f %12.10f | %12.10f %12.10f %12.10f \n", cy2_r, cy2_h, cy2_v, cen1[0], cen1[1], cen1[2]);
    }

    fprintf (file2, " 0 \n");

    fclose (file2);
  }

  */

  /*

  {
    FILE * merged_file;

    merged_file = fopen ("merged-hough-voted-ransac-models.txt", "a");

    int v1v2 = 0;

    for (int cu1 = 0; cu1 < clouds_of_cubs.at (v1v2).size (); cu1++)
    {
      double cu1_d1 = sqrt (_sqr (cubs.at (v1v2).at (cu1).at (0).values.at (0) - cubs.at (v1v2).at (cu1).at (1).values.at (0)) + _sqr (cubs.at (v1v2).at (cu1).at (0).values.at (1) - cubs.at (v1v2).at (cu1).at (1).values.at (1)));
      double cu1_d2 = sqrt (_sqr (cubs.at (v1v2).at (cu1).at (1).values.at (0) - cubs.at (v1v2).at (cu1).at (2).values.at (0)) + _sqr (cubs.at (v1v2).at (cu1).at (1).values.at (1) - cubs.at (v1v2).at (cu1).at (2).values.at (1)));
      double cu1_d3 =             cubs.at (v1v2).at (cu1).at (4).values.at (2) - cubs.at (v1v2).at (cu1).at (0).values.at (2);
      double cu1_v  = cu1_d1 * cu1_d2 * cu1_d3;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cubs.at (v1v2).at (cu1), cen1);

      fprintf (merged_file, " 1 | %12.10f %12.10f %12.10f %12.10f | %12.10f %12.10f %12.10f \n", cu1_d1, cu1_d2, cu1_d3, cu1_v, cen1[0], cen1[1], cen1[2]);
    }

    for (int cy1 = 0; cy1 < clouds_of_cyls.at (v1v2).size (); cy1++)
    {
      double cy1_h = cyls.at (v1v2).at (cy1).values.at (5);
      double cy1_r = cyls.at (v1v2).at (cy1).values.at (6);
      double cy1_v = M_PI * _sqr(cy1_r) * cy1_h;

      Eigen::Vector4f cen1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cyls.at (v1v2).at (cy1), cen1);

      fprintf (merged_file, " 2 | %12.10f %12.10f %12.10f | %12.10f %12.10f %12.10f \n", cy1_r, cy1_h, cy1_v, cen1[0], cen1[1], cen1[2]);
    }

    fprintf (merged_file, " 0 \n");

    fclose (merged_file);
  }

  */

  /*

  fclose (file);

  */





/*             *
 * Big Section *
 *             *

  for (int view = 0; view < pcd_file_indices.size(); view++)
  {
    float r = (rand () / (RAND_MAX + 1.0));
    float g = (rand () / (RAND_MAX + 1.0));
    float b = (rand () / (RAND_MAX + 1.0));

    int R = 255 * r;
    int G = 255 * g;
    int B = 255 * b;

    for (int cub = 0; cub < cubs.at(view).size(); cub++)
    {
      std::stringstream cub_id;
      cub_id << "cub-id-" << getTimestamp();
      viewer.addCuboid (cubs.at(view).at(cub), r, g, b, 0.5, cub_id.str ());

      std::stringstream id;
      id << "id-" << getTimestamp();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> color (clouds_of_cubs.at(view).at(cub), R, G, B);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cubs.at(view).at(cub), color, id.str());
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.str());
    }

    for (int cyl = 0; cyl < cyls.at(view).size(); cyl++)
    {
      std::stringstream cyl_id;
      cyl_id << "cyl-id-" << getTimestamp();
      viewer.addCylinder (cyls.at(view).at(cyl), r, g, b, 0.5, cyl_id.str ());

      std::stringstream id;
      id << "id-" << getTimestamp ();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> color (clouds_of_cyls.at(view).at(cyl), R, G, B);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (clouds_of_cyls.at(view).at(cyl), color, id.str());
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.str());
    }

    viewer.spin();
  }
  viewer.removeAllShapes ();
  viewer.removeAllPointClouds ();





  cerr << endl;
  bool match_not_found;
  std::vector <std::vector <object> > objs;

  for (int view = 0; view < pcd_file_indices.size(); view++)
  {
    for (int cub = 0; cub < cubs.at(view).size(); cub++)
    {
      object obj;
      obj.type = 1;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cubs.at(view).at(cub), obj.centroid);
      obj.cuboid = cubs.at(view).at(cub);
      obj.cloud = clouds_of_cubs.at(view).at(cub);

      match_not_found = true;

      if (objs.size() == 0)
      {
        // create first element
        objs.resize(1);
        objs.at(0).push_back(obj);
      }
      else
      {
        for (int idx = 0; idx < objs.size(); idx++)
        {
          double distance_between_centroids = sqrt( _sqr(objs.at(idx).at(0).centroid[0]-obj.centroid[0]) + _sqr(objs.at(idx).at(0).centroid[1]-obj.centroid[1]) + _sqr(objs.at(idx).at(0).centroid[2]-obj.centroid[2]) );
          cerr << setprecision(5) << " " << distance_between_centroids << " " << endl;


          if (distance_between_centroids < threshold_between_centroids_of_cuboids)
          {
            // add another element
            objs.at(idx).push_back(obj);

            match_not_found = false;
            cerr << " Match was found. " << endl;
          }
        }

        if ( match_not_found )
        {
          // create another element
          int size = objs.size();
          objs.resize(objs.size() + 1);
          objs.at(objs.size() - 1).push_back(obj);
        }
      }
      cerr << endl;
    }

    for (int cyl = 0; cyl < cyls.at(view).size(); cyl++)
    {
      object obj;
      obj.type = 2;
      pcl::compute3DCentroid<pcl::PointXYZRGBNormalRSD> (*clouds_of_cyls.at(view).at(cyl), obj.centroid);
      obj.cylinder = cyls.at(view).at(cyl);
      obj.cloud = clouds_of_cyls.at(view).at(cyl);

      match_not_found = true;

      if (objs.size() == 0)
      {
        // create first element
        objs.resize(1);
        objs.at(0).push_back(obj);
      }
      else
      {
        for (int idx = 0; idx < objs.size(); idx++)
        {
          double distance_between_centroids = sqrt( _sqr(objs.at(idx).at(0).centroid[0]-obj.centroid[0]) + _sqr(objs.at(idx).at(0).centroid[1]-obj.centroid[1]) + _sqr(objs.at(idx).at(0).centroid[2]-obj.centroid[2]) );
          cerr << setprecision(5) << " " << distance_between_centroids << " " << endl;

          if (distance_between_centroids < threshold_between_centroids_of_cylinders)
          {
            // add another element
            objs.at(idx).push_back(obj);

            match_not_found = false;
            cerr << " Match was found. " << endl;
          }
        }

        if ( match_not_found )
        {
          // create another element
          int size = objs.size();
          objs.resize(objs.size() + 1);
          objs.at(objs.size() - 1).push_back(obj);
        }
      }
      cerr << endl;
    }
  }

  cerr << objs.size() << " objects " << endl;
  for (int idx = 0; idx < objs.size(); idx++)
    cerr << " object " << idx << " has " << objs.at(idx).size() << " instances " << endl;
  cerr << endl;





  for (int obj = 0; obj < objs.size(); obj++)
  {
    float r = (rand () / (RAND_MAX + 1.0));
    float g = (rand () / (RAND_MAX + 1.0));
    float b = (rand () / (RAND_MAX + 1.0));

    int R = 255 * r;
    int G = 255 * g;
    int B = 255 * b;

    for (int inst = 0; inst < objs.at(obj).size(); inst++)
    {
      std::stringstream obj_id;
      obj_id << "obj-id-" << getTimestamp();
      if (objs.at(obj).at(inst).type == 1)
        viewer.addCuboid     (objs.at(obj).at(inst).cuboid, r, g, b, 0.5, obj_id.str ());
      if (objs.at(obj).at(inst).type == 2)
        viewer.addCylinder (objs.at(obj).at(inst).cylinder, r, g, b, 0.5, obj_id.str ());

      std::stringstream id;
      id << "id-" << getTimestamp();
      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> color (objs.at(obj).at(inst).cloud, R, G, B);
      viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (objs.at(obj).at(inst).cloud, color, id.str());
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.str());
    }

    viewer.spin();
  }
  viewer.removeAllShapes ();
  viewer.removeAllPointClouds ();





  {
    FILE * txt;
    txt = fopen ("sequential-hough-ransac-numbers.txt", "a");

    for (int obj = 0; obj < objs.size(); obj++)
    {
      if (objs.at(obj).at(0).type == 1)
      {
        fprintf (txt, "\n----------------------------------------------------------------------------------------------------\n\n");
        fprintf (txt, "  model %d [box]\n\n", obj);

        for (int inst = 0; inst < objs.at(obj).size(); inst++)
        {
          double d1 = sqrt (_sqr (objs.at(obj).at(inst).cuboid.at(0).values.at(0) - objs.at(obj).at(inst).cuboid.at(1).values.at(0)) + _sqr (objs.at(obj).at(inst).cuboid.at(0).values.at(1) - objs.at(obj).at(inst).cuboid.at(1).values.at(1)));
          double d2 = sqrt (_sqr (objs.at(obj).at(inst).cuboid.at(1).values.at(0) - objs.at(obj).at(inst).cuboid.at(2).values.at(0)) + _sqr (objs.at(obj).at(inst).cuboid.at(1).values.at(1) - objs.at(obj).at(inst).cuboid.at(2).values.at(1)));
          double d3 =             objs.at(obj).at(inst).cuboid.at(4).values.at(2) - objs.at(obj).at(inst).cuboid.at(0).values.at(2);
          double v  = d1 * d2 * d3;

          fprintf (txt, "    view %d | %12.10f x %12.10f x %12.10f = %12.10f | %12.10f %12.10f %12.10f \n", inst, d1, d2, d3, v, objs.at(obj).at(inst).centroid[0], objs.at(obj).at(inst).centroid[1], objs.at(obj).at(inst).centroid[2]);
        }
      }

      if (objs.at(obj).at(0).type == 2)
      {
        fprintf (txt, "\n----------------------------------------------------------------------------------------------------\n\n");
        fprintf (txt, "  model %d [cylinder]\n\n", obj);

        for (int inst = 0; inst < objs.at(obj).size(); inst++)
        {
          double h = objs.at(obj).at(inst).cylinder.values.at(5);
          double r = objs.at(obj).at(inst).cylinder.values.at(6);
          double v = M_PI * _sqr(r) * h;

          fprintf (txt, "    view %d | pi x %12.10f ^2 x %12.10f = %12.10f | %12.10f %12.10f %12.10f \n", inst, r, h, v, objs.at(obj).at(inst).centroid[0], objs.at(obj).at(inst).centroid[1], objs.at(obj).at(inst).centroid[2]);
        }
      }
    }

    fprintf (txt, "\n----------------------------------------------------------------------------------------------------\n");
    fprintf (txt,   "----------------------------------------------------------------------------------------------------\n");
    fprintf (txt,   "----------------------------------------------------------------------------------------------------\n");

    // fclose (txt); // just, NOT to save this file
  }





  double colors[6][3];

  //colors[0][0] = 1.0; colors[0][1] = 0.0; colors[0][2] = 0.0; // red
  //colors[1][0] = 0.0; colors[1][1] = 1.0; colors[1][2] = 0.5; // green-cyan
  //colors[2][0] = 1.0; colors[2][1] = 0.0; colors[2][2] = 1.0; // pink
  //colors[3][0] = 0.5; colors[3][1] = 1.0; colors[3][2] = 0.0; // green-yellow
  //colors[4][0] = 0.0; colors[4][1] = 0.0; colors[4][2] = 1.0; // blue
  //colors[5][0] = 1.0; colors[5][1] = 0.5; colors[5][2] = 0.0; // red-yellow
  //colors[6][0] = 0.0; colors[6][1] = 1.0; colors[6][2] = 1.0; // cyan

  colors[0][0] = 1.0; colors[0][1] = 0.0; colors[0][2] = 0.0; // red
  colors[1][0] = 0.0; colors[1][1] = 1.0; colors[1][2] = 0.0; // green
  colors[2][0] = 0.0; colors[2][1] = 0.0; colors[2][2] = 1.0; // blue
  colors[3][0] = 1.0; colors[3][1] = 1.0; colors[3][2] = 0.0; // yellow
  colors[4][0] = 0.0; colors[4][1] = 1.0; colors[4][2] = 1.0; // cyan
  colors[5][0] = 1.0; colors[5][1] = 0.0; colors[5][2] = 1.0; // pink
  colors[6][0] = 0.5; colors[6][1] = 0.5; colors[6][2] = 0.5; // grey





  std::vector <object> merged_objs;

  for (int obj = 0; obj < objs.size(); obj++)
  {
    float r = (rand () / (RAND_MAX + 1.0));
    float g = (rand () / (RAND_MAX + 1.0));
    float b = (rand () / (RAND_MAX + 1.0));

    int R = 255 * r;
    int G = 255 * g;
    int B = 255 * b;

    object obj_merged;

    if (objs.at(obj).at(0).type == 1)
    {
      std::vector<pcl::ModelCoefficients> tmp_coeffs;
      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      for (int inst = 0; inst < objs.at(obj).size(); inst++)
      {
        tmp_coeffs.resize(8);

        for (int corner = 0; corner < tmp_coeffs.size(); corner++)
        {
          tmp_coeffs[corner].values.push_back ((objs.at(obj).at(inst).cuboid.at(corner).values.at(0) + objs.at(obj).at(inst).cuboid.at(corner).values.at(0)) /2);
          tmp_coeffs[corner].values.push_back ((objs.at(obj).at(inst).cuboid.at(corner).values.at(1) + objs.at(obj).at(inst).cuboid.at(corner).values.at(1)) /2);

          if (corner < 4)
            tmp_coeffs[corner].values.push_back (std::min (objs.at(obj).at(inst).cuboid.at(corner).values.at(2), objs.at(obj).at(inst).cuboid.at(corner).values.at(2)));
          else
            tmp_coeffs[corner].values.push_back (std::max (objs.at(obj).at(inst).cuboid.at(corner).values.at(2), objs.at(obj).at(inst).cuboid.at(corner).values.at(2)));
        }

        *tmp_cloud += *objs.at(obj).at(inst).cloud;
      }

      obj_merged.type = objs.at(obj).at(0).type;
      obj_merged.cuboid = tmp_coeffs;
      obj_merged.cloud = tmp_cloud;
    }

    //

    if (objs.at(obj).at(0).type == 2)
    {
      pcl::ModelCoefficients tmp_coeffs;
      pcl::PointCloud<pcl::PointXYZRGBNormalRSD>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZRGBNormalRSD> ());

      double mean_cx = 0;
      double mean_cy = 0;
      double mean_r  = 0;
      double min_z = DBL_MAX;
      double max_z = DBL_MIN;

      for (int inst = 0; inst < objs.at(obj).size(); inst++)
      {
        mean_cx = mean_cx + objs.at(obj).at(inst).cylinder.values[0];
        mean_cy = mean_cy + objs.at(obj).at(inst).cylinder.values[1];
        mean_r  = mean_r  + objs.at(obj).at(inst).cylinder.values[6];

        if (min_z > objs.at(obj).at(inst).cylinder.values[2])
          min_z = objs.at(obj).at(inst).cylinder.values[2];
        if (max_z < objs.at(obj).at(inst).cylinder.values[5])
          max_z = objs.at(obj).at(inst).cylinder.values[5];

        *tmp_cloud += *objs.at(obj).at(inst).cloud;
      }

      mean_cx = mean_cx / objs.at(obj).size();
      mean_cy = mean_cy / objs.at(obj).size();
      mean_r  = mean_r  / objs.at(obj).size();

      tmp_coeffs.values.resize(7);
      tmp_coeffs.values[0] = mean_cx;
      tmp_coeffs.values[1] = mean_cy;
      tmp_coeffs.values[2] = min_z;
      tmp_coeffs.values[3] = 0.0;
      tmp_coeffs.values[4] = 0.0;
      tmp_coeffs.values[5] = max_z;
      tmp_coeffs.values[6] = mean_r;

      obj_merged.type = objs.at(obj).at(0).type;
      obj_merged.cylinder = tmp_coeffs;
      obj_merged.cloud = tmp_cloud;
    }

    merged_objs.push_back(obj_merged);
  }





  for (int obj = 0; obj < merged_objs.size(); obj++)
  {
    float r = (rand () / (RAND_MAX + 1.0));
    float g = (rand () / (RAND_MAX + 1.0));
    float b = (rand () / (RAND_MAX + 1.0));

    int R = 255 * r;
    int G = 255 * g;
    int B = 255 * b;

    std::stringstream obj_id;
    obj_id << "obj-id-" << getTimestamp();
    if (merged_objs.at(obj).type == 1) viewer.addCuboid (merged_objs.at(obj).cuboid, r, g, b, 0.5, obj_id.str());
    if (merged_objs.at(obj).type == 2) viewer.addCylinder (merged_objs.at(obj).cylinder, r, g, b, 0.5, obj_id.str());

    std::stringstream id;
    id << "id-" << getTimestamp();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormalRSD> color (merged_objs.at(obj).cloud, R, G, B);
    viewer.addPointCloud<pcl::PointXYZRGBNormalRSD> (merged_objs.at(obj).cloud, color, id.str());
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, size, id.str());

    viewer.spin();
  }
  viewer.removeAllShapes ();
  viewer.removeAllPointClouds ();

*             *
* Big Section *
*             */





  printf (" \n \n \n ");
  std::string aux = "/media/go/data/tu-munich/concat-t0_cut3-unsmoothed-unfiltered.pcd";
  const char* dataset = aux.c_str();

//  char dataset[255];
//  printf ("\n\n\n");
//  printf ("Enter original dataset: ");
//   scanf ("%s", dataset);

  // ---------- Load Original Data Set ---------- //

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

  if (pcl::io::loadPCDFile (dataset, *cloud) == -1)
  {
    pcl::console::print_error ("Couldn't read file %s\n", dataset);
    return (-1);
  }

  if (verbose) pcl::console::print_info ("Loaded %d data points from %s with the following fields: %s\n", (int) (cloud->points.size ()), dataset, pcl::getFieldsList (*cloud).c_str ());

  if (true)
  {
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color (cloud, 0, 0, 0);
    viewer.addPointCloud<pcl::PointXYZRGB> (cloud, color, "generic");
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "generic");
    viewer.spin ();
  }

  // ---------- Path and Name of File ---------- //

  std::string file = dataset;
  size_t f = file.find_last_of (".");
  std::string directory = file.substr (0, f);

  cerr << f << endl ;
  cerr << directory << endl;

  // ---------- Visualize Detected Objects ---------- //

  int view = 0;

  for (int cub_idx = 0; cub_idx < cubs.at(view).size (); cub_idx++)
  {
    float r = (rand () / (RAND_MAX + 1.0));
    float g = (rand () / (RAND_MAX + 1.0));
    float b = (rand () / (RAND_MAX + 1.0));

    int R = 255 * r;
    int G = 255 * g;
    int B = 255 * b;

    std::stringstream cub_id;
    cub_id << "cuboid_" << getTimestamp ();
    viewer.addCuboid (cubs.at(view).at(cub_idx), r, g, b, 0.75, cub_id.str ());
  }

  viewer.spin ();

  for (int cyl_idx = 0; cyl_idx < cyls.at(view).size (); cyl_idx++)
  {
    float r = (rand () / (RAND_MAX + 1.0));
    float g = (rand () / (RAND_MAX + 1.0));
    float b = (rand () / (RAND_MAX + 1.0));

    int R = 255 * r;
    int G = 255 * g;
    int B = 255 * b;

    std::stringstream cyl_id;
    cyl_id << "cylinder_" << getTimestamp ();
    viewer.addCylinder (cyls.at(view).at(cyl_idx), r, g, b, 0.75, cyl_id.str ());
  }

  viewer.spin ();

  viewer.removeAllShapes ();
  viewer.spin ();

  std::vector<pcl::PointIndices::Ptr>                 indices_of_cylinders;
  std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>  clouds_of_cylinders;

  for (int cyl_idx = 0; cyl_idx < cyls.at(view).size (); cyl_idx++)
  {
    Vec3 pt1;
    pt1.x        = cyls.at (view).at (cyl_idx).values.at (0);
    pt1.y        = cyls.at (view).at (cyl_idx).values.at (1);
    pt1.z        = cyls.at (view).at (cyl_idx).values.at (2);
    Vec3 pt2;
    pt2.x        = cyls.at (view).at (cyl_idx).values.at (0);
    pt2.y        = cyls.at (view).at (cyl_idx).values.at (1);
    pt2.z        = cyls.at (view).at (cyl_idx).values.at (5) + cyls.at (view).at (cyl_idx).values.at (2);

    float radius = cyls.at (view).at (cyl_idx).values.at (6);

    float sqd_length = _sqr(pt2.x - pt1.x) + _sqr(pt2.y - pt1.y) + _sqr(pt2.z - pt1.z);
    float sqd_radius = _sqr(radius + circle_threshold /**//2/**/);

    pcl::PointIndices::Ptr                 cylinder_indices (new pcl::PointIndices ());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cylinder_cloud   (new pcl::PointCloud<pcl::PointXYZRGB> ());

    for (int pnt_idx = 0; pnt_idx < cloud->points.size (); pnt_idx++)
    {
      Vec3 pnt;
      pnt.x = cloud->points.at (pnt_idx).x;
      pnt.y = cloud->points.at (pnt_idx).y;
      pnt.z = cloud->points.at (pnt_idx).z;

      float dist = isPointInsideCylinder (pt1, pt2, sqd_length, sqd_radius, pnt);

      if (dist > 0) cylinder_indices->indices.push_back (pnt_idx);
    }

    // Save for later...
    indices_of_cylinders.push_back (cylinder_indices);

    pcl::ExtractIndices<pcl::PointXYZRGB> cyl_ei;
    cyl_ei.setInputCloud (cloud);
    cyl_ei.setIndices (cylinder_indices);
    cyl_ei.setNegative (false);
    cyl_ei.filter (*cylinder_cloud);

    // Save for later...
    clouds_of_cylinders.push_back (cylinder_cloud);

    float r = (rand () / (RAND_MAX + 1.0));
    float g = (rand () / (RAND_MAX + 1.0));
    float b = (rand () / (RAND_MAX + 1.0));

    int R = 255 * r;
    int G = 255 * g;
    int B = 255 * b;

    std::stringstream cyl_id;
    cyl_id << "cylinder_" << getTimestamp ();
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> cylinder_cloud_color (cylinder_cloud, R, G, B);
    viewer.addPointCloud<pcl::PointXYZRGB> (cylinder_cloud, cylinder_cloud_color, cyl_id.str ());
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cyl_id.str ());
    viewer.spin ();
  }



  viewer.spin ();

  return (0);
}
