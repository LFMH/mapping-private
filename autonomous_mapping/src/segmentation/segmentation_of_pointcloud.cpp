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
#include "pcl/common/common.h"
#include "pcl/features/normal_3d.h"
#include "pcl/filters/passthrough.h"
#include <pcl/filters/voxel_grid.h>
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/statistical_outlier_removal.h"
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/sac_model_circle.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/segmentation/extract_clusters.h"

// pcl visualization dependencies
#include "pcl_visualization/pcl_visualizer.h"



// Segmentation's Parameters
double threshold = 0.075; /// [percentage]
double floor_limit = 0.025; /// [percentage]
double ceiling_limit = 0.100; /// [percentage]

// Method's Parameters
double plane_threshold = 0.100; /// [meters]
int minimum_plane_inliers = 100; /// [points]
int maximum_plane_iterations = 1000; /// [iterations]
double plane_inliers_clustering_tolerance = 0.100; /// [meters]

// Visualization's Parameters
bool step = false;
bool clean = false;
int size_of_points = 1;

//Optional parameters
bool find_box_model = false;

#include <pcl_ias_sample_consensus/pcl_sac_model_orientation.h>
//#include <pointcloud_segmentation/box_fit2_algo.h>
//#include <pcl_cloud_algos/box_fit2_algo.h>

// Sample Consensus
//#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/impl/ransac.hpp>
// For computeCentroid
//#include <point_cloud_mapping/geometry/nearest.h>

//template class pcl::RandomSampleConsensus<pcl::Normal>;

//using namespace std;
//using namespace pcl_cloud_algos;
//using namespace pcl;
//using namespace sample_consensus;



void getMinAndMax (boost::shared_ptr<const pcl::PointCloud <pcl::PointXYZINormal> > cloud_, Eigen::VectorXf model_coefficients, boost::shared_ptr<pcl::SACModelOrientation<pcl::Normal> > model, std::vector<int> &min_max_indices, std::vector<float> &min_max_distances)
{

  boost::shared_ptr<std::vector<int> > inliers = model->getIndices();
  boost::shared_ptr<std::vector<int> > indices = model->getIndices();

  // Initialize result vectors
  min_max_indices.resize (6);
  min_max_distances.resize (6);
  min_max_distances[0] = min_max_distances[2] = min_max_distances[4] = +DBL_MAX;
  min_max_distances[1] = min_max_distances[3] = min_max_distances[5] = -DBL_MAX;

  // The 3 coordinate axes are nm, nc and axis_
  Eigen::Vector3f nm;
  nm[0] = model_coefficients[0];
  nm[1] = model_coefficients[1];
  nm[2] = model_coefficients[2];
  //Eigen::Vector3f nm = Eigen::Vector3d::Map(&(*model_coefficients)[0]).cast<float> ();
  Eigen::Vector3f nc = model->axis_.cross (nm);

  // Find minimum and maximum distances from origin along the three axes
  for (std::vector<int>::iterator it = inliers->begin (); it != inliers->end (); it++)
  //for (unsigned i = 0; i < inliers.size (); i++)
  {
    // @NOTE inliers is a list of indices of the indices_ array!
    Eigen::Vector3f point (cloud_->points[(*indices)[*it]].x, cloud_->points[(*indices)[*it]].y, cloud_->points[(*indices)[*it]].z);
    //Eigen::Vector3f point (cloud_->points[indices_[*it]].x - center.x, cloud_->points[indices_[*it]].y - center.y, cloud_->points[indices_[*it]].z - center.z);
    //Eigen::Vector3f point (cloud_->points[indices_[inliers[i]]].x, cloud_->points[indices_[inliers[i]]].y, cloud_->points[indices_[inliers[i]]].z);
    double dists[3];
    dists[0] = nm.dot(point);
    dists[1] = nc.dot(point);
    dists[2] = model->axis_.dot(point);
    for (int d=0; d<3; d++)
    {
      if (min_max_distances[2*d+0] > dists[d]) { min_max_distances[2*d+0] = dists[d]; min_max_indices[2*d+0] = *it; }
      if (min_max_distances[2*d+1] < dists[d]) { min_max_distances[2*d+1] = dists[d]; min_max_indices[2*d+1] = *it; }
    }
  }

}



////////////////////////////////////////////////////////////////////////////////
/**
 * actual model fitting happens here
 */
bool find_model (boost::shared_ptr<const pcl::PointCloud <pcl::PointXYZINormal> > cloud, std::vector<double> &coeff, double eps_angle_ = 0.1, double success_probability_ = 0.99, int verbosity_level_ = 1)
{

  std::cerr << endl << "  A  " << endl << endl; 

  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Looking for box in a cluster of %u points", (unsigned)cloud->points.size ());

  // Compute center point
  //cloud_geometry::nearest::computeCentroid (*cloud, box_centroid_);
  pcl::PointCloud<pcl::Normal> nrmls ;
  nrmls.header = cloud->header;
  nrmls.points.resize(cloud->points.size());
  for(size_t i = 0 ; i < cloud->points.size(); i++)
  {
    nrmls.points[i].normal[0] = cloud->points[i].normal[0];
    nrmls.points[i].normal[1] = cloud->points[i].normal[1];
    nrmls.points[i].normal[2] = cloud->points[i].normal[2];
  }

  std::cerr << endl << "  B  " << endl << endl; 

  // Create model
  //pcl::SACModelOrientation<pcl::Normal> model1 (nrmls.makeShared ());
  pcl::SACModelOrientation<pcl::Normal>::Ptr model = boost::make_shared<pcl::SACModelOrientation<pcl::Normal> >(boost::make_shared<pcl::PointCloud<pcl::Normal> > (nrmls));
  // SACModelOrientation<pcl::Normal> model(nrmls);

  std::cerr << endl << "  C  " << endl << endl; 

  model->axis_[0] = 0 ;
  model->axis_[1] = 0 ;
  model->axis_[2] = 1 ;

  //model->setDataSet ((sensor_msgs::PointCloud*)(cloud.get())); // TODO: this is nasty :)
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Axis is (%g,%g,%g) and maximum angular difference %g",
      model->axis_[0], model->axis_[1], model->axis_[2], eps_angle_);

  // Check probability of success and decide on method
  Eigen::VectorXf refined;
  std::vector<int> inliers;
  /// @NOTE: inliers are actually indexes in the indices_ array, but that is not set (by default it has all the points in the correct order)
  if (success_probability_ > 0 && success_probability_ < 1)
  {
    if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Using RANSAC with stop probability of %g and model refinement", success_probability_);

    // Fit model using RANSAC
    pcl::RandomSampleConsensus<pcl::Normal> *sac = new pcl::RandomSampleConsensus<pcl::Normal> (model, eps_angle_);
    sac->setProbability (success_probability_);
    if (!sac->computeModel ())
    {
      if (verbosity_level_ > -2) ROS_ERROR ("[RobustBoxEstimation] No model found using the angular threshold of %g!", eps_angle_);
      return false;
    }

    // Get inliers and refine result
    sac->getInliers(inliers);
    if (verbosity_level_ > 1) cerr << "number of inliers: " << inliers.size () << endl;
    // Exhaustive search for best model
    std::vector<int> best_sample;
    std::vector<int> best_inliers;
    Eigen::VectorXf model_coefficients;
    for (unsigned i = 0; i < cloud->points.size (); i++)
    {
      std::vector<int> selection (1);
      selection[0] = i;
      model->computeModelCoefficients (selection, model_coefficients);

      model->selectWithinDistance (model_coefficients, eps_angle_, inliers);
      if (best_inliers.size () < inliers.size ())
      {
        best_inliers = inliers;
        best_sample = selection;
      }
    }

    // Check if successful and save results
    if (best_inliers.size () > 0)
    {
      model->computeModelCoefficients (best_sample, refined);
      //model->getModelCoefficients (refined);
      /// @NOTE: making things transparent for the outside... not really needed
      inliers = best_inliers;
      //model->setBestModel (best_sample);
      //model->setBestInliers (best_inliers);
      // refine results: needs inliers to be set!
      // sac->refineCoefficients(refined);
    }
    /// @NOTE best_model_ contains actually the samples used to find the best model!
    //model->computeModelCoefficients(model->getBestModel ());
    //Eigen::VectorXf original;
    //model->getModelCoefficients (original);
    //if (verbosity_level_ > 1) cerr << "original direction: " << original[0] << " " << original[1] << " " << original[2] << ", found at point nr " << original[3] << endl;
    //sac->refineCoefficients(refined);
   // if (verbosity_level_ > 1) cerr << "refitted direction: " << refined.at (0) << " " << refined.at (1) << " " << refined.at (2) << ", initiated from point nr " << refined.at (3) << endl;
   // if (refined[3] == -1)
   //   refined = original;
  }
  else
  {
    if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Using exhaustive search in %ld points", (long int) cloud->points.size ());

    // Exhaustive search for best model
    std::vector<int> best_sample;
    std::vector<int> best_inliers;
    Eigen::VectorXf model_coefficients;
    for (unsigned i = 0; i < cloud->points.size (); i++)
    {
      std::vector<int> selection (1);
      selection[0] = i;
      model->computeModelCoefficients (selection, model_coefficients);

      model->selectWithinDistance (model_coefficients, eps_angle_, inliers);
      if (best_inliers.size () < inliers.size ())
      {
        best_inliers = inliers;
        best_sample = selection;
      }
    }

    // Check if successful and save results
    if (best_inliers.size () > 0)
    {
      model->computeModelCoefficients (best_sample, refined);
      //model->getModelCoefficients (refined);
      /// @NOTE: making things transparent for the outside... not really needed
      inliers = best_inliers;
      //model->setBestModel (best_sample);
      //model->setBestInliers (best_inliers);
      // refine results: needs inliers to be set!
      // sac->refineCoefficients(refined);
    }
    else
    {
      if (verbosity_level_ > -2) ROS_ERROR ("[RobustBoxEstimation] No model found using the angular threshold of %g!", eps_angle_);
      return false;
    }
  }

  // Save fixed axis
  coeff[12+0] = model->axis_[0];
  coeff[12+1] = model->axis_[1];
  coeff[12+2] = model->axis_[2];

  // Save complementary axis (cross product)
  coeff[9+0] = model->axis_[1]*refined[2] - model->axis_[2]*refined[1];
  coeff[9+1] = model->axis_[2]*refined[0] - model->axis_[0]*refined[2];
  coeff[9+2] = model->axis_[0]*refined[1] - model->axis_[1]*refined[0];

  // Save principle axis (corrected)
  refined[0] = - (model->axis_[1]*coeff[9+2] - model->axis_[2]*coeff[9+1]);
  refined[1] = - (model->axis_[2]*coeff[9+0] - model->axis_[0]*coeff[9+2]);
  refined[2] = - (model->axis_[0]*coeff[9+1] - model->axis_[1]*coeff[9+0]);

  ROS_INFO("refined[0]: %f", refined[0]);
  ROS_INFO("refined[1]: %f", refined[1]);
  ROS_INFO("refined[2]: %f", refined[2]);
  coeff[6+0] = refined[0];
  coeff[6+1] = refined[1];
  coeff[6+2] = refined[2];

  /*// Save complementary axis (AGIAN, JUST TO MAKE SURE)
  coeff[9+0] = model->axis_[1]*refined[2] - model->axis_[2]*refined[1];
  coeff[9+1] = model->axis_[2]*refined[0] - model->axis_[0]*refined[2];
  coeff[9+2] = model->axis_[0]*refined[1] - model->axis_[1]*refined[0];*/

  // Compute minimum and maximum along each dimension for the whole cluster
  std::vector<int> min_max_indices;
  std::vector<float> min_max_distances;
  //boost::shared_ptr<vector<int> > indices (new vector<int>);
  //indices = model->getIndices();

  //model->getMinAndMax (&refined, &inliers, min_max_indices, min_max_distances);
  //getMinAndMax (&refined, model->getIndices (), min_max_indices, min_max_distances);
  getMinAndMax (cloud, refined, model, min_max_indices, min_max_distances);
  //vector<int> min_max_indices = model->getMinAndMaxIndices (refined);

  //cerr << min_max_distances.at (1) << " " << min_max_distances.at (0) << endl;
  //cerr << min_max_distances.at (3) << " " << min_max_distances.at (2) << endl;
  //cerr << min_max_distances.at (5) << " " << min_max_distances.at (4) << endl;

  // Save dimensions
  coeff[3+0] = min_max_distances.at (1) - min_max_distances.at (0);
  coeff[3+1] = min_max_distances.at (3) - min_max_distances.at (2);
  coeff[3+2] = min_max_distances.at (5) - min_max_distances.at (4);

  // Distance of box's geometric center relative to origin along orientation axes
  double dist[3];
  dist[0] = min_max_distances[0] + coeff[3+0] / 2;
  dist[1] = min_max_distances[2] + coeff[3+1] / 2;
  dist[2] = min_max_distances[4] + coeff[3+2] / 2;

  // Compute position of the box's geometric center in XYZ
  coeff[0] = dist[0]*coeff[6+0] + dist[1]*coeff[9+0] + dist[2]*coeff[12+0];
  coeff[1] = dist[0]*coeff[6+1] + dist[1]*coeff[9+1] + dist[2]*coeff[12+1];
  coeff[2] = dist[0]*coeff[6+2] + dist[1]*coeff[9+2] + dist[2]*coeff[12+2];
  //coeff[0] = box_centroid_.x + dist[0]*coeff[6+0] + dist[1]*coeff[9+0] + dist[2]*coeff[12+0];
  //coeff[1] = box_centroid_.y + dist[0]*coeff[6+1] + dist[1]*coeff[9+1] + dist[2]*coeff[12+1];
  //coeff[2] = box_centroid_.z + dist[0]*coeff[6+2] + dist[1]*coeff[9+2] + dist[2]*coeff[12+2];
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Cluster center x: %g, y: %g, z: %g", coeff[0], coeff[1], coeff[2]);

  // Print info
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Dimensions x: %g, y: %g, z: %g",
      coeff[3+0], coeff[3+1], coeff[3+2]);
  if (verbosity_level_ > 0) ROS_INFO ("[RobustBoxEstimation] Direction vectors: \n\t%g %g %g \n\t%g %g %g \n\t%g %g %g",
      coeff[3+3], coeff[3+4], coeff[3+5],
      coeff[3+6], coeff[3+7], coeff[3+8],
      coeff[3+9], coeff[3+10],coeff[3+11]);

  return true;
}






/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Main routine of the method. Segmentation of point cloud data.
 */
int main (int argc, char** argv)
{

  // Initialize random number generator
  srand (time(0));

  // Declare the timer
  terminal_tools::TicToc tt;

  // Starting timer
  tt.tic ();


  
  // --------------------------------------------------------------- //
  // ------------------ Check and parse arguments ------------------ //
  // --------------------------------------------------------------- //

  // Argument check and info about
  if (argc < 2)
  {
    ROS_INFO (" ");
    ROS_INFO ("Syntax is: %s <input>.pcd <options>", argv[0]);
    ROS_INFO ("where <options> are: -threshold X                            = threshold for line inlier selection");
    ROS_INFO ("                     -floor_limit X                          = ");
    ROS_INFO ("                     -ceiling_limit X                        = ");
    ROS_INFO (" ");
    ROS_INFO ("                     -plane_threshold X                      = ");
    ROS_INFO ("                     -minimum_plane_inliers X                = ");
    ROS_INFO ("                     -maximum_plane_iterations X             = ");
    ROS_INFO ("                     -plane_inliers_clustering_tolarence     = ");
    ROS_INFO (" ");
    ROS_INFO ("                     -step B                                 = wait or not wait");
    ROS_INFO ("                     -clean B                                = remove or not remove the clouds from viewer");
    ROS_INFO ("                     -size_of_points D                       = set the size of points");
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

  // Parsing the arguments for segmenation
  terminal_tools::parse_argument (argc, argv, "-threshold", threshold);
  terminal_tools::parse_argument (argc, argv, "-floor_limit", floor_limit);
  terminal_tools::parse_argument (argc, argv, "-ceiling_limit", ceiling_limit);

  // Parsing the arguments for fitting plane models
  terminal_tools::parse_argument (argc, argv, "-plane_threshold", plane_threshold);
  terminal_tools::parse_argument (argc, argv, "-minimum_plane_inliers", minimum_plane_inliers);
  terminal_tools::parse_argument (argc, argv, "-maximum_plane_iterations", maximum_plane_iterations);
  terminal_tools::parse_argument (argc, argv, "-plane_inliers_clustering_tolerance", plane_inliers_clustering_tolerance);

  // Parsing the arguments for visualization
  terminal_tools::parse_argument (argc, argv, "-step", step);
  terminal_tools::parse_argument (argc, argv, "-clean", clean);
  terminal_tools::parse_argument (argc, argv, "-size_of_points", size_of_points);

  // Parsing the optional arguments
  terminal_tools::parse_argument (argc, argv, "-find_box_model", find_box_model);

  ROS_WARN ("Timer started !");
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");

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
  


  // --------------------------------------------------------------- //
  // ------------------ Load the point cloud data ------------------ //
  // --------------------------------------------------------------- //

  // Input point cloud data
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());

  // Load point cloud data
  if (pcl::io::loadPCDFile (argv[pFileIndicesPCD[0]], *input_cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv[pFileIndicesPCD[0]]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*input_cloud).c_str ());

  // Add the input cloud
  viewer.addPointCloud (*input_cloud, "INPUT");
  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 0.0, "INPUT");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, "INPUT"); 
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("INPUT");
    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }



/*

  // ------------------------------------------------------------------------ //
  // ------------------ Removal of outliers for input data ------------------ //
  // ------------------------------------------------------------------------ //

  // Filtered point cloud
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZINormal> sor;
  // Set which point cloud to filter
  sor.setInputCloud (input_cloud);
  // Set number of points for mean distance estimation
  sor.setMeanK (25);
  // Set the standard deviation multiplier threshold
  sor.setStddevMulThresh (1.0);
  // Call the filtering method
  sor.filter (*filtered_cloud);

  ROS_INFO ("Statistical Outlier Removal ! before: %d points | after: %d points | filtered: %d points", input_cloud->points.size (),  filtered_cloud->points.size (), input_cloud->points.size () - filtered_cloud->points.size ());

  // Add the filtered cloud 
  viewer.addPointCloud (*filtered_cloud, "FILTERED");
  // Color the cloud in yellow
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 0.0, "FILTERED");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, "FILTERED"); 
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove the point cloud data
  viewer.removePointCloud ("FILTERED");
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

*/







  // ---------------------------------------------------------------- //
  // ------------------ Start filtering cloud data ------------------ //
  // ---------------------------------------------------------------- //

  pcl::PointXYZINormal minimum_point, maximum_point;

  pcl::getMinMax3D (*input_cloud, minimum_point, maximum_point);

  ROS_INFO ("Minimum point is (%6.3f,%6.3f,%6.3f)", minimum_point.x, minimum_point.y, minimum_point.z);
  ROS_INFO ("Maximum point is (%6.3f,%6.3f,%6.3f)", maximum_point.x, maximum_point.y, maximum_point.z);
  
  double height_of_cloud = maximum_point.z - minimum_point.z;
  ROS_INFO ("Height is %5.3f meters", height_of_cloud);

  threshold = height_of_cloud * threshold;
  floor_limit = height_of_cloud * floor_limit;
  ceiling_limit = height_of_cloud * ceiling_limit;

  ROS_INFO ("      threshold = %5.3f meters", threshold);
  ROS_INFO ("    floor limit = %5.3f meters", floor_limit);
  ROS_INFO ("  ceiling limit = %5.3f meters", ceiling_limit);





  //Calculate the centroid of the cluster
  //point_center_.x = (point_max_.x + point_min_.x)/2;
  //point_center_.y = (point_max_.y + point_min_.y)/2;
  //point_center_.z = (point_max_.z + point_min_.z)/2;
  //ss_position << point_center_.x << "_" << point_center_.y << "_" << point_center_.z;



  //std::cerr << "Cloud before filtering: " << std::endl;
  //for (size_t i = 0; i < cloud->points.size (); ++i)
  //std::cerr << "    " << cloud->points[i].x << " " << cloud->points[i].y << " " << cloud->points[i].z << std::endl;

  //ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*input_cloud).c_str ());


 
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZINormal> pass;
  pass.setInputCloud (input_cloud);
  pass.setFilterFieldName ("z");





  // Point cloud of floor points
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr floor_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());

  // Set floor limits
  if ( floor_limit == 0.0 )
    pass.setFilterLimits (minimum_point.z, minimum_point.z + threshold);
  else
    pass.setFilterLimits (minimum_point.z, minimum_point.z + floor_limit);

  // Call the filtering function
  pass.setFilterLimitsNegative (false);
  pass.filter (*floor_cloud);
  pass.setFilterLimitsNegative (true);
  pass.filter (*input_cloud);

  // Show the floor's number of points
  ROS_INFO ("The floor has %d points !", floor_cloud->points.size ());

  // Save these points to disk
  pcl::io::savePCDFile ("data/floor_.pcd", *floor_cloud);

  // Add point cloud to viewer
  viewer.addPointCloud (*floor_cloud, "FLOOR");
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("FLOOR");
    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }








  // Point cloud of ceiling points
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr ceiling_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());

  // Set ceiling limits
  if ( ceiling_limit == 0.0 ) 
    pass.setFilterLimits (maximum_point.z - threshold, maximum_point.z);
  else
    pass.setFilterLimits (maximum_point.z - ceiling_limit, maximum_point.z);

  // Call the filtering function
  pass.setFilterLimitsNegative (false);
  pass.filter (*ceiling_cloud);
  pass.setFilterLimitsNegative (true);
  pass.filter (*input_cloud);

  // Show the ceiling's number of points
  ROS_INFO ("The ceiling has %d points !", ceiling_cloud->points.size ());

  // Save these points to disk
  pcl::io::savePCDFile ("data/ceiling_.pcd", *ceiling_cloud);

  // Add point cloud to viewer
  viewer.addPointCloud (*ceiling_cloud, "CEILING");
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  // Remove or not remove the cloud from viewer
  if ( clean )
  {
    // Remove the point cloud data
    viewer.removePointCloud ("CEILING");
    // Wait or not wait
    if ( step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }
  }



  // TODO To save point cloud after each stage 



  // Point cloud of walls points
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr walls_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());

  // Save the cloud with the walls
  *walls_cloud = *input_cloud;

  // Save these points to disk
  pcl::io::savePCDFile ("data/walls_.pcd", *walls_cloud);

  // Add the input cloud
  viewer.addPointCloud (*walls_cloud, "WALLS");
  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }

  //// Remove the point cloud data
  //viewer.removePointCloud ("WALLS");
  //// Wait or not wait
  //if ( step )
  //{
    //// And wait until Q key is pressed
    //viewer.spin ();
  //}

  ///*

  //////////////////////////////////////////////////////////////////////////////////////////////////
  // Find Box Model of a point cloud
  if (find_box_model)
  {
    pcl::VoxelGrid<pcl::PointXYZINormal> vgrid;
    vgrid.setLeafSize (0.05, 0.05, 0.05);
    vgrid.setInputCloud (input_cloud);
    pcl::PointCloud<pcl::PointXYZINormal> cloud_downsampled;
    vgrid.filter (cloud_downsampled);
    std::vector<double> coefficients (15, 0.0);
    bool yes_found_model = find_model (cloud_downsampled.makeShared(), coefficients);
    
    if (yes_found_model)
      ROS_INFO("Box Model found");
  }



  ROS_INFO ("Remaning cloud has %d poitns", input_cloud->points.size());

  ///*

  // ------------------------------------------------------------------- //
  // ------------------ Estiamte 3D normals of points ------------------ //
  // ------------------------------------------------------------------- //
 
  // Point cloud of normals
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> ());
  // Build kd-tree structure for normals
  pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr normals_tree (new pcl::KdTreeFLANN<pcl::PointXYZINormal> ());

  // Create object for normal estimation
  pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> estimation_of_normals;
  // Provide pointer to the search method
  estimation_of_normals.setSearchMethod (normals_tree);
  // Set for which point cloud to compute the normals
  estimation_of_normals.setInputCloud (input_cloud);
  // Set number of k nearest neighbors to use
  estimation_of_normals.setKSearch (50);
  // Estimate the normals
  estimation_of_normals.compute (*normals_cloud);

  //*/

  ROS_INFO ("With %d normals of course", normals_cloud->points.size());

  // All the objects needed
  //pcl::PCDReader reader;
  //pcl::PassThrough<pcl::PointXYZINormal> pass;
  //pcl::NormalEstimation<pcl::PointXYZINormal, pcl::Normal> estimation_of_normals;
  pcl::SACSegmentationFromNormals<pcl::PointXYZINormal, pcl::Normal> segmentation_of_plane;
  //pcl::PCDWriter writer;
  pcl::ExtractIndices<pcl::PointXYZINormal> extract;
  pcl::ExtractIndices<pcl::Normal> extraction_of_normals;
  //pcl::KdTreeFLANN<pcl::PointXYZINormal>::Ptr normals_tree (new pcl::KdTreeFLANN<pcl::PointXYZINormal> ());

  // Datasets
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());
  //pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> ());
  pcl::ModelCoefficients::Ptr plane_coefficients (new pcl::ModelCoefficients ()), coefficients_cylinder (new pcl::ModelCoefficients ());
  pcl::PointIndices::Ptr plane_inliers (new pcl::PointIndices ()), inliers_cylinder (new pcl::PointIndices ());



  // Create the segmentation object for the planar model and set all the parameters
  segmentation_of_plane.setOptimizeCoefficients (true);

  //segmentation_of_plane.setModelType (pcl::SACMODEL_PLANE); 
  segmentation_of_plane.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  //segmentation_of_plane.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
  //segmentation_of_plane.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
  //segmentation_of_plane.setModelType (pcl::SACMODEL_NORMAL_PARALLEL_PLANE);

  segmentation_of_plane.setNormalDistanceWeight (0.1);
  segmentation_of_plane.setMethodType (pcl::SAC_RANSAC);
  segmentation_of_plane.setMaxIterations (maximum_plane_iterations);
  segmentation_of_plane.setDistanceThreshold (plane_threshold);

  //Eigen::Vector3f x_axis;
  //x_axis[0] = 1.0;
  //x_axis[1] = 0.0;
  //x_axis[2] = 0.0;

  //Eigen::Vector3f z_axis;
  //z_axis[0] = 0.0;
  //z_axis[1] = 0.0;
  //z_axis[2] = 1.0;

  Eigen::Vector3f Z = Eigen::Vector3f (0.0, 0.0, 1.0); 

  /** \brief Set the axis along which we need to search for a model perpendicular to.
   * \param ax the axis along which we need to search for a model perpendicular to
   */
  segmentation_of_plane.setAxis (Z);
  segmentation_of_plane.setEpsAngle (0.1);

  segmentation_of_plane.setInputCloud (input_cloud);
  segmentation_of_plane.setInputNormals (normals_cloud);

  // Obtain the plane inliers and coefficients
  segmentation_of_plane.segment (*plane_inliers, *plane_coefficients);
  std::cerr << "Plane coefficients: " << *plane_coefficients << std::endl;

 

  // Point cloud of plane inliers
  pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_inliers_cloud (new pcl::PointCloud<pcl::PointXYZINormal> ());

  // Extract the circular inliers from the input cloud
  pcl::ExtractIndices<pcl::PointXYZINormal> extraction_of_plane_inliers;
  // Set point cloud from where to extract
  extraction_of_plane_inliers.setInputCloud (input_cloud);
  // Set which indices to extract
  extraction_of_plane_inliers.setIndices (plane_inliers);

  // Return the points which represent the inliers
  extraction_of_plane_inliers.setNegative (false);
  // Call the extraction function
  extraction_of_plane_inliers.filter (*plane_inliers_cloud);

  // Return the remaining points of inliers
  extraction_of_plane_inliers.setNegative (true);
  // Call the extraction function
  extraction_of_plane_inliers.filter (*input_cloud);

  ///*

  ROS_INFO ("Plane model has %d inliers", plane_inliers_cloud->points.size());
  ROS_INFO ("%d points remain in cloud after extraction", input_cloud->points.size ());

  //*/


  //// Add the input cloud
  //viewer.addPointCloud (*input_cloud, "INPUT");
  //// Wait or not wait
  //if ( step )
  //{
    //// And wait until Q key is pressed
    //viewer.spin ();
  //}

  int plane_fit = 0; 

  // Create ID for plane model
  std::stringstream plane_id;
  plane_id << "PLANE_" << plane_fit;

  // Create ID for plane inliers
  std::stringstream plane_inliers_id;
  plane_inliers_id << "PLANE_INLIERS_" << plane_fit ;

  // Add plane model to point cloud
  viewer.addPlane (*plane_coefficients, plane_id.str ());

  // Add the point cloud data
  viewer.addPointCloud (*plane_inliers_cloud, plane_inliers_id.str ());

  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, size_of_points, plane_inliers_id.str ()); 

  // Wait or not wait
  if ( step )
  {
    // And wait until Q key is pressed
    viewer.spin ();
  }









  //std::cerr << "Cloud after filtering: " << std::endl;
  //for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
  //std::cerr << "    " << cloud_filtered->points[i].x << " " << cloud_filtered->points[i].y << " " << cloud_filtered->points[i].z << std::endl;





  // Displaying the overall time
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Finished in %5.3g [s]", tt.toc ());



  // And wait until Q key is pressed
  viewer.spin ();

  return (0);
}

