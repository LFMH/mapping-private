
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

int minimum_line_inliers   = 10; /// [points]
int minimum_circle_inliers = 50; /// [points]

int maximum_line_iterations   = 100; /// [iterations]
int maximum_circle_iterations = 100; /// [iterations]

double minimum_radius = 0.010; /// [meters]
double maximum_radius = 0.100; /// [meters]

double   line_inliers_clustering_tolerance = 0.010; /// [meters]
double circle_inliers_clustering_tolerance = 0.010; /// [meters]

// Visualization's Parameters
int point_size = 3;

bool   line_step = false;
bool circle_step = false;



// Method's Main
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
    ROS_INFO ("where <options> are: -line_threshold X                 = threshold for line inlier selection");
    ROS_INFO ("                     -circle_threshold X               = threshold for circle inlier selection");
    ROS_INFO ("                     -voting_threshold X               = threshold for Hough-based model voting");

    ROS_INFO ("                     -minimum_line_inliers D           = ");
    ROS_INFO ("                     -minimum_circle_inliers D         = ");

    ROS_INFO ("                     -maximum_line_iterations D        = ");
    ROS_INFO ("                     -maximum_circle_iterations D      = ");

    ROS_INFO ("                     -line_inliers_clustering_tolerance D         = ");
    ROS_INFO ("                     -circle_inliers_clustering_tolerance D       = ");

    ROS_INFO ("                     -point_size B                     = wait or not wait");

    ROS_INFO ("                     -line_step B                      = wait or not wait");
    ROS_INFO ("                     -circle_step B                    = wait or not wait");
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

  // Parsing the arguments of method
  terminal_tools::parse_argument (argc, argv,   "-line_threshold",   line_threshold);
  terminal_tools::parse_argument (argc, argv, "-circle_threshold", circle_threshold);
  terminal_tools::parse_argument (argc, argv, "-voting_threshold", voting_threshold);

  terminal_tools::parse_argument (argc, argv, "-minimum_line_inliers",   minimum_line_inliers);
  terminal_tools::parse_argument (argc, argv, "-minimum_circle_inliers", minimum_circle_inliers);

  terminal_tools::parse_argument (argc, argv, "-maximum_line_iterations",   maximum_line_iterations);
  terminal_tools::parse_argument (argc, argv, "-maximum_circle_iterations", maximum_circle_iterations);

  terminal_tools::parse_argument (argc, argv,   "-line_inliers_clustering_tolerance",   line_inliers_clustering_tolerance);
  terminal_tools::parse_argument (argc, argv, "-circle_inliers_clustering_tolerance", circle_inliers_clustering_tolerance);

  terminal_tools::parse_argument (argc, argv,  "-point_size",  point_size);

  terminal_tools::parse_argument (argc, argv,   "-line_step",   line_step);
  terminal_tools::parse_argument (argc, argv, "-circle_step", circle_step);

  // Input point cloud data
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  // Load point cloud data
  if (pcl::io::loadPCDFile (argv[pFileIndicesPCD[0]], *input_cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv[pFileIndicesPCD[0]]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int) (input_cloud->points.size ()), argv[pFileIndicesPCD[0]], pcl::getFieldsList (*input_cloud).c_str ());



  // ----------------------------------------------------------------- //
  // ------------------ Work with pcl visualization ------------------ //
  // ----------------------------------------------------------------- //

  // Open a 3D viewer
  pcl_visualization::PCLVisualizer viewer ("3D VIEWER");
  // Set the background of viewer
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  // Add system coordiante to viewer
  viewer.addCoordinateSystem (1.0f);
  // Add the point cloud data
  viewer.addPointCloud (*input_cloud, "INPUT");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "INPUT"); 

  // Parse the camera settings and update the internal camera
  viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render.
  viewer.updateCamera ();
  // And wait until Q key is pressed
  viewer.spin ();

  // TODO add color to point clouds and to certain points from point clouds using pcl visualization

  /*

  // Color the cloud in white
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 1.0, 1.0, "input");
  // And wait until Q key is pressed
  viewer.spin ();

  */



  // ------------------------------------------------------------- //
  // ------------------ Filter point cloud data ------------------ //
  // ------------------------------------------------------------- //

  // TODO Split statistical outlier removal code from pcl visualization code

  // Filtered point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

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

  ROS_INFO ("Statistical Outlier Removal ! before: %d points | after: %d points | filtered: %d points", input_cloud->points.size (),  filtered_cloud->points.size (), input_cloud->points.size () - filtered_cloud->points.size ());

  // Remove the point cloud data
  viewer.removePointCloud ("INPUT");
  // And wait until Q key is pressed
  viewer.spin ();

  // Add the filtered point cloud data in the same viewer
  viewer.addPointCloud (*filtered_cloud, "FILTERED");
  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, "FILTERED");
  // And wait until Q key is pressed
  viewer.spin ();



  // ------------------------------------------------------------------- //
  // ------------------ Estiamte 3D normals of points ------------------ //
  // ------------------------------------------------------------------- //

  // TODO Split normal estimation code from pcl visualization code

  // Build kd-tree structure for normals
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr normals_tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());
  // Point cloud of normals
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud (new pcl::PointCloud<pcl::Normal> ());

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

  // TODO add color to the normals of points

  /*

  int level = 1;
  double scale = 0.025;
  // Add normals of points in the same viewer
  viewer.addPointCloudNormals (*filtered_cloud, *normals_cloud, level, scale, "NORMALS");
  // And wait until Q key is pressed
  viewer.spin ();

  // Color the normals in blue
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 0.0, 0.0, 1.0, "FILTERED");
  // And wait until Q key is pressed
  viewer.spin ();

  */



  // -------------------------------------------------------------- //
  // ------------------ Start fitting 2D circles ------------------ //
  // -------------------------------------------------------------- //

  bool stop = false;

  int circle_fit = 0;

  // Space of parameters for fitted circle models
  pcl::PointCloud<pcl::PointXYZ>::Ptr circle_parameters_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

  do
  {

    // Inliers of circle model
    pcl::PointIndices::Ptr circle_inliers (new pcl::PointIndices ());
    // Coefficients of cirlce model
    pcl::ModelCoefficients circle_coefficients;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optimize coefficients
    seg.setOptimizeCoefficients (false);
    // Set type of method
    seg.setMethodType (pcl::SAC_RANSAC);
    // Set type of model
    seg.setModelType (pcl::SACMODEL_CIRCLE2D);
    // Set number of maximum iterations
    seg.setMaxIterations (maximum_circle_iterations);
    // Set threshold of model
    seg.setDistanceThreshold (circle_threshold);
    // Set minimum and maximum radii
    seg.setRadiusLimits (minimum_radius, maximum_radius);
    // Give as input the filtered point cloud
    seg.setInputCloud (filtered_cloud);
    // Call the segmenting method
    seg.segment (*circle_inliers, circle_coefficients);

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
      // That is why it should stop !
      stop = true;
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

    // ---------------------------- //
    // Start the extraction process //
    // ---------------------------- //

    // Point cloud of circle inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr circle_inliers_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    // Extract the circular inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> ei;
    // Set point cloud from where to extract
    ei.setInputCloud (filtered_cloud);
    // Set which indices to extract
    ei.setIndices (circle_inliers);

    // Return the points which represent the inliers
    ei.setNegative (false);
    // Call the extraction function
    ei.filter (*circle_inliers_cloud);

    // Return the remaining points of inliers
    ei.setNegative (true);
    // Call the extraction function
    ei.filter (*filtered_cloud);

    //ROS_INFO ("Circle has %d inliers", circle_inliers_cloud->points.size());
    //ROS_INFO ("%d points remain after extraction", filtered_cloud->points.size ());

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
    viewer.addCircle (circle_coefficients, circle_id.str ());

    // Add the point cloud data
    viewer.addPointCloud (*circle_inliers_cloud, circle_inliers_id.str ());

    // Set the size of points for cloud
    viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size, circle_inliers_id.str ()); 

    // Wait or not wait
    if ( circle_step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
    }

    // ---------------------------- //
    // Start the clustering process //
    // ---------------------------- //

    // Vector of clusters from inliers
    std::vector<pcl::PointIndices> clusters;
    // Build kd-tree structure for clusters
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr clusters_tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

    // Instantiate cluster extraction object
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> clu;
    // Set as input the cloud of circle inliers
    clu.setInputCloud (circle_inliers_cloud);
    // Radius of the connnectivity threshold
    clu.setClusterTolerance (circle_inliers_clustering_tolerance);
    // Provide pointer to the search method
    clu.setSearchMethod (clusters_tree);
    // Call the extraction function
    clu.extract (clusters);

    //ROS_WARN (" has %d clusters where", clusters.size() );
    //for (int c = 0; c < (int) clusters.size(); c++)
      //ROS_WARN ("       cluster %d has %d points", c, clusters.at(c).indices.size() );

    // Wait or not wait
    if ( circle_step )
    {
      // And wait until Q key is pressed
      viewer.spin ();
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

  } while ((int) filtered_cloud->points.size () > minimum_circle_inliers && stop == false);

  // And wait until Q key is pressed
  viewer.spin ();

  // Save these points to disk
  pcl::io::savePCDFile ("data/circle-parameters-cloud.pcd", *circle_parameters_cloud);

  // Add the point cloud data
  viewer.addPointCloud (*circle_parameters_cloud, "CIRCLE_PARAMETER");

  // Set the size of points for cloud
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_POINT_SIZE, point_size * 2, "CIRCLE_PARAMETER"); 

  // Save these points to disk
  pcl::io::savePCDFile ("data/circle-rest-cloud.pcd", *filtered_cloud);

  // Done with 2D circle models
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Done with 2D circle models in %5.3g [s]", tt.toc ());

  // TODO Obviously, the 2D-line-models code

  // Done with 2D line models
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Done with 2D line models in %5.3g [s]", tt.toc ());

  // TODO Computing the best votes and segmenting the point cloud data

  // Displaying the overall time
  ROS_WARN ("-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------");
  ROS_WARN ("Finished in %5.3g [s]", tt.toc ());

   // And wait until Q key is pressed
  viewer.spin ();

  return (0);
}

