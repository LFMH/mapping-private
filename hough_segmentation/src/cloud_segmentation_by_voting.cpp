
#include <string>
#include <vector>

// ros messages dependencies 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// pcl dependencies 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//#include "pcl/PointIndices.h"
//#include "pcl/ModelCoefficients.h"
//#include "pcl/segmentation/sac_segmentation.h"



#include <pcl/features/normal_3d.h>


#include "pcl/sample_consensus/method_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include <pcl/sample_consensus/sac_model_circle.h>
#include "pcl/sample_consensus/model_types.h"

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>

// pcl visualization dependencies 
#include <pcl_visualization/pcl_visualizer.h>

// terminal tools dependecies 
#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>



// Method's Constants 
const double minimum_radius = 0.025; /// [m]
const double maximum_radius = 0.100; /// [m] 

const int minimum_inliers_line = 10; /// [points]
const int minimum_inliers_circle = 50; /// [points]

// Method's Parameters 
double line_thresh = 0.010; /// [m]
double circle_thresh = 0.010; /// [m]
double voting_thresh = 0.25; /// [%]



// Method's Main 
int main (int argc, char** argv)
{

  // Argument check and info about 
  if (argc < 2)
  {
    std::cout << std::endl;
    ROS_INFO ("Syntax is: %s <input>.pcd <options>", argv[0]);
    ROS_INFO ("where <options> are: -line_thresh X               = threshold for line inlier selection");
    ROS_INFO ("                     -circle_thresh X             = threshold for circle inlier selection");
    ROS_INFO ("                     -voting_thresh X             = threshold for Hough-based model voting");
    std::cout << std::endl;
    return (-1);
  } 

  // Parsing the arguments of method 
  terminal_tools::parse_argument (argc, argv, "-line_thresh", line_thresh);
  terminal_tools::parse_argument (argc, argv, "-circle_thresh", circle_thresh);
  terminal_tools::parse_argument (argc, argv, "-voting_thresh", voting_thresh);

  // Take only the first .pcd file into account
  std::vector<int> pFileIndicesPCD = terminal_tools::parse_file_extension_argument (argc, argv, ".pcd");
  if (pFileIndicesPCD.size () == 0)
  {
    ROS_INFO ("No .PCD file given as input!");
    return (-1);
  }

  // Declare dataset for point cloud 
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Load point cloud data 
  if (pcl::io::loadPCDFile (argv[pFileIndicesPCD[0]], cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv[pFileIndicesPCD[0]]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud.width * cloud.height), argv[pFileIndicesPCD[0]], pcl::getFieldsList (cloud).c_str ());

  // Pointer to dataset for point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (cloud));

 

/*

  //////////////////////////////////////////
  // Just to publish the point cloud data //
  ////////////////////////////////////////// 

  // Convert from sensor_msgs::PointCloud2 to pcl::PointCloud<T>
  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg (cloud, msg);

  ros::init(argc, argv, "node_name");
  // Create a node handler 
  ros::NodeHandle nh = ros::NodeHandle("~");
  // And a topic for the node handler 
  std::string topic = nh.resolveName("point_cloud_data");
  // And the size of queue 
  uint32_t queue_size = 1;

  // Create a templated publisher
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> (topic, queue_size);
  // And just publish the object directly
  pub.publish (msg);

*/



  //// -----------------------------------------
  //// ------ Work with pcl visualization ------
  //// -----------------------------------------

  // Open a 3D viewer 
  pcl_visualization::PCLVisualizer viewer ("3D Viewer");

  // Set the background of viewer 
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  // Add system coordiante to viewer 
  viewer.addCoordinateSystem (1.0f);
  // Add the point cloud data 
  //viewer.addPointCloud (cloud, "Original Point Cloud Input");
  viewer.addPointCloud (cloud, "Input");

  // Set default camera parameters
  viewer.initCameraParameters ();
  // And wait until Q key is pressed 
  viewer.spin ();

  // Set default camera parameters
  viewer.resetCamera ();
  // And wait until Q key is pressed 
  viewer.spin ();

  // Parse the camera settings and update the internal camera
  viewer.getCameraParameters (argc, argv);
  // Update camera parameters and render.
  viewer.updateCamera ();
  // And wait until Q key is pressed 
  viewer.spin ();

  //// TODO
  //// Add color to point clouds 
  //// And to certain points from clouds 
  //// Using pcl visualization 



  //// -------------------------------------
  //// ------ Filter point cloud data ------ 
  //// -------------------------------------

  // Declare filtered point cloud
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // Set which point cloud to filter 
  sor.setInputCloud (cloud_ptr);
  // Set number of points for mean distance estimation 
  sor.setMeanK (25);
  // Set the standard deviation multiplier threshold 
  sor.setStddevMulThresh (1.0);
  // Call the filtering method 
  sor.filter (filtered_cloud);

  // Pointer to filtered point cloud 
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ> (filtered_cloud));

  /*

  // Print number of points before 
  std::cerr << std::endl << " Before Filtering: " << cloud.points.size () << " points " << std::endl;
  // Print number of points after  
  std::cerr << " After Filtering: " << filtered_cloud.points.size () << " points " <<  std::endl;
  // Print number of filtered points 
  std::cerr << " Filtered Points: " << (cloud.points.size () - filtered_cloud.points.size ()) << std::endl << std::endl;

  */

  ROS_INFO ("Statistical Outlier Removal | before: %d points | after: %d points | filtered: %d points", cloud.points.size (),  filtered_cloud.points.size (), cloud.points.size () - filtered_cloud.points.size ());

  // Remove the point cloud data 
  viewer.removePointCloud ("Original Point Cloud Input");
  // And wait until Q key is pressed 
  viewer.spin ();
  // Add the filtered point cloud data in the same viewer 
  viewer.addPointCloud (filtered_cloud, "Filtered Point Cloud Data");
  // And wait until Q key is pressed 
  viewer.spin ();

  //// TODO
  //// Solve segmentation fault by filtering 
  //// http://eigen.tuxfamily.org/dox/UnalignedArrayAssert.html 



  //// -------------------------------------------
  //// ------ Estiamte 3D normals of points ------ 
  //// -------------------------------------------

  pcl::PointCloud<pcl::Normal> normals_cloud;
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr tree (new pcl::KdTreeFLANN<pcl::PointXYZ> ());

  // Create an object and estimate the normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setSearchMethod (tree);
  ne.setInputCloud (filtered_cloud_ptr);
  ne.setKSearch (50);
  ne.compute (normals_cloud);



/*

  //pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> sac;
  //pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> sac (cloud);
  //pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr sac (new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ> (cloud)); 

  typedef SampleConsensusModelCircle2D<PointXYZ>::Ptr SampleConsensusModelCircle2DPtr;

  SampleConsensusModel2DPtr model;

  // Create a shared 2d circle model pointer directly
  SampleConsensusModelCircle2DPtr model (new SampleConsensusModelCircle2D<PointXYZ> (cloud.makeShared ()));

  // Create the RANSAC object
  RandomSampleConsensus<PointXYZ> sac (model, 0.03);

*/

  //// TODO
  //// Can not instantiate object of class 



  //// --------------------------------------
  //// ------ Start fitting 2D circles ------ 
  //// --------------------------------------

  // Declare inliers of circle model 
  pcl::PointIndices inliers;
  // Coefficients of cirlce model 
  pcl::ModelCoefficients coefficients;
  // Create the segmentation object 
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optimize coefficients 
  seg.setOptimizeCoefficients (true);
  // Set type of method 
  seg.setMethodType (pcl::SAC_RANSAC);
  // Set type of model 
  seg.setModelType (pcl::SACMODEL_CIRCLE2D);
  // Set number of maximum iterations 
  seg.setMaxIterations (100);
  // Set threshold of model 
  seg.setDistanceThreshold (0.010);
  // Set minimum and maximum radii 
  seg.setRadiusLimits (minimum_radius, maximum_radius);
  // Give as input the filtered point cloud 
  seg.setInputCloud (filtered_cloud_ptr);
  // Call the segmenting method 
  seg.segment (inliers, coefficients);

  // Pointer to inliers of circle model
  pcl::PointIndices::Ptr inliers_ptr (new pcl::PointIndices (inliers));

  if (inliers.indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a circular model for the given dataset.");
    return (-1);
  }

  std::cerr << std::endl << " Model coefficients: " << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << std::endl;
  std::cerr << " Model inliers: " << inliers.indices.size () << std::endl << std::endl;

  /*

  int level = 1;
  double scale = 0.025;
  viewer.addPointCloudNormals (filtered_cloud, normals_cloud, level, scale, "Normals");
  // And wait until Q key is pressed 
  viewer.spin ();

  */

  // Color the cloud with red
  viewer.setPointCloudRenderingProperties (pcl_visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "Filtered Point Cloud Data"); 
  // And wait until Q key is pressed 
  viewer.spin ();

  // TODO 
  // Does NOT color any points 



  // Add circle model to point cloud 
  viewer.addCircle (coefficients, "circle");
  // And wait until Q key is pressed 
  viewer.spin ();


  // Extract the circular inliers from the input cloud 
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  ROS_INFO (" Filtered Cloud Ptr Size : %d points", filtered_cloud_ptr->points.size ());
  extract.setInputCloud (filtered_cloud_ptr);
  ROS_INFO (" Inliers Ptr Size : %d points", inliers_ptr->indices.size ());
  extract.setIndices (inliers_ptr);
  extract.setNegative (true);
  extract.filter (*filtered_cloud_ptr);
  ROS_INFO (" Extracted Cloud Ptr Size : %d points", filtered_cloud_ptr->points.size ());



  //int number_of_points = cloud.points.size();
  //do
  //{
  // 
  //} while ( )




  //ros::init(argc, argv, "node_name");

  //for (size_t i = 0; i < cloud.points.size (); ++i)
  //std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  //pcl::io::savePCDFile ("output.pcd", cloud_all);
  //pcl::io::savePCDFileASCII ("output_of_test_code.pcd", cloud);

  //ros::spin ();

  return (0);
}

