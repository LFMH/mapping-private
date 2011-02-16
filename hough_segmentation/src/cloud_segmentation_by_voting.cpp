
#include <string>
#include <vector>

// ros messages dependencies 
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// pcl dependencies 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

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

//#include "pcl/PointIndices.h"
//#include "pcl/ModelCoefficients.h"
//#include <pcl/segmentation/sac_segmentation.h>



// Method's Parameters 
double line_thresh = 0.0100; // [mm]
double circle_thresh = 0.0100; // [mm]
double voting_thresh = 0.10; // [%]

int minimum_inliers_line = 10; /// [points]
int minimum_inliers_circle = 50; /// [points]



// Method's Main 
int main (int argc, char** argv)
{

  // Argument check and info about 
  if (argc < 2)
  {
    std::cout << std::endl;
    ROS_INFO ("Syntax is: %s <input>.pcd <options>",  argv[0]);
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

  // Load point cloud data 
  pcl::PointCloud<pcl::PointXYZ> cloud;
  if (pcl::io::loadPCDFile (argv[pFileIndicesPCD[0]], cloud) == -1)
  {
    ROS_ERROR ("Couldn't read file %s", argv[pFileIndicesPCD[0]]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud.width * cloud.height), argv[pFileIndicesPCD[0]], pcl::getFieldsList (cloud).c_str ());

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
  //pcl_visualization::PCLVisualizer::setBackgroundColor (const double &r, const double &g, const double &b, int viewport);
  viewer.setBackgroundColor (0.0, 0.0, 0.0);
  // Add system coordiante to viewer 
  viewer.addCoordinateSystem (2.0f);
  // Add the point cloud data 
  viewer.addPointCloud (cloud, "Original Point Cloud Input");
  // And wait until key is pressed 
  viewer.spin ();

  //// TODO
  //// Add color to point clouds 
  //// And to certain points from clouds 
  //// Using pcl visualization 

/*

  // Filter point cloud data 
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  // Set which point cloud to filter 
  sor.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (cloud));
  // Set number of points for mean distance estimation 
  sor.setMeanK (50);
  // Set the standard deviation multiplier threshold 
  sor.setStddevMulThresh (1.0);
  // Call the filtering method 
  sor.filter (filtered_cloud);
   
  // Print number of points before 
  std::cerr << std::endl << " Before filtering: " << cloud.points.size () << " points " << std::endl;
  // Print number of points after  
  std::cerr << " After filtering: " << filtered_cloud.points.size () << " points " <<  std::endl;
  // Print number of filtered points 
  std::cerr << " Filtered points: " << (cloud.points.size () - filtered_cloud.points.size ()) << std::endl << std::endl;

  // Remove the point cloud data 
  viewer.removePointCloud ("Original Point Cloud Input");
  // And wait until key is pressed 
  viewer.spin ();
  // Add the filtered point cloud data in the same viewer 
  viewer.addPointCloud (filtered_cloud, "Filtered Point Cloud Data");
  // And wait until key is pressed 
  viewer.spin ();

*/

  //// TODO
  //// Solve segmentation fault by filtering 
  //// http://eigen.tuxfamily.org/dox/UnalignedArrayAssert.html 

  // Filter point cloud data 
  pcl::PointCloud<pcl::PointXYZ> filtered_cloud;
  // Allegedly filtered 
  filtered_cloud = cloud;

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



  // Inliers of circle model 
  pcl::PointIndices inliers;
  // Coefficients of cirlce model 
  pcl::ModelCoefficients coefficients;
  // Create the segmentation object 
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optimize coefficients 
  seg.setOptimizeCoefficients (true);
  // Set type of method 
  seg.setMethodType (pcl::SAC_RANSAC);
  // Set number of maximum iterations 
  seg.setMaxIterations (10);
  // Set type of model 
  seg.setModelType (pcl::SACMODEL_CIRCLE2D);
  // Set threshold of model 
  seg.setDistanceThreshold (0.010);
  // Give as input the filtered point cloud 
  seg.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (filtered_cloud));
  // Call the segmenting method 
  seg.segment (inliers, coefficients);

  if (inliers.indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
    return (-1);
  }

  // Add circle model to point cloud 
  viewer.addCircle (coefficients, "circle");
  // And wait until key is pressed 
  viewer.spin ();

  std::cerr << "Model coefficients: " << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << std::endl;

  std::cerr << "Model inliers: " << inliers.indices.size () << std::endl;

  //for (size_t i = 0; i < inliers.indices.size (); ++i)
    //std::cerr << inliers.indices[i] << "    " << filtered_cloud.points[inliers.indices[i]].x << " " << filtered_cloud.points[inliers.indices[i]].y << " " << filtered_cloud.points[inliers.indices[i]].z << std::endl;



  std::cerr << std::endl << " Points: " << filtered_cloud.points.size () << std::endl << std::endl;

  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Extract the circular inliers from the input cloud 
  extract.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZ> > (filtered_cloud));
  extract.setIndices (boost::make_shared<pcl::PointIndices> (inliers));
  extract.setNegative (false);
  extract.filter (filtered_cloud);

  std::cerr << std::endl << " Points: " << filtered_cloud.points.size () << std::endl << std::endl;



  //pcl::ModelCoefficients coefficients;

  // 
  //int number_of_points = cloud.points.size();
  //do
  //{
  // 
  // 
  // 
  //} while (   )






  //ros::init(argc, argv, "node_name");

  //for (size_t i = 0; i < cloud.points.size (); ++i)
  //std::cerr << "    " << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;

  //pcl::io::savePCDFile ("output.pcd", cloud_all);
  //pcl::io::savePCDFileASCII ("output_of_test_code.pcd", cloud);

  //ros::spin ();

  return (0);
}

