#ifndef CLOUD_ALGOS_DEPTH_IMAGE_TRIANGULATION_H
#define CLOUD_ALGOS_DEPTH_IMAGE_TRIANGULATION_H
#include <cloud_algos/cloud_algos.h>

// For extra Eigen functions
#include <Eigen/Array>
// if needed:
#include <Eigen/LU> // matrix inversion
#include <Eigen/Geometry> // cross product

// Kd Tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/statistics.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>

//PolygonalMap to output triangles
#include <mapping_msgs/PolygonalMap.h>

//boost
#include <boost/thread/mutex.hpp>

namespace cloud_algos
{

class DepthImageTriangulation : public CloudAlgo
{
 public:
 

  // Input/Output type
  typedef sensor_msgs::PointCloud InputType;
  typedef mapping_msgs::PolygonalMap OutputType;

  // Topic name to advertise
  static std::string default_output_topic ()
  {
    return std::string ("cloud_triangulated");
  }
  
  // Topic name to subscribe to
  static std::string default_input_topic ()
    {
      return std::string ("/cloud_pcd");
    }

  // Node name
  static std::string default_node_name ()
    {
      return std::string ("depth_image_triangulation_node");
    }

  // Algorithm methods
  void init (ros::NodeHandle&);
  std::vector<std::string> pre  ();
  std::vector<std::string> post ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();
  /*****************************************************************************
   * \brief  get scan and point id for hokuyo scans
   * \param sensor_msg::PointCloud
   ******************************************************************************/
  void get_scan_and_point_id (const boost::shared_ptr<const InputType>&);


  // Constructor-Destructor
  DepthImageTriangulation () : CloudAlgo ()
  {

  }
  ~DepthImageTriangulation ()
  {
    
  }

private:
  //lock the function when restoring line id
  //NB: pointer because mutex is not copyable
  boost::mutex  cloud_lock_;
  // ROS stuff
  ros::NodeHandle nh_;

    // ROS messages
  sensor_msgs::PointCloud cloud_with_line_;
};

}
#endif

