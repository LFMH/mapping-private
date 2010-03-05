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

//TriangularMesh to output resultant triangles
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <ias_table_msgs/TriangularMesh.h>
//boost
#include <boost/thread/mutex.hpp>

namespace cloud_algos
{

class DepthImageTriangulation : public CloudAlgo
{
 public:
  //! \brief triangle points
  struct triangle 
  {  
    int a,b,c;
  };

  // Input/Output type
  typedef sensor_msgs::PointCloud InputType;
  //typedef mapping_msgs::PolygonalMap OutputType;
  typedef ias_table_msgs::TriangularMesh OutputType;
  // Topic name to advertise
  static std::string default_output_topic ()
  {
    return std::string ("/cloud_triangulated");
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
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();

  /**
   * \brief  get scan and point id for hokuyo scans
   * \param sensor_msg::PointCloud
   */
  void get_scan_and_point_id (sensor_msgs::PointCloud &cloud, signed int &line_index);

  /**
   * \brief  computes distance between 2 points
   * \param cloud_in
   * \param int a
   * \param int b
   */
  float dist_3d (const sensor_msgs::PointCloud &cloud_in, int a, int b);


  /**
   * \brief  writes triangulation result to a VTK file for visualization purposes
   * \param output[] output vtk file
   * \param triangles vector of triangles
   * \param &cloud_in
   * \param nr_tr number of triangles
   */
  void write_vtk_file(std::string output, std::vector<triangle> triangles,  const sensor_msgs::PointCloud &cloud_in,
                      int nr_tr);

  // Constructor-Destructor
  DepthImageTriangulation () : CloudAlgo ()
  {
    max_length = 0.05;
    max_index_ = max_line_ = 0;
    write_to_vtk_ = false;
    save_pcd_ = false;
    line_nr_in_channel_ = index_nr_in_channel_ = -1;
  }
  ~DepthImageTriangulation ()
  {
    
  }

private:
  //! \brief lock the function when restoring line id
  boost::mutex  cloud_lock_;
  
  // ROS stuff
  ros::NodeHandle nh_;

  //! \brief working point cloud
  sensor_msgs::PointCloud cloud_with_line_;
  
  //! \brief max index and max line in point cloud
  int max_index_, max_line_;
  
  //! \brief channel indices for line and index in point cloud msg
  signed int line_nr_in_channel_, index_nr_in_channel_;

  //! \brief max allowed length between triangle's line segments
  float max_length;

  //! \brief write output to vtk yes/no, save PCD file yes/no
  bool write_to_vtk_, save_pcd_;

  // \brief resultant output triangulated map
  //OutputType pmap_;
  OutputType mesh_;
};
}
#endif

