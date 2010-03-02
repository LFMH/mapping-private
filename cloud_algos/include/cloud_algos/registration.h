#ifndef CLOUD_ALGOS_REGISTRATION_H
#define CLOUD_ALGOS_REGISTRATION_H
#include <geometry_msgs/Transform.h>
#include <sensor_msgs/PointCloud.h>
#include <cloud_algos/cloud_algos.h>
#include <Eigen/Array>
#include <Eigen/SVD> 
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <point_cloud_mapping/cloud_io.h>

namespace cloud_algos
{

/** This class implements Registration based on vertical structures in scans. 
    Initial Alignment from AMCL or similar is assumed. */
class Registration : public CloudAlgo
{
 public:
  typedef geometry_msgs::Transform OutputType;
  typedef sensor_msgs::PointCloud InputType;

  static std::string default_input_topic ()
    {return std::string ("planar_decomposition");}

  static std::string default_output_topic ()
    {return std::string ("localization_error");};

  static std::string default_node_name () 
    {return std::string ("registration_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();
 
  double RigidTransformSVD (const boost::shared_ptr<const sensor_msgs::PointCloud>&, std::vector<int>&, std::vector<int>&, Eigen::Matrix4d&);
  double oneIteration (const boost::shared_ptr<const sensor_msgs::PointCloud>&, Eigen::Matrix4d&);
  void setTarget (const boost::shared_ptr<const sensor_msgs::PointCloud>&);
  boost::shared_ptr<const sensor_msgs::PointCloud> target_;
 
  geometry_msgs::Transform transform_; 
  cloud_kdtree::KdTree* kdtree_;
  int nxIdx_;
  int nyIdx_;
  int nzIdx_;
  std::vector<int> indices_sorted_;
  ros::NodeHandle nh_;
};

}
#endif


