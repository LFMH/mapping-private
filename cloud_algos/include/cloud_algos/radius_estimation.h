#ifndef CLOUD_ALGOS_RADIUS_H
#define CLOUD_ALGOS_RADIUS_H
#include <cloud_algos/cloud_algos.h>

// Eigen
#include <Eigen/Core>

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

// IO stuff
#include <point_cloud_mapping/cloud_io.h>

namespace cloud_algos
{

class LocalRadiusEstimation : public CloudAlgo
{
 public:

  // Input/output type
  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  // Options
  double radius_;       // search radius for getting the nearest neighbors
  int max_nn_;          // maximum number of nearest neighbors to consider
  double plane_radius_; // radius value to set for planes - it is infinity when computed, but that messes up visualization :)
  int distance_div_;    // number of divisions for distance discretization
  int point_label_ ;    // set the value for the class label OR -1 if estimated surface type should be set
  bool rmin2curvature_; // overwrite curvature values with r_min if enabled

  // Topic name to subscribe to
  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  // Topic name to advertise
  static std::string default_output_topic ()
    {return std::string ("cloud_radius");};

  // Node name
  static std::string default_node_name () 
    {return std::string ("radius_estimation_node");};

  // Algorithm methods
  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  boost::shared_ptr<const OutputType> output ();

  // Clearing previously set data
  void clear ()
  {
    points_indices_.clear ();
    points_sqr_distances_.clear ();
    if (kdtree_ != NULL)
    {
      delete kdtree_;
      kdtree_ = NULL;
    }
  }

  // Constructor-Destructor
  LocalRadiusEstimation () : CloudAlgo ()
  {
    kdtree_ = NULL;

    // set default values for parameters
    radius_ = 0.03;
    max_nn_ = 150;
    plane_radius_ = 0.1;
    distance_div_ = 10;
    point_label_ = -1;
    rmin2curvature_ = false;
  }
  ~LocalRadiusEstimation ()
  {
    clear ();
  }

  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }
 private:

  // ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // ROS messages
  boost::shared_ptr<sensor_msgs::PointCloud> cloud_radius_;

  // Kd-tree stuff
  cloud_kdtree::KdTree *kdtree_;
  std::vector<std::vector<int> > points_indices_;
  std::vector<std::vector<float> > points_sqr_distances_;
};

}
#endif

