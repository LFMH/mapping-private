#ifndef CLOUD_ALGOS_NOISE_REMOVAL_H
#define CLOUD_ALGOS_NOISE_REMOVAL_H
#include <cloud_algos/cloud_algos.h>

// Eigen
//#include <Eigen/Array>

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

namespace cloud_algos
{

class StatisticalNoiseRemoval : public CloudAlgo
{
 public:

  // Input/output type
  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  // Options
  double alpha_;          // discard points with average nearest neighbors distance further than alpha_*STD
  int neighborhood_size_; // number of nearest neighbors (including self) to consider
  int min_nr_pts_;        // minimum number of points in the cloud that is still acceptable

  // Topic name to subscribe to
  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  // Topic name to advertise
  static std::string default_output_topic ()
    {return std::string ("cloud_denoise");};

  // Node name
  static std::string default_node_name () 
    {return std::string ("statistical_noise_removal_node");};

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
  StatisticalNoiseRemoval () : CloudAlgo ()
  {
    kdtree_ = NULL;

    // set default values for parameters
    alpha_ = 3;
    neighborhood_size_ = 10;
    min_nr_pts_ = 0;
  }
  ~StatisticalNoiseRemoval ()
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
  //ros::Publisher pub_;

  // ROS messages
  boost::shared_ptr<sensor_msgs::PointCloud> cloud_denoise_;

  // Kd-tree stuff
  cloud_kdtree::KdTree *kdtree_;
  std::vector<std::vector<int> > points_indices_;
  std::vector<std::vector<float> > points_sqr_distances_;
};

}
#endif

