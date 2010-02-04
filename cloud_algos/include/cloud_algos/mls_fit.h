#ifndef CLOUD_ALGOS_PLANAR_ESTIMATION_H
#define CLOUD_ALGOS_PLANAR_ESTIMATION_H
#include <cloud_algos/cloud_algos.h>

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

// Moving Least Squares implementation
class MovingLeastSquares : public CloudAlgo
{
 public:

  // Output type
  typedef sensor_msgs::PointCloud OutputType;

  // Options
  enum GenerationMethod {COPY, FILLING, CLOSE, SURROUNDING, ADAPTIVE, FIXED_K};
  GenerationMethod point_generation_ = COPY;
  double radius_ = 0.02;
  double max_nn_ = 30;

  // Topic name
  std::string default_topic_name () 
    {return std::string ("cloud_mls");}

  // Node name
  std::string default_node_name () 
    {return std::string ("mls_fit_node");}

  // Algorithm methods
  void init (ros::NodeHandle&);
  std::vector<std::string> pre  ();
  std::vector<std::string> post ();
  std::string process (sensor_msgs::PointCloudConstPtr);
  OutputType output ();

  // Setter functions
  void setKdTree (cloud_kdtree::KdTree *kdtree)
    {kdtree_ = kdtree;}
  void setCloud2Fit (sensor_msgs::PointCloud *cloud_fit)
    {cloud_fit_ = cloud_fit;}

  // Clearing previously set data
  void clear ()
  {
    kdtree_ = NULL;
    cloud_fit_ = NULL;
  }

  // Constructor-Destructor
  MovingLeastSquares () : CloudAlgo ()
      {clear ();}
  ~MovingLeastSquares ()
  {
    points_indices_.clear ();
    points_sqr_distances_.clear ();
    if (kdtree_ != NULL)
      delete kdtree_;
    if (cloud_fit_ != NULL)
      delete cloud_fit_;
  }

 private:

  // ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // ROS messages
  sensor_msgs::PointCloud *cloud_fit_;

  // Kd-tree stuff
  cloud_kdtree::KdTree *kdtree_;
  vector<vector<int> > points_indices_;
  vector<vector<float> > points_sqr_distances_;
};

#endif

