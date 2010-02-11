#ifndef CLOUD_ALGOS_REGION_GROWING_H
#define CLOUD_ALGOS_REGION_GROWING_H
#include <point_cloud_mapping/kdtree/kdtree_ann.h>
#include <cloud_algos/cloud_algos.h>

namespace cloud_algos
{

/** This class implements Region Growing. It segments a point cloud */
class RegionGrowing : public CloudAlgo
{
 public:
  /** this struct allows for different stopping criteria (where region growing 
      should stop) */
  struct StopAt {
    /** stop method to decide over a given point. This method should be 
        overriden to implement new stopping criteria */
    bool stop (const sensor_msgs::PointCloudConstPtr &cloud, int i, int seed)
    {
      return false;
    }
  };
  /** region growing should stop at points with dimension / channel values 
      higher than a user defined threshold */
  struct StopAtHighDimension : StopAt {
    /** set threshold value (maximum accepted value) */
    void setThreshold (double thresh) {max_val_ = thresh;}
    /** set threshold dimension (maximum accepted value) */
    void setChannelDimension (int d) {chIdx_ = d;}
    /** overriden stop method to decide over a given point */
    bool stop (const sensor_msgs::PointCloudConstPtr &cloud, int i, int seed)
    {
      return cloud->channels.at (chIdx_).values.at (i) > max_val_;
    }
    double max_val_;
    int chIdx_;
  };
  /** region growing should stop at points which differ from a given seed point in a 
      given dimension / channel value more than a user defined threshold */
  struct StopAtDimensionDifference : StopAt {
    /** set threshold value (maximum accepted value) */
    void setThreshold (double thresh) {max_val_ = thresh;}
    /** set threshold dimension (maximum accepted value) */
    void setChannelDimension (int d) {chIdx_ = d;}
    /** overriden stop method to decide over a given point */
    bool stop (const sensor_msgs::PointCloudConstPtr &cloud, int i, int seed)
    {
      return fabs (cloud->channels.at (chIdx_).values.at (i) - cloud->channels.at (chIdx_).values.at (i)) > max_val_;
    }
    double max_val_;
    int chIdx_;
  };

  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  static std::string default_output_topic ()
    {return std::string ("cloud_pcd");};

  static std::string default_node_name () 
    {return std::string ("region_growing_node");};

  void init (ros::NodeHandle&);
  std::vector<std::string> pre  ();
  std::vector<std::string> post ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();
  
  void GrowFromPoint (const sensor_msgs::PointCloudConstPtr&, int);
  void OptimizedRegionGrowing (const sensor_msgs::PointCloudConstPtr&);
  void OptimizedRegionGrowing (const sensor_msgs::PointCloudConstPtr&, std::vector<int>);

  void setStopAtCriterion (StopAt);
  void setRadius (double radius);
  void setExtraDimensionIgnore (int dimIdx, double dimVal);
  void setMaxNrNN (int max_nr_nn);
  
  double radius_;
  int dimIdx_;
  double dimVal_;
  int max_nr_nn_;

  cloud_kdtree::KdTree* kdtree_;
  std::vector<sensor_msgs::PointCloud> clusters_;
  std::vector<int> regions_; // ?? 
  bool create_pcds_;
  int regInitial_;
  int regId_;
  ros::NodeHandle nh_;
  StopAt stop_at_;
};

}
#endif

