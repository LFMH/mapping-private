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
  struct Behaviour {
    /** grow_from_point method to decide over a seed given point. This method should be 
        overriden to implement new region growing behaviours */
    bool grow_from_point (const sensor_msgs::PointCloudConstPtr &cloud, int seed, cloud_kdtree::KdTree* kdtree)
    {
      return true;
    }
    /** grow_into_point method to decide over a given point. This method should be 
        overriden to implement new stopping criteria */
    bool grow_into_point (const sensor_msgs::PointCloudConstPtr &cloud, int i, int seed)
    {
      return true;
    }
  };
  /** region growing should stop at points with dimension / channel values 
      higher than a user defined threshold */
  struct StopAtHighDimension : Behaviour {
    /** set threshold value (maximum accepted value) */
    void setThreshold (double thresh) {max_val_ = thresh;}
    /** set threshold dimension (maximum accepted value) */
    void setChannelDimension (int d) {chIdx_ = d;}
    /** overriden grow_from_point method to decide over a given seed point */
    bool grow_from_point (const sensor_msgs::PointCloudConstPtr &cloud, int seed, cloud_kdtree::KdTree* kdtree)
    {
      return true;
    }
    /** overriden grow_into_point method to decide over a given point */
    bool grow_into_point (const sensor_msgs::PointCloudConstPtr &cloud, int i, int seed)
    {
      return cloud->channels.at (chIdx_).values.at (i) <= max_val_;
    }
    double max_val_;
    int chIdx_;
  };
  /** region growing should stop at points which differ from a given seed point in a 
      given dimension / channel value more than a user defined threshold */
  struct StopAtDimensionDifference : Behaviour {
    /** set threshold value (maximum accepted value) */
    void setThreshold (double thresh) {max_val_ = thresh;}
    /** set threshold dimension (maximum accepted value) */
    void setChannelDimension (int d) {chIdx_ = d;}
    /** overriden grow_from_point method to decide over a given seed point */
    bool grow_from_point (const sensor_msgs::PointCloudConstPtr &cloud, int seed, cloud_kdtree::KdTree* kdtree)
    {
      return true;
    }
    /** overriden grow_into_point method to decide over a given point */
    bool grow_into_point (const sensor_msgs::PointCloudConstPtr &cloud, int i, int seed)
    {
      return fabs (cloud->channels.at (chIdx_).values.at (i) - cloud->channels.at (chIdx_).values.at (i)) <= max_val_;
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
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();
  
  void GrowFromPoint (const sensor_msgs::PointCloudConstPtr&, int);
  void OptimizedRegionGrowing (const sensor_msgs::PointCloudConstPtr&);
  void OptimizedRegionGrowing (const sensor_msgs::PointCloudConstPtr&, std::vector<int>);

  void setBehaviour (Behaviour);
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
  Behaviour behaviour_;
};

}
#endif

