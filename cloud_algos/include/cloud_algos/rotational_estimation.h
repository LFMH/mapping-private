#ifndef CLOUD_ALGOS_ROTATIONAL_ESTIMATION_H
#define CLOUD_ALGOS_ROTATIONAL_ESTIMATION_H
#include <cloud_algos/cloud_algos.h>

class RotationalEstimation : public CloudAlgo
{
 public:
  RotationalEstimation () {};
  typedef sensor_msgs::PointCloud OutputType;

  std::string default_topic_name () 
    {return std::string ("cloud_normals");};

  std::string default_node_name () 
    {return std::string ("rotational_estimation_node");};

  void init (ros::NodeHandle&);
  std::vector<std::string> pre  ();
  std::vector<std::string> post ();
  std::string process (const sensor_msgs::PointCloudConstPtr&);
  OutputType output ();
 
 private: 
  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::Publisher pmap_pub_;
};

#endif

