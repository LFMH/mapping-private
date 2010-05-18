#ifndef CLOUD_ALGOS_NORMAL_ESTIMATION_H
#define CLOUD_ALGOS_NORMAL_ESTIMATION_H
#include <cloud_algos/cloud_algos.h>

namespace cloud_algos
{

class NormalEstimation : public CloudAlgo
{
 public:
  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  static std::string default_output_topic ()
    {return std::string ("cloud_normals");};

  static std::string default_node_name () 
    {return std::string ("normal_estimation_node");};

  void init ();
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();
  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }
};

}
#endif

