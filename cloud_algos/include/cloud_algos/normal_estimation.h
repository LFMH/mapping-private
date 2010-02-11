#ifndef CLOUD_ALGOS_NORMAL_ESTIMATION_H
#define CLOUD_ALGOS_NORMAL_ESTIMATION_H
#include <cloud_algos/cloud_algos.h>

class NormalEstimation : public CloudAlgo
{
 public:
  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  std::string default_input_topic ()
    {return std::string ("cloud_pcd");}


  std::string default_output_topic ()
    {return std::string ("cloud_normals");};

  std::string default_node_name () 
    {return std::string ("normal_estimation_node");};

  void init ();
  std::vector<std::string> pre  ();
  std::vector<std::string> post ();
  std::string process (const sensor_msgs::PointCloudConstPtr);
  OutputType output ();
};

#endif
