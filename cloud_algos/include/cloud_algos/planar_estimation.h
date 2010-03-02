#ifndef CLOUD_ALGOS_PLANAR_ESTIMATION_H
#define CLOUD_ALGOS_PLANAR_ESTIMATION_H
#include <cloud_algos/cloud_algos.h>

namespace cloud_algos
{

class PlanarEstimation : public CloudAlgo
{
 public:
  typedef sensor_msgs::PointCloud OutputType;
  typedef geometry_msgs::Point32 InputType;

  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  static std::string default_output_topic ()
    {return std::string ("cloud_normals");};

  static std::string default_node_name () 
    {return std::string ("normal_estimation_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();
};

}
#endif

