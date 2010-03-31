#ifndef CLOUD_ALGOS_PIPELINE_LOADER_H
#define CLOUD_ALGOS_PIPELINE_LOADER_H
#include <cloud_algos/cloud_algos.h>
#include <std_msgs/Empty.h>

namespace cloud_algos
{

class PipelineLoader : public CloudAlgo
{
 public:
  PipelineLoader () { };
  typedef std_msgs::Empty OutputType;
  typedef sensor_msgs::PointCloud InputType;

  static std::string default_input_topic ()
    {return std::string ("clusters");}

  static std::string default_output_topic ()
    {return std::string ("semantic_map");};

  static std::string default_node_name () 
    {return std::string ("pipeline_loader_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>);
  boost::shared_ptr<const OutputType> output ();
 
 private: 
  boost::shared_ptr<OutputType> output_;

  ros::NodeHandle nh_;

 protected:
  std::string pipeline_description_; 
};

}
#endif



