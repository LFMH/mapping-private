//ros
#include <ros/ros.h>

//cloud_algos
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/plugin_loader.h>

using namespace cloud_algos;

void PipelineLoader::init (ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("pipeline_description", pipeline_description_, std::string("pipeline.yaml"));
}

void PipelineLoader::pre  ()
{
}

void PipelineLoader::post ()
{
}

std::vector<std::string> PipelineLoader::requires ()
{
  return std::vector<std::string>();
}

std::vector<std::string> PipelineLoader::provides ()
{
  return std::vector<std::string>();
}

std::string PipelineLoader::process (const boost::shared_ptr<const InputType> input)
{
  //pipeline.process (input);
}

boost::shared_ptr<const PipelineLoader::OutputType> PipelineLoader::output () 
{
  return output_;
};


////////////////////////////////////////////////////////////////////////////////
#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <PipelineLoader> (argc, argv);
}
#endif


