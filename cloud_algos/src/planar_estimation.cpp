#include <cloud_algos/cloud_algo.h>
#include <cloud_algos/planar_estimation.h>

void PlanarEstimation::init () 
{
}

std::vector<std::string> PlanarEstimation::pre () 
  {return std::vector<std::string>();}

std::vector<std::string> PlanarEstimation::post ()
  {return std::vector<std::string>();}

std::string PlanarEstimation::process (sensor_msgs::PointCloudConstPtr)
  {return std::string("");}

PlanarEstimation::OutputType PlanarEstimation::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <PlanarEstimation> (argc, argv);
}
#endif


