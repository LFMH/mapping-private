#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/planar_estimation.h>

using namespace cloud_algos;

void PlanarEstimation::init (ros::NodeHandle&)
{
}

void PlanarEstimation::pre ()
{

}

void PlanarEstimation::pre ()
{

}

std::vector<std::string> PlanarEstimation::requires ()
  {return std::vector<std::string>();}

std::vector<std::string> PlanarEstimation::provides ()
  {return std::vector<std::string>();}

std::string PlanarEstimation::process (const boost::shared_ptr<const PlanarEstimation::InputType>&)
  {return std::string("");}

PlanarEstimation::OutputType PlanarEstimation::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <PlanarEstimation> (argc, argv);
}
#endif


