#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/normal_estimation.h>

using namespace cloud_algos;

void NormalEstimation::init () 
{
}

std::vector<std::string> NormalEstimation::pre () 
  {return std::vector<std::string>();}

std::vector<std::string> NormalEstimation::post ()
  {return std::vector<std::string>();}

std::string NormalEstimation::process (const boost::shared_ptr<const NormalEstimation::InputType>&)
  {return std::string("");}

NormalEstimation::OutputType NormalEstimation::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <NormalEstimation> (argc, argv);
}
#endif
