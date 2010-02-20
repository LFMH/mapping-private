#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/svm_classification.h>

using namespace cloud_algos;

void SVMClassification::init (ros::NodeHandle&)
{
}

std::vector<std::string> SVMClassification::pre ()
  {return std::vector<std::string>();}

std::vector<std::string> SVMClassification::post ()
  {return std::vector<std::string>();}

std::string SVMClassification::process (const boost::shared_ptr<const SVMClassification::InputType>&)
  {return std::string("");}

SVMClassification::OutputType SVMClassification::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <SVMClassification> (argc, argv);
}
#endif

