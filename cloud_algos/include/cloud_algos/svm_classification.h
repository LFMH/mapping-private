#ifndef CLOUD_ALGOS_SVM_CLASSIFICATION_H
#define CLOUD_ALGOS_SVM_CLASSIFICATION_H
#include <cloud_algos/cloud_algos.h>

#include <libsvm/svm.h>

namespace cloud_algos
{

class SVMClassification : public CloudAlgo
{
 public:
  typedef sensor_msgs::PointCloud OutputType;
  typedef geometry_msgs::Point32 InputType;

  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  static std::string default_output_topic ()
    {return std::string ("cloud_gp3");};

  static std::string default_node_name () 
    {return std::string ("normal_gp3_node");};

  void init (ros::NodeHandle&);
  std::vector<std::string> pre  ();
  std::vector<std::string> post ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();
};

}
#endif

