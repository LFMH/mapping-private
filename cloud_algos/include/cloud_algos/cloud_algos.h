#ifndef CLOUD_ALGOS_H
#define CLOUD_ALGOS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>

namespace cloud_algos
{

class CloudAlgo
{
 public:
  CloudAlgo () {};
  
  typedef void OutputType;
  typedef sensor_msgs::PointCloud InputType;
  static std::string default_output_topic () {return std::string("");};
  static std::string default_input_topic () {return std::string("");};
  static std::string default_node_name () {return std::string("");};
  
  virtual void init (ros::NodeHandle&) = 0;

  virtual std::vector<std::string> pre  () = 0;
  virtual std::vector<std::string> post () = 0;
//  virtual std::string process (const boost::shared_ptr<const InputType>&) = ;
  OutputType output ();
};

template <class algo>
  class CloudAlgoNode
{
 public:
  CloudAlgoNode (ros::NodeHandle& nh, algo alg)
  : nh_ (nh), a (alg)
  {
    pub_ = nh_.advertise <typename algo::OutputType> (a.default_output_topic (), 5);
    sub_ = nh_.subscribe (a.default_input_topic (), 1, &CloudAlgoNode<algo>::input_cb, this);
    ROS_INFO("CloudAlgoNode (%s) created. SUB [%s], PUB[%s]",
             a.default_node_name().c_str(),
             a.default_input_topic().c_str(), 
	     a.default_output_topic().c_str());
  }

  void input_cb (const boost::shared_ptr<const typename algo::InputType> &input)
  {
    ROS_INFO("Received message.");
    a.process (input);
    ROS_INFO("Processed message.");
    pub_.publish (a.output());
    ROS_INFO("Published result message.");
  }
  
  ros::NodeHandle& nh_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

  algo& a;
};

template <class algo>
  int standalone_node (int argc, char* argv[])
{
  ros::init (argc, argv, algo::default_node_name());
  algo a;
  ros::NodeHandle nh ("~");
  CloudAlgoNode<algo> c(nh, a);

  ros::spin ();

  return (0);
}

}
#endif

