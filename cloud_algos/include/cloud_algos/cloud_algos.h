#ifndef CLOUD_ALGOS_H
#define CLOUD_ALGOS_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>

class CloudAlgo
{
 public:
  CloudAlgo () {};
  
  typedef void OutputType;
  virtual std::string default_topic_name () = 0;
  virtual std::string default_node_name () = 0;
  
  virtual void init (ros::NodeHandle&) = 0;

  virtual std::vector<std::string> pre  () = 0;
  virtual std::vector<std::string> post () = 0;
  virtual std::string process (sensor_msgs::PointCloudConstPtr&) = 0;
  OutputType output ();
};

template <class algo>
  class CloudAlgoNode
{
 public:
  CloudAlgoNode (ros::NodeHandle& nh, algo alg)
  : nh_ (nh), a (alg)
  {
    pub_ = nh_.advertise <typename algo::OutputType> (a.default_topic_name (), 5);
  }
  
  void cloud_cb (sensor_msgs::PointCloudConstPtr &cloud)
  {
    a.process (cloud);
    pub_.publish (a.output());
  }
  
  ros::NodeHandle& nh_;
  ros::Publisher pub_;

  algo& a;
};

template <class algo>
  int standalone_node (int argc, char* argv[])
{
  algo a;
  ros::init (argc, argv, a.default_node_name ());
  ros::NodeHandle nh ("~");
  CloudAlgoNode<algo> c(nh, a);

  ros::spin ();

  return (0);
}

#endif
