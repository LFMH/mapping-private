#include <ros/ros.h>
#include <ros/node_handle.h>
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/rotational_estimation.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <ias_sample_consensus/sac_model_rotational.h>

void RegionGrowing::init (ros::NodeHandle &nh)
{
  nh_ = nh;
}

std::vector<std::string> RegionGrowing::pre () 
  {return std::vector<std::string> ();}

std::vector<std::string> RegionGrowing::post ()
  {return std::vector<std::string>();}

void RegionGrowing::bla ()
{
}

std::string RegionGrowing::process (sensor_msgs::PointCloudConstPtr cloud)
{
//  for (std::vector<int>::iterator it = indices.begin (); it != indices.end (); it++)
  return std::string("ok");
}

RegionGrowing::OutputType RegionGrowing::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <RegionGrowing> (argc, argv);
}
#endif

