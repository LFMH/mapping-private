#include "ros/ros.h"
// #include "roscpp_tutorials/TwoInts.h"
#include <mapping_srvs/GetPlaneClusters.h>
#include <cstdlib>

using namespace mapping_srvs;    
    
int main(int argc, char **argv)
{
  ros::init(argc, argv, "bla");
  /*if (argc != 3)
    {
      ROS_INFO("usage: add_two_ints_client X Y");
      return 1;
    }
  
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<roscpp_tutorials::TwoInts>("/get_plane_cluster_sr");
  roscpp_tutorials::TwoInts srv;
  srv.request.a = atoi(argv[1]);
  srv.request.b = atoi(argv[2]);
  if (client.call(srv))
    {
      ROS_INFO("Sum: %ld", (long int)srv.response.sum);
    }
  else
    {
      ROS_ERROR("Failed to call service add_two_ints");
      return 1;
      }*/

  ros::Node n;
  GetPlaneClusters::Request req;
  GetPlaneClusters::Response resp;
 
  ros::service::call ("/get_plane_cluster_sr", req, resp);
  ROS_INFO("a coefficient: %f", resp.a);
  
  
  return 0;
}
