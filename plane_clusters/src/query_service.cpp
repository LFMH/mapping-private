#include "ros/ros.h"
#include "ros/node.h"
#include <mapping_srvs/GetPlaneClusters.h>
#include <cstdlib>

using namespace mapping_srvs;    
    
int main(int argc, char **argv)
{
  
   ros::init(argc, argv, "query_service");

  
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<GetPlaneClusters>("get_plane_clusters_sr");
    GetPlaneClusters srv;
    if (client.call(srv))
      {
        //add response variables of choice
        ROS_INFO("Coeff a: %f", srv.response.a);
      }
    else
    {
      ROS_ERROR("Failed to call service get_plane_clusters_sr");
      return 1;
    }
  
    return 0;
}
