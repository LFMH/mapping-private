#include "ros/ros.h"
#include "ros/node.h"
#include <mapping_srvs/GetBoxes.h>
#include <cstdlib>

using namespace mapping_srvs;    
    
int main(int argc, char **argv)
{
  
   ros::init(argc, argv, "query_service");

  
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<GetBoxes>("get_detect_boxes_service");
    GetBoxes srv;
    if (client.call(srv))
      {
        //add response variables of choice
        /*
        ROS_INFO("Coeff a: %f", srv.response.a);
        ROS_INFO("Coeff b: %f", srv.response.b);
        ROS_INFO("Coeff c: %f", srv.response.c);
        ROS_INFO("Coeff d: %f", srv.response.d);
        ROS_INFO("pcenter X: %f", srv.response.pcenter.x);
        ROS_INFO("pcenter Y: %f", srv.response.pcenter.y);
        ROS_INFO("pcenter Z: %f", srv.response.pcenter.z);
        for (int i = 0; i < srv.response.oclusters.size(); i++)
          {
            ROS_INFO("OBJECT: %d", i);
            ROS_INFO("ocluster X: %f", srv.response.oclusters[i].center.x);
            ROS_INFO("ocluster Y: %f", srv.response.oclusters[i].center.y);
            ROS_INFO("ocluster Z: %f", srv.response.oclusters[i].center.z);

          }
        */
        for (unsigned int i = 0; i < 4; i++)
          ROS_INFO("plane0 coeff: %f", srv.response.boxes[0].plane0[i]);
         for (unsigned int i = 0; i < 4; i++)
          ROS_INFO("plane1 coeff: %f", srv.response.boxes[0].plane1[i]);
         for (unsigned int i = 0; i < 4; i++)
           ROS_INFO("plane2 coeff: %f", srv.response.boxes[0].plane2[i]);
        ROS_INFO("angle01 %f", srv.response.boxes[0].angle01);
        ROS_INFO("angle12 %f", srv.response.boxes[0].angle12);
        ROS_INFO("angle02 %f", srv.response.boxes[0].angle02);
        //ROS_INFO("Service launched");
      }
    else
    {
      ROS_ERROR("Failed to call service get_detect_boxes_service");
      return 1;
    }
  
    return 0;
}
