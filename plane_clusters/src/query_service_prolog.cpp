#include "ros/ros.h"
#include "ros/node.h"
#include <mapping_srvs/GetPlaneClusters.h>
#include <cstdlib>
#include <ctype.h>
#include </usr/lib/swi-prolog/include/SWI-Prolog.h>

using namespace mapping_srvs;    

foreign_t
pl_getPlaneROS(term_t l)    
{
  int argc = 1;
  char **argv = NULL;
  ros::init(argc, argv, "query_service");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<GetPlaneClusters>("get_detect_planes_service");
  GetPlaneClusters srv;
  term_t tmp = PL_new_term_ref();
  if (client.call(srv))
    {
      //add response variables of choice
      ROS_INFO("Coeff a: %f", srv.response.a);
      if (!PL_unify_list(l, tmp, l) ||
          !PL_unify_float(tmp, srv.response.a))
        PL_fail;
      ROS_INFO("Coeff b: %f", srv.response.b);
      if (!PL_unify_list(l, tmp, l) ||
          !PL_unify_float(tmp, srv.response.b))
        PL_fail;
      ROS_INFO("Coeff c: %f", srv.response.c);
      if (!PL_unify_list(l, tmp, l) ||
          !PL_unify_float(tmp, srv.response.c))
        PL_fail;
      ROS_INFO("Coeff d: %f", srv.response.d);
      if (!PL_unify_list(l, tmp, l) ||
          !PL_unify_float(tmp, srv.response.d))
        PL_fail;
      /*
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
      return PL_unify_nil(l);
    }
  //  else
  //  {
  //  ROS_ERROR("Failed to call service get_detect_planes_service");
  //  return -1;
  //}
  // return 0;
}

/////////////////////////////////////////////////////////////////////////////
// register foreign functions
////////////////////////////////////////////////////////////////////////////
install_t
install()
{ 
  PL_register_foreign("getPlaneROS", 1, pl_getPlaneROS, 0);
}
