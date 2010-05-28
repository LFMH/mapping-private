/*********************************************************************
*
*  Copyright (c) 2009, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include <ros/ros.h>

#include <tabletop_object_detector/TabletopDetection.h>
#include <grasp_execution/mechanism_interface.h>
#include <object_grasping/collision_map_interface.h>

#include "grasping_app_executive/GraspingAppGraspCloud.h"
#include "grasping_app_executive/GraspingAppPlace.h"

static const std::string DETECTION_SERVICE_NAME="/object_detection";
static const std::string GRASPING_CLOUD_SERVICE_NAME = "grasping_app_grasp_cloud";
static const std::string PLACING_SERVICE_NAME = "grasping_app_place";

int main(int argc, char** argv)
{
  //initialize node handle
  ros::init(argc, argv, "cloud_grasp_node");
  ros::NodeHandle nh;
  std::string input_cloud_topic_;
  nh.param ("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
  cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, cloud_cb, this);
//   //wait for and initialize services
//   ros::ServiceClient detection_client = nh.serviceClient<tabletop_object_detector::TabletopDetection>(DETECTION_SERVICE_NAME);
//   while ( !detection_client.waitForExistence(ros::Duration(3.0)) && nh.ok() )
//   {
//     ROS_INFO("Waiting for detection service: %s", DETECTION_SERVICE_NAME.c_str());
//   }  
//   while ( !ros::service::waitForService(GRASPING_CLOUD_SERVICE_NAME, ros::Duration(2.0)) && nh.ok() ) 
//   {
//     ROS_INFO("Waiting for grasping model service to come up");
//   }
//   ros::ServiceClient grasping_cloud_srv = nh.serviceClient<grasping_app_executive::GraspingAppGraspCloud>
//     (GRASPING_CLOUD_SERVICE_NAME, true);  
//   while ( !ros::service::waitForService(PLACING_SERVICE_NAME, ros::Duration(2.0)) && nh.ok() ) 
//   {
//     ROS_INFO("Waiting for placing service to come up");
//   }
//   ros::ServiceClient placing_srv = nh.serviceClient<grasping_app_executive::GraspingAppPlace>(PLACING_SERVICE_NAME, true);

//   //call tabletop detection to get clusters
//   tabletop_object_detector::TabletopDetection detect;
//   detect.request.return_clusters = true;
//   detect.request.return_models = false;
//   if (!detection_client.call(detect))
//   {
//     ROS_ERROR("Failed to call tabletop detection");
//     exit(0);
//   }

  void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
  {
    ROS_INFO("Cloud received in frame: %s", detect.response.detection.clusters[0].header.frame_id.c_str());
    
    grasping_app_executive::ArmSelection arm;
    arm.which_arm = grasping_app_executive::ArmSelection::RIGHT_ARM;
    
    //call grasping service on first cluster
    grasping_app_executive::GraspingAppGraspCloud grasp;
    //grasp.request.cloud = detect.response.detection.clusters[0];
    grasp.request.cloud = cloud;
    grasp.request.which_arm = arm;
    if (!grasping_cloud_srv.call(grasp))
    {
      ROS_ERROR("Grasping cloud service call failed altogether");
      exit(0);
    }

    //process result
    if (grasp.response.grasp_result.value != grasping_app_executive::ResultCode::SUCCESS)
    {
      ROS_ERROR("Grasp failed with error code %d", grasp.response.grasp_result.value);
      exit(0);
    }
    ROS_INFO("Grasp successfully executed");

    grasp_execution::WhichArm which_arm = grasp_execution::RIGHT_ARM;

    //open-loop, place object and get out of there
    object_grasping::CollisionMapInterface collision_map_interface(NULL);
    collision_map_interface.detachAllObjectsFromGripper(which_arm);   
    grasp_execution::MechanismInterface mech_interface;
    motion_planning_msgs::OrderedCollisionOperations ord;
    std::vector<motion_planning_msgs::LinkPadding> link_padding;
    unsigned int actual_steps;
    mech_interface.translateGripper(which_arm, btVector3(0.0, 0.0, -0.1), 
                                    true, ord, link_padding, 10, 0, actual_steps);
    mech_interface.moveGripper(which_arm, grasp_execution::MechanismInterface::GRIPPER_OPEN);
    mech_interface.translateGripper(which_arm, btVector3(0.0, 0.0, 0.1), 
                                    true, ord, link_padding, 10, 0, actual_steps);
    mech_interface.moveArmToSide(which_arm);
  }
}
