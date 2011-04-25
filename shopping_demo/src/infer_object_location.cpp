/*
 * Copyright (c) 2010, Thomas Ruehr <ruehr@cs.tum.edu>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <ias_drawer_executive/OpenContainerAction.h>
#include <ias_drawer_executive/CloseContainerAction.h>
#include <ias_drawer_executive/OperateHandleController.h>
#include <ias_drawer_executive/RobotDriver.h>
#include <ias_drawer_executive/RobotArm.h>
#include <ias_drawer_executive/Torso.h>
#include <ias_drawer_executive/Gripper.h>
#include <ias_drawer_executive/Perception3d.h>

//for GetClustersAction
#include <pcl_cloud_tools/GetClustersAction.h>
#include <pr2_gripper_sensor_msgs/PR2GripperGrabAction.h>

#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>

//for pointing the head
#include <pr2_controllers_msgs/PointHeadAction.h>

//for getting percepts from odu_finder
#include <vision_msgs/cop_answer.h>

//for copAnswerCB
#include <boost/bind.hpp> 

//for moving the Torso
#include <pr2_controllers_msgs/SingleJointPositionAction.h>

//for best object location
#include <shopping_demo/QueryBestObjLocation.h>

void copAnswerCB(std::string *object_type, const vision_msgs::cop_answer::ConstPtr& msg)
{
  // vision_msgs::cop_answer msg;
  // vision_msgs::cop_descriptor cop_descriptor;
  // vision_msgs::aposteriori_position aposteriori_position;
  // msg.found_poses.push_back(aposteriori_position);
  // msg.found_poses[0].models.push_back(cop_descriptor);
  // //TODO: only one needed probably, ask Moritz
  //msg.found_poses[0].models[0].type = "ODUFinder";
  *object_type = msg->found_poses[0].models[0].sem_class;
  // msg.found_poses[0].models[0].object_id = ++object_id;
  // msg.found_poses[0].position = 0;
  // template_publisher.publish(msg);
  return;
}

std::string getObjectType(ros::NodeHandle &nh)
{
  std::string object_type;
  ros::Subscriber sub_cop = nh.subscribe<vision_msgs::cop_answer>("/odu_finder/TemplateName", 10, boost::bind(&copAnswerCB, &object_type, _1)); 
  return object_type;
}

void computeMarkerAndPublish (ros::Publisher &vis_pub, geometry_msgs::PointStamped point, int id, double color[])
{
  //advertise as latched
  //  ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "/segment_objects_interactive/vis_marker", 1, true);
  visualization_msgs::Marker marker;
  marker.header.frame_id = point.header.frame_id;
  marker.header.stamp = ros::Time::now();
  marker.ns = "";
  marker.id = id;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = point.point.x;
  marker.pose.position.y = point.point.y;
  marker.pose.position.z = point.point.z;
  marker.pose.orientation.x = 0;
  marker.pose.orientation.y = 0;
  marker.pose.orientation.z = 0;
  marker.pose.orientation.w = 1;
  marker.scale.x = 0.03;
  marker.scale.y = 0.03;
  marker.scale.z = 0.03;
  marker.color.a = 1.0;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.lifetime = ros::Duration::Duration();
  ROS_INFO_STREAM("CUBE MARKER COMPUTED, WITH FRAME " << marker.header.frame_id);
  vis_pub.publish( marker );
  ros::spinOnce();
  return;
}

void moveTorso(double position, double velocity, std::string direction)
{
  actionlib::SimpleActionClient<pr2_controllers_msgs::SingleJointPositionAction> tc ("torso_controller/position_joint_action", true);
  
  //wait for the action server to come up
  while(!tc.waitForServer(ros::Duration(5.0)))
    {
      ROS_INFO("Waiting for the torso action server to come up");
    }
  pr2_controllers_msgs::SingleJointPositionGoal goal;
  goal.position = position;  //all the way up is 0.3
  goal.min_duration = ros::Duration(2.0);
  goal.max_velocity = velocity;
  
  ROS_INFO("Sending '%s' goal", direction.c_str());
  tc.sendGoal(goal);
  tc.waitForResult();
  
  if(tc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
      ROS_INFO("Torso moved %s.", direction.c_str());
    }
  else
    {
      ROS_ERROR("Error in moving torso %s.", direction.c_str());
    }
  return;
}

void pointHead(std::string pointing_frame, std::string frame_id, double x, double y, double z, double duration, double velocity)
  {
    typedef actionlib::SimpleActionClient<pr2_controllers_msgs::PointHeadAction> PointHeadClient;
    
    PointHeadClient point_head_client ("/head_traj_controller/point_head_action", true);
    while(!point_head_client.waitForServer())
      {
	ROS_INFO("Waiting for the point_head_action server to come up");
      }
    
    //the goal message we will be sending
    pr2_controllers_msgs::PointHeadGoal goal;

    //the target point, expressed in the requested frame
    geometry_msgs::PointStamped point;
    point.header.frame_id = frame_id;
    point.point.x = x; point.point.y = y; point.point.z = z;
    goal.target = point;

    //we are pointing the wide_stereo camera frame 
    //(pointing_axis defaults to X-axis)
    goal.pointing_frame = pointing_frame;

    //take at least 5 seconds to get there
    goal.min_duration = ros::Duration(duration);

    //and go no faster than 0.1 rad/s
    goal.max_velocity = velocity;

    //send the goal
    point_head_client.sendGoal(goal);

    //wait for it to get there
    point_head_client.waitForResult();
    return;
  }

void closeGripperComp(std::string side, double gain)
{
  typedef actionlib::SimpleActionClient<pr2_gripper_sensor_msgs::PR2GripperGrabAction>    GrabAC;
  GrabAC grab("/"+ side + "_gripper_sensor_controller/grab", true);
  while(!grab.waitForServer())
    {
      ROS_INFO("Waiting for the /%s_gripper_sensor_controller/grab to come up", side.c_str());
    }
  pr2_gripper_sensor_msgs::PR2GripperGrabGoal grab_goal;
  grab_goal.command.hardness_gain=gain;
  ROS_INFO("sending grab goal");
  grab.sendGoal(grab_goal);
  grab.waitForResult(ros::Duration(5.0));
  if (grab.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("/%s_gripper_sensor_controller/grab SUCCEEDED", side.c_str());
  return;
}

boost::shared_ptr<const pcl_cloud_tools::GetClustersResult> getClusters(int ec_goal)
{
  //get the object from Kinect
  typedef actionlib::SimpleActionClient<pcl_cloud_tools::GetClustersAction> GetClustersClient;
  GetClustersClient gcc("/extract_clusters/get_clusters", true);
  while(!gcc.waitForServer())
    {
      ROS_INFO("Waiting for /extract_clusters/get_clusters server to come up");
    }
  
  pcl_cloud_tools::GetClustersGoal goal;
  goal.ec_goal = ec_goal;
  // Fill in goal here
  gcc.sendGoal(goal);
  gcc.waitForResult(ros::Duration(5.0));
  if (gcc.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("/extract_clusters/get_clusters SUCCEEDED");
  boost::shared_ptr<const pcl_cloud_tools::GetClustersResult> result = gcc.getResult();
  return result;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "segment_objects_interactive");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "/segment_objects_interactive/vis_marker", 1, true);
    std::string to_frame, pointing_frame;
    double x, y, z, duration, velocity;
    double robot_pose [4] = {-3.15, -1.1, -0.707, 0.707};

    nh.param("to_frame", to_frame, std::string("base_link"));
    nh.param("pointing_frame", pointing_frame, std::string("narrow_stereo_optical_frame"));
    nh.param("x", x, 0.5);
    nh.param("y", y, 0.0);
    nh.param("z", z, 0.7);
    nh.param("duration", duration, 0.1);
    nh.param("velocity", velocity, 0.3);

    tf::TransformListener tf_listener_;
    
    //raise up the Torso
    moveTorso(0.3, 1.0, "up");
    OperateHandleController::plateAttackPose();
    //get the robot to the pose in map
    RobotDriver::getInstance()->moveBaseP(robot_pose[0], robot_pose[1], robot_pose[2], robot_pose[3]);

    Gripper::getInstance(0)->open();

    //point the head to the table
    pointHead(pointing_frame, to_frame, x, y, z, duration, velocity);
    ROS_INFO("[segment_objects_interactive: ] Head pointed to the table.");

    //turn on projected stereo
    int sysret = system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'");
    ROS_INFO("[segment_objects_interactive: ] %i rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 3, narrow_stereo_trig_mode: 3}'", sysret);

    //get the cluster on the table
    boost::shared_ptr<const pcl_cloud_tools::GetClustersResult> result = getClusters(0);

    sysret=system("rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node projector_mode 1");
    ROS_INFO("[segment_objects_interactive: ] %i rosrun dynamic_reconfigure dynparam set  /camera_synchronizer_node '{projector_mode: 1}'", sysret);

    if (result->clusters.size () == 0)
      {
	ROS_ERROR("No clusters found, returning");
	return -1;
      }
	
    geometry_msgs::PointStamped stamped_in, center, point_min, point_max;
    for (size_t j=0; j < result->clusters.size(); ++j)
      {
	ROS_INFO("cluster center %f, %f, %f", result->clusters[j].center.x, result->clusters[j].center.y, result->clusters[j].center.z);
	ros::Time time = ros::Time::now();
	bool found_transform = tf_listener_.waitForTransform("base_link", result->clusters[j].header.frame_id, time, ros::Duration(0.1));
	if (!found_transform)
	  {
	    ROS_WARN("No transform found" );
	    continue;
	  }
	stamped_in.header = result->clusters[j].header;
	stamped_in.point.x = result->clusters[j].center.x;
	stamped_in.point.y = result->clusters[j].center.y;
	stamped_in.point.z = result->clusters[j].center.z;
	tf_listener_.transformPoint("base_link", stamped_in, center);
	double color [3] = {1.0, 0.0, 0.0};
	computeMarkerAndPublish (vis_pub, center, 0, color);
	ROS_INFO("Cluster center in base_link %f, %f, %f", center.point.x, center.point.y, center.point.z);

	stamped_in.point.x = result->clusters[j].min_bound.x;
	stamped_in.point.y = result->clusters[j].min_bound.y;
	stamped_in.point.z = result->clusters[j].min_bound.z;
	tf_listener_.transformPoint("base_link", stamped_in, point_min);
	color = {0.0, 1.0, 0.0};
	computeMarkerAndPublish (vis_pub, point_min, 1, color);

	stamped_in.point.x = result->clusters[j].max_bound.x;
	stamped_in.point.y = result->clusters[j].max_bound.y;
	stamped_in.point.z = result->clusters[j].max_bound.z;
	tf_listener_.transformPoint("base_link", stamped_in, point_max);
	color = {0.0, 0.0, 1.0};
	computeMarkerAndPublish (vis_pub, point_max, 2, color);
      }

    ROS_INFO("Closing to %f", fabs(point_max.point.y - point_min.point.y));
    //grasp the object and calculate 
    double obj_z_grasp_correction = 0.2 * fabs(point_max.point.z - point_min.point.z);
    RobotArm::getInstance(0)->universal_move_toolframe_ik(center.point.x-0.1, center.point.y-0.01, center.point.z, 0.0, 0.0, 0.0, 1.0, "base_link");
    RobotArm::getInstance(0)->universal_move_toolframe_ik(center.point.x, center.point.y-0.01, center.point.z-obj_z_grasp_correction, 0.0, 0.0, 0.0, 1.0, "base_link");

    //close compliant
    closeGripperComp("r", 0.05);
    //moving the object in front of the camera
    RobotArm::getInstance(0)->universal_move_toolframe_ik(center.point.x, center.point.y, center.point.z-obj_z_grasp_correction+0.2, 0.0, 0.0, 0.0, 1.0, "base_link");
    RobotArm::getInstance(0)->universal_move_toolframe_ik(0.537, -0.008, 0.873, 0.301, 0.244, 0.618, 0.684, "base_link");


    //call recognition
    //TODO
    // ros::Rate rate(1);
    // std::string obj = "";
    // while (obj == "")
    // {
    //   rate.sleep();
    //   ros::spinOnce();
    //   obj = getObjectType(nh);
    // }
    // ROS_INFO("Object type: %s", obj.c_str());
    
    //call best_object_location
    ros::ServiceClient client = nh.serviceClient<shopping_demo::QueryBestObjLocation>("/best_object_location/query");
    shopping_demo::QueryBestObjLocation srv;
    
    srv.request.name_space = argc == 3 ? argv[1] : "orgprinciples_demo";
    srv.request.object_type = argc == 3 ? argv[2] : "AlpenMilch_Fettarme_Milch";
    if (client.call(srv))
      {
      for (uint i = 0; i < srv.response.location.size(); i++)
	{
	ROS_INFO("[segment_objects_interactive: ] /best_object_location/query: service reponse %s", srv.response.location[i].c_str ());
	}
      }
    else
      ROS_ERROR("Failed to call [segment_objects_interactive: ] /best_object_location/query: sevice!");

     sleep(5.0);


    //move the object back to the table
    RobotArm::getInstance(0)->universal_move_toolframe_ik(center.point.x, center.point.y-0.01, center.point.z-obj_z_grasp_correction+0.2, 0.0, 0.0, 0.0, 1.0, "base_link");
    RobotArm::getInstance(0)->universal_move_toolframe_ik(center.point.x, center.point.y-0.01, center.point.z-obj_z_grasp_correction, 0.0, 0.0, 0.0, 1.0, "base_link");
    Gripper::getInstance(0)->open();
    RobotArm::getInstance(0)->universal_move_toolframe_ik(center.point.x-0.1, center.point.y-0.01, center.point.z, 0.0, 0.0, 0.0, 1.0, "base_link");
    OperateHandleController::plateAttackPose();
    return 0;
}
