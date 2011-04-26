/*
 * Copyright (c) 2010, Dejan Pangercic <pangercic@cs.tum.edu>
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

//for putting the object in the fridge
#include <ias_drawer_executive/DemoScripts.h>

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

void threadTorso(double position, double velocity, std::string direction, double sleepTime)
{
    sleep(sleepTime);
    moveTorso(position,velocity,direction);
    std::cerr<<"moveTorso thread finished"<<std::endl;
}


int main (int argc, char **argv)
{
    ros::init(argc, argv, "put_object_in_container");
    ros::NodeHandle nh;
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::Marker>( "/segment_objects_interactive/vis_marker", 1, true);
    std::string to_frame, pointing_frame;
    double x, y, z, duration, velocity;
    double robot_pose1 [4] = {-3.094, 0.155,  0.000, 1.000};
    double robot_pose2 [4] = {-1.947, 0.216,  0.000, 1.000};
    double robot_pose3 [4] = {-0.500, 0.100,  0.000, 1.000};
    double robot_pose4 [4] = { 0.000,-0.500,  0.000, 1.000};
    //0 - right, 1 - left
    int side = 1;
    nh.param("to_frame", to_frame, std::string("base_link"));
    nh.param("pointing_frame", pointing_frame, std::string("narrow_stereo_optical_frame"));
    nh.param("x", x, 0.5);
    nh.param("y", y, 0.0);
    nh.param("z", z, 0.7);
    nh.param("duration", duration, 0.1);
    nh.param("velocity", velocity, 0.3);

    tf::TransformListener tf_listener_;

    //raise up the Torso
    //moveTorso(0.01, 1.0, "down");
    boost::thread threadT1(threadTorso, 0.15, 1.0, "down",0);
    //get the robot to the pose in map
    RobotDriver::getInstance()->moveBaseP(robot_pose1[0], robot_pose1[1], robot_pose1[2], robot_pose1[3]);
    RobotDriver::getInstance()->moveBaseP(robot_pose2[0], robot_pose2[1], robot_pose2[2], robot_pose2[3]);
    RobotDriver::getInstance()->moveBaseP(robot_pose3[0], robot_pose3[1], robot_pose3[2], robot_pose3[3]);
    boost::thread threadT2(threadTorso, 0.30, 1.0,   "up",0);
    RobotDriver::getInstance()->moveBaseP(robot_pose4[0], robot_pose4[1], robot_pose4[2], robot_pose4[3]);
    //moveTorso(0.3, 1.0, "up");
    //DemoScripts::putObjectIntoFridge();
}
