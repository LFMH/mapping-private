#include "ros/ros.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Pose.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "pcl/common/angles.h"
#include "LinearMath/btQuaternion.h"
#include <pr2_controllers_msgs/SingleJointPositionAction.h>
#include <pr2_msgs/SetPeriodicCmd.h>
#include <pr2_msgs/PeriodicCmd.h>
#include <pr2_msgs/LaserTrajCmd.h>
#include <pr2_msgs/SetLaserTrajCmd.h>
#include "pr2_msgs/LaserScannerSignal.h"
#include <pcl_ros/publisher.h>
#include <pcl_ros/pcl_nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "pcl/io/pcd_io.h"
#include "pcl/filters/voxel_grid.h"
#include <geometry_msgs/PoseArray.h>


namespace autonomous_exploration
{
class AutonomousExploration:public pcl_ros::PCLNodelet
{
public:
	AutonomousExploration();
	~AutonomousExploration();
	void autonomousExplorationCallBack(const geometry_msgs::PoseArray& pose_msg);
	void pointcloudCallBack(const sensor_msgs::PointCloud2& pointcloud_msg);
	void laserScannerSignalCallBack(const pr2_msgs::LaserScannerSignal laser_scanner_signal_msg);
	void moveRobot(geometry_msgs::Pose goal_pose);
	void moveTorso(double position, double velocity, std::string direction);
	void setLaserProfile(std::string mode);
	void onInit();
	void spin();
protected:
	using pcl_ros::PCLNodelet::pnh_;
private:
	//ros::NodeHandle nh_;
	ros::Subscriber pose_subscriber_;
	ros::Subscriber pointcloud_subscriber_;
	ros::Subscriber laser_signal_subscriber_;
	ros::Publisher pointcloud_publisher_;
	ros::Publisher tilt_laser_publisher_;
	ros::Publisher tilt_laser_traj_cmd_publisher_;
	std::string subscribe_pose_topic_;
	boost::thread spin_thread_;
	geometry_msgs::PoseArray pose_msg_;
	bool get_pointcloud_, received_pose_, received_laser_signal_,move_robot_, publish_cloud_incrementally_;
	//ros::ServiceClient tilt_laser_client_;
	pcl::PointCloud<pcl::PointXYZ> cloud_merged_;
};

AutonomousExploration::AutonomousExploration()
{

}
AutonomousExploration::~AutonomousExploration()
{
	ROS_INFO("Shutting down autonomous_exploration node.");
}

void AutonomousExploration::onInit()
{
	pcl_ros::PCLNodelet::onInit ();
	move_robot_=false;
	pnh_->param("subscribe_pose_topic", subscribe_pose_topic_, std::string("/robot_pose"));
	pnh_->param("publish_cloud_incrementally", publish_cloud_incrementally_, false);
	ROS_INFO("autonomous_exploration node is up and running.");
	get_pointcloud_ = false;
	received_pose_ = false;
	received_laser_signal_ = false;
	pose_subscriber_ = pnh_->subscribe(subscribe_pose_topic_, 100, &AutonomousExploration::autonomousExplorationCallBack, this);
	pointcloud_publisher_ = pnh_->advertise<sensor_msgs::PointCloud2>("pointcloud", 100);
	tilt_laser_publisher_ = pnh_->advertise<pr2_msgs::PeriodicCmd>("/laser_tilt_controller/set_periodic_cmd", 1);
	pointcloud_subscriber_ = pnh_->subscribe("/points2_out", 100, &AutonomousExploration::pointcloudCallBack, this);
	laser_signal_subscriber_ = pnh_->subscribe("/laser_tilt_controller/laser_scanner_signal", 1, &AutonomousExploration::laserScannerSignalCallBack, this);
	// tilt_laser_client_ = pnh_->serviceClient<pr2_msgs::SetPeriodicCmd>("laser_tilt_controller/set_periodic_cmd");
	tilt_laser_traj_cmd_publisher_ = pnh_->advertise<pr2_msgs::LaserTrajCmd>("/laser_tilt_controller/set_traj_cmd", 1);
	spin_thread_ = boost::thread (boost::bind (&AutonomousExploration::spin, this));
	//ros::spin();
}
void AutonomousExploration::spin()
{
	ros::Rate loop(1);
	while(pnh_->ok())
	{
		loop.sleep();
		if(received_pose_ )
		{
			for (int i=0;i<pose_msg_.poses.size();i++)
			{
				if (!move_robot_)
				{
					//move robot to the received pose
					ROS_INFO("Moving the robot");
					moveRobot(pose_msg_.poses[i]);
					ROS_INFO("End of Moving the robot");
				}
				else break;

				if (!move_robot_)
				{
					ROS_INFO("no good pose to navigate to");
					exit(1);
				}
				//rise the spine up to scan
				setLaserProfile("scan");
				moveTorso(0.3, 1.0, "up");
				double angles = 0.0;
				while(angles <= 360)
				{
					geometry_msgs::Pose new_pose = pose_msg_.poses[i];
					btQuaternion q ( pcl::deg2rad(angles), 0, 0);

					new_pose.orientation.x = q.x();
					new_pose.orientation.y = q.y();
					new_pose.orientation.z = q.z();
					new_pose.orientation.w = q.w();
					//setLaserProfile("scan");
					moveRobot(new_pose);
					//setLaserProfile("navigation");

					received_laser_signal_ = false;
					while(!received_laser_signal_)
					{
						loop.sleep();
					}

					//TODO: Use laser service to sync
					//Wait 11 seconds before taking a scan
					sleep(11);
					//get a point cloud
					get_pointcloud_ = true;
					angles += 30.0;
				}
			}
			while (!get_pointcloud_)
			{
				loop.sleep();
			}
			ROS_INFO("Published aggregated cloud with %ld points", cloud_merged_.points.size());
			cloud_merged_.header.stamp = ros::Time::now();
			pointcloud_publisher_.publish(cloud_merged_);

			//Clear points for the next round
			cloud_merged_.points.clear();
			cloud_merged_.width = 0;
			cloud_merged_.height = 0;
			received_pose_ = false;
			//lower the spine to navigate some place else
			moveTorso(0.01, 1.0, "down");
			setLaserProfile("navigation");

		}
	}
}
void AutonomousExploration::moveRobot(geometry_msgs::Pose goal_pose)
{
	actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

	//wait for the action server to come up
	while(!ac.waitForServer(ros::Duration(5.0)))
	{
		ROS_INFO("Waiting for the move_base action server to come up");
	}

	move_base_msgs::MoveBaseGoal goal;
	goal.target_pose.header.frame_id = "/map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose = goal_pose;

	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();

	ROS_INFO("Pose Position x: %f, y:%f, z:%f Orientation x:%f, y:%f, z:%f, w:%f", goal.target_pose.pose.position.x,
			goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x,
			goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);

	if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_INFO("Robot has moved to the goal pose");
		move_robot_=true;
	}
	else
	{
		ROS_ERROR("Error in moving robot to goal pose.");
		//exit(1);
		move_robot_=false;
	}
}

void AutonomousExploration::moveTorso(double position, double velocity, std::string direction)
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
}

void AutonomousExploration::setLaserProfile(std::string mode)
{
	//while ( !ros::service::waitForService("laser_tilt_controller/set_periodic_cmd", ros::Duration(2.0)) && pnh_->ok() )
	//{
	//  ROS_INFO("Waiting for tilt laser periodic command service to come up");
	//}

	//pr2_msgs::SetPeriodicCmd::Request req;
	//pr2_msgs::SetPeriodicCmd::Response res;
	//req.command.header.frame_id = "/map";
	//req.command.header.stamp = ros::Time::now();
	//req.command.header.seq = 0;

	//req.command.profile = "linear";

	if(mode == "scan")
	{
		pr2_msgs::PeriodicCmd periodic_cmd_msg;

		periodic_cmd_msg.header.frame_id = "/map";
		periodic_cmd_msg.header.stamp = ros::Time::now();

		periodic_cmd_msg.profile = "linear";
		periodic_cmd_msg.offset = 0.3;
		periodic_cmd_msg.period = 20.0;
		periodic_cmd_msg.amplitude = 0.8;
		//req.command.period = 20.0;
		//req.command.amplitude = 0.8;
		//req.command.offset = 0.3;
		tilt_laser_publisher_.publish(periodic_cmd_msg);
		ROS_INFO("Tilt laser successfully published. Mode: %s", mode.c_str());
	}
	else if(mode == "navigation")
	{
		pr2_msgs::LaserTrajCmd laser_traj_cmd_msg;

		laser_traj_cmd_msg.header.frame_id = "/map";
		laser_traj_cmd_msg.header.stamp = ros::Time::now();

		laser_traj_cmd_msg.profile = "blended_linear";
		laser_traj_cmd_msg.position.resize(3);
		ROS_INFO("Resized");
		laser_traj_cmd_msg.position[0] = 1.05;
		laser_traj_cmd_msg.position[1] = -0.7;
		laser_traj_cmd_msg.position[2] = 1.05;

		ros::Duration dur;
		laser_traj_cmd_msg.time_from_start.resize(3);
		ROS_INFO("Resized time_from_start");
		laser_traj_cmd_msg.time_from_start[0] = dur.fromSec(0.0);
		laser_traj_cmd_msg.time_from_start[1] = dur.fromSec(1.8);
		laser_traj_cmd_msg.time_from_start[2] = dur.fromSec(2.3125);

		laser_traj_cmd_msg.max_velocity = 10;
		laser_traj_cmd_msg.max_acceleration = 30;
		tilt_laser_traj_cmd_publisher_.publish(laser_traj_cmd_msg);
		ROS_INFO("Tilt Laser Traj Command successfully called. Mode: %s\n", mode.c_str());

	}
	//if(!tilt_laser_client_.call(req,res))
	//{
	//  ROS_ERROR("Tilt laser service call failed.\n");
	//  exit(1);
	//}
	//else
	//{
	//  ROS_INFO("Tilt laser service successfully called.\n");
	//}
}
void AutonomousExploration::laserScannerSignalCallBack(const pr2_msgs::LaserScannerSignal laser_scanner_signal_msg)
{
	if(!received_laser_signal_)
	{
		received_laser_signal_ = true;
	}
}
void AutonomousExploration::pointcloudCallBack(const sensor_msgs::PointCloud2& pointcloud_msg)
{
	if(get_pointcloud_)
	{
		ROS_INFO("pointcloud received");

		if (publish_cloud_incrementally_)
		{
			pointcloud_publisher_.publish(pointcloud_msg);
		}
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp_filtered (new pcl::PointCloud<pcl::PointXYZ> ());
		pcl::fromROSMsg(pointcloud_msg, *cloud_tmp);
		pcl::VoxelGrid<pcl::PointXYZ> sor;
		sor.setInputCloud (cloud_tmp);
		sor.setLeafSize (0.02, 0.02, 0.02);
		sor.filter (*cloud_tmp_filtered);


		if (cloud_merged_.points.size() == 0)
			cloud_merged_ = *cloud_tmp_filtered;
		else
			cloud_merged_ += *cloud_tmp_filtered;
		//    ROS_INFO("pointcloud published");
		get_pointcloud_ = false;
	}
}
void AutonomousExploration::autonomousExplorationCallBack(const geometry_msgs::PoseArray& pose_msg)
{
	pose_msg_=pose_msg;
	received_pose_ = true;
	ROS_INFO("Received a Pose");

}

}
PLUGINLIB_DECLARE_CLASS(autonomous_mapping,AutonomousExploration,autonomous_exploration::AutonomousExploration,nodelet::Nodelet);

