/*
 * semantic_map_to_marker.cpp
 *
 *  Created on: Oct 11, 2010
 *      Author: marton
 */
//#include <cstdlib>
#include <iostream>
#include <iterator>
//#include <vector>

#include <ros/ros.h>
#include <mod_semantic_map/SemMap.h>
#include <visualization_msgs/Marker.h>
//#include <visualization_msgs/MarkerArray.h>

// Eigen
#include <Eigen3/Core>
#include <Eigen3/Geometry>

class SemanticMapToMarker
{
  protected:

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::NodeHandle nh_;

  public:

    double alpha_;

    void mapCallback(const mod_semantic_map::SemMap::ConstPtr& msg)
    {
      //nh_.getParam ("alpha", alpha_);
      ROS_INFO ("[SemanticMapToMarker] Publishing %zu objects on the %s topic with alpha %g", msg->objects.size (), pub_.getTopic ().c_str (), alpha_);
      for (unsigned i = 0; i < msg->objects.size (); i++)
      {
        visualization_msgs::Marker marker;
        Eigen3::Map<Eigen3::Matrix4f> pose (&msg->objects[i].pose[0]); // map an eigen matrix onto the vector
        pose.transposeInPlace(); /// @NOTE: the mapped vector holds the matrices transposed!
        Eigen3::Quaternion<float> qt (pose.topLeftCorner<3,3> ());

        //marker.header.frame_id = "base_link";
        //marker.header.stamp = ros::Time();
        marker.header = msg->header;
        marker.header.stamp += ros::Duration (i * 0.001); // TODO is this hack of Radu's needed?
        marker.ns = "SemanticMapToMarker";
        marker.id = msg->objects[i].id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pose(0,3);
        marker.pose.position.y = pose(1,3);
        marker.pose.position.z = pose(2,3);
        marker.pose.orientation.x = qt.x();
        marker.pose.orientation.y = qt.y();
        marker.pose.orientation.z = qt.z();
        marker.pose.orientation.w = qt.w();
        marker.scale.x = msg->objects[i].depth;
        marker.scale.y = msg->objects[i].width;
        marker.scale.z = msg->objects[i].height;
        marker.color.a = alpha_;
        marker.color.r = msg->objects[i].type == "handle" || msg->objects[i].type == "knob" ? 1 : 0.5;
        marker.color.g = msg->objects[i].type == "door" ? 1 : 0.5;
        marker.color.b = msg->objects[i].type == "horizontal_plane" ? 1 : 0.5;

        pub_.publish (marker);
      }
    }

    SemanticMapToMarker (ros::NodeHandle nh, double alpha)
    {
      alpha_ = alpha;
      nh_ = nh;
      sub_ = nh_.subscribe("semantic_map", 100, &SemanticMapToMarker::mapCallback, this);
      ROS_INFO ("[SemanticMapToMarker] Subscribed to: %s", sub_.getTopic ().c_str ());
      pub_ = nh_.advertise<visualization_msgs::Marker>("markers", 0 );
      ROS_INFO ("[SemanticMapToMarker] Publishing on: %s", pub_.getTopic ().c_str ());
    }
};

int main(int argc, char **argv)
{
  double alpha = 0.75;
  if (argc > 2)
  {
    std::stringstream ss (argv[2]);
    ss >> alpha;
  }

  ros::init(argc, argv, "semantic_map_to_marker");
  ros::NodeHandle nh ("~");
  SemanticMapToMarker sm2m (nh, alpha);
  ros::spin();
  return 0;
}
