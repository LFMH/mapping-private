// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>

class MsgToPCD
{
  protected:
    ros::NodeHandle nh_;
    //dir_ - destination folder for pcds
    std::string input_cloud_topic_, dir_;
    ros::Subscriber cloud_sub_;
    int counter_;
    //for continous saving of pcds
    bool continous_; 
    bool debug_;
    //lock while saving to pcd
    boost::mutex m_lock_;
    boost::format filename_format_;

  public:
  MsgToPCD () :  nh_("~"), counter_(0), debug_(true)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
      nh_.param ("dir", dir_, std::string(""));       
      nh_.param ("continous", continous_, false);   
      if(debug_)
      ROS_INFO("input_cloud_topic_: %s", input_cloud_topic_.c_str());
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &MsgToPCD::cloud_cb, this);
    }
    
    void
      cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
    {
      //std::ostringstream filename;
      //filename << dir_ << "cloud_" << time (NULL) << "_" << getpid () << ".pcd";
      //filename << dir_ << "cloud_" <<  << ".pcd";
      filename_format_.parse(std::string("cloud_%f.pcd"));
      std::string filename = dir_ + (filename_format_ %  ros::Time::now().toSec()).str();
      if(debug_)
      ROS_INFO("parameter continous: %d", continous_);
      if ((counter_ == 0) && (!continous_))
      {
        ROS_INFO ("PointCloud message received on %s with %d points. Saving to %s", input_cloud_topic_.c_str (), (int)cloud->points.size (), filename.c_str ());
        cloud_io::savePCDFile (filename.c_str (), *cloud, true);
      }
      if ((counter_ >= 0) && (continous_))
      {
        ROS_INFO ("PointCloud message received on %s with %d points. Saving to %s", input_cloud_topic_.c_str (), (int)cloud->points.size (), filename.c_str ());
        m_lock_.lock ();
        cloud_io::savePCDFile (filename.c_str (), *cloud, true);
        m_lock_.unlock ();
      }
      counter_ ++;
    }

  /**
   * @brief toggles parameter to start/stop saving pcds
   */

   void updateParametersFromServer ()
    {
      nh_.getParam ("continous", continous_);
      if(debug_)
      ROS_INFO("cont in update: %d", continous_);
    }


    bool 
      spin ()
    {
      ros::Rate loop_rate(1);
      while (nh_.ok())
      {
        ros::spinOnce ();
        if ((counter_ == 1) && (!continous_))
          return true;
        updateParametersFromServer();
	loop_rate.sleep();
      }
      return true;
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "msg_to_pcd");

  MsgToPCD n;
  n.spin ();

  return (0);
}

