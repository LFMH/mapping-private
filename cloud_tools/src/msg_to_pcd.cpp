// #include <unistd.h>

#include <ctime>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud.h>
#include <point_cloud_mapping/cloud_io.h>
#include <boost/thread/mutex.hpp>
#include <boost/format.hpp>
#include <ros/ros.h>
class MsgToPCD
{
  protected:
    ros::NodeHandle nh_;
    //dir_ - destination folder for pcds
    std::string input_cloud_topic_, dir_;
    //check if there is someone exporting the name for our pcd
    std::string get_name_from_param_server_;
    ros::Subscriber cloud_sub_;
    int counter_;
    //for continous saving of pcds
    int nr_saved_pcds_; 
    bool debug_;
    std::string name_;
    //lock while saving to pcd
    boost::mutex m_lock_;
    boost::format filename_format_;

  public:
  MsgToPCD () :  nh_("~"), counter_(0), debug_(true)
    {
      nh_.param ("input_cloud_topic", input_cloud_topic_, std::string("cloud_pcd"));
      nh_.param ("name", name_, std::string("cloud"));
      nh_.param ("dir", dir_, std::string(""));       
      nh_.param ("nr_saved_pcds", nr_saved_pcds_, 0);   
      nh_.param ("get_name_from_param_server", get_name_from_param_server_, std::string(""));
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
      if(get_name_from_param_server_ != "")
      {
        ros::param::get(get_name_from_param_server_, name_);
        std::cerr << "name: " << name_ << std::endl;
      }
      filename_format_.parse(name_ + std::string("_%f.pcd"));
      std::string filename = dir_ + (filename_format_ %  ros::Time::now().toSec()).str();
      if(debug_)
      ROS_INFO("parameter nr_saved_pcds: %d", nr_saved_pcds_);
//       if ((counter_ == 0) && (nr_saved_pcds_ == 0))
//       {
//         ROS_INFO ("PointCloud message received on %s with %d points. Saving to %s", input_cloud_topic_.c_str (), (int)cloud->points.size (), filename.c_str ());
//         cloud_io::savePCDFile (filename.c_str (), *cloud, true);
//       }
      if (counter_ < nr_saved_pcds_)
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
      nh_.getParam ("nr_saved_pcds", nr_saved_pcds_);
      if(debug_)
      ROS_INFO("cont in update: %d", nr_saved_pcds_);
    }


    bool 
      spin ()
    {
      ros::Rate loop_rate(1);
      while (nh_.ok())
      {
        ros::spinOnce ();
        if (counter_ >= nr_saved_pcds_)
          return true;
	loop_rate.sleep();
        //updateParametersFromServer();
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

