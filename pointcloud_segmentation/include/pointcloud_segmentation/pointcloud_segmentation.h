#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

class PointCloudSegmentation
{
  public:
    PointCloudSegmentation();
    ~PointCloudSegmentation();
    void pointcloudSegmentationCallBack(const sensor_msgs::PointCloud2& msg);
    void run();

  private:
    ros::NodeHandle nh_;
    std::string subscribe_pointcloud_topic_, frame_id_;
    ros::Subscriber pointcloud_subscriber_;
};


