#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

//Segmentation includes
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/ModelCoefficients.h"

//Point type
#include "pointcloud_segmentation/pointcloud_segmentation_point_types.h"

class PointCloudSegmentation
{
  public:
    PointCloudSegmentation();
    ~PointCloudSegmentation();
    void pointcloudSegmentationCallBack(const sensor_msgs::PointCloud2& msg);
    void run();

    void publishPointCloud(pcl::PointCloud<pcl::PointSegmentation> pointcloud);
    void publishPointCloud(pcl::PointCloud<pcl::PointNormal> pointcloud);
    void segmentPointCloud(pcl::PointCloud<pcl::PointNormal> &pointcloud);
  private:
    ros::NodeHandle nh_;
    std::string subscribe_pointcloud_topic_, frame_id_;
    ros::Subscriber pointcloud_subscriber_;
    ros::Publisher pointcloud_publisher_;
    ros::Publisher segment_publisher_;
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointNormal> seg_;
    Eigen::Vector3f axis_;
    int label_;

};


