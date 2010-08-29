#include <pointcloud_segmentation/pointcloud_segmentation.h>

pcl::PointCloud<pcl::PointNormal> updatePointCloud(pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointIndices inliers)
{
  int check = 0;
  pcl::PointCloud<pcl::PointNormal> ret_cloud;

  for(u_int i = 0; i < cloud.points.size(); i++)
  {
    for(u_int j = 0 ; i < inliers.indices.size(); j++)
    {
      if( i == inliers.indices[j])
      {
        check = 1;
        break;
      }
    }
    if(check == 0)
    {
      ret_cloud.points.push_back(cloud.points[i]);
    }
    else
      check = 0;
  }

  return (ret_cloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudSegmentation::PointCloudSegmentation(): nh_("~")
{
  nh_.param("subscribe_pointcloud_topic", subscribe_pointcloud_topic_, std::string("/point_cloud"));
  axis_[0] = 0; axis_[1] = 0.5; axis_[2] = 0;
  label_ = 0;
  ROS_INFO("pointcloud_segmentation node is up and running.");
  run();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PointCloudSegmentation::~PointCloudSegmentation()
{
  ROS_INFO("Shutting down pointcloud_segmentation node!");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::run()
{
  pointcloud_subscriber_ = nh_.subscribe(subscribe_pointcloud_topic_, 100, &PointCloudSegmentation::pointcloudSegmentationCallBack, this);
  segment_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("segmented_plane", 100);
  pointcloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("unsegmented_cloud", 100);
  ros::spin();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::publishPointCloud(pcl::PointCloud<pcl::PointNormal> pointcloud)
{
  sensor_msgs::PointCloud2 pointcloud2_msg;
  pointcloud.width = 1;
  pointcloud.height = pointcloud.points.size();
  pcl::toROSMsg(pointcloud, pointcloud2_msg);
  pointcloud2_msg.header.frame_id = frame_id_;
  pointcloud2_msg.header.stamp = ros::Time::now();
  pointcloud_publisher_.publish(pointcloud2_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::publishPointCloud(pcl::PointCloud<pcl::PointSegmentation> pointcloud)
{
  sensor_msgs::PointCloud2 pointcloud2_msg;
  pcl::toROSMsg(pointcloud, pointcloud2_msg);
  pointcloud2_msg.header.frame_id = frame_id_;
  pointcloud2_msg.header.stamp = ros::Time::now();
  segment_publisher_.publish(pointcloud2_msg);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::segmentPointCloud(pcl::PointCloud<pcl::PointNormal> &input_cloud)
{
  ROS_INFO("Segmenting Point Cloud of size %d", (int) input_cloud.points.size());
  label_ ++;
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;
  pcl::PointCloud<pcl::PointSegmentation> segmented_plane;

  seg_.setOptimizeCoefficients (true);
  seg_.setModelType (pcl::SACMODEL_PLANE);
  seg_.setMethodType (pcl::SAC_RANSAC);
  seg_.setDistanceThreshold (0.01);
  seg_.setAxis(axis_);

  seg_.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointNormal> > (input_cloud));
  seg_.segment(inliers, coefficients);

  if (inliers.indices.size () == 0)
  {
    ROS_ERROR ("Could not estimate a planar model for the given dataset.");
    exit(1);
  }

  segmented_plane.points.resize(inliers.indices.size());
  for(u_int i = 0 ; i < inliers.indices.size(); i++)
  {
    segmented_plane.points[i].x = input_cloud.points[inliers.indices[i]].x;
    segmented_plane.points[i].y = input_cloud.points[inliers.indices[i]].y;
    segmented_plane.points[i].z = input_cloud.points[inliers.indices[i]].z;
    segmented_plane.points[i].normal[0] = input_cloud.points[inliers.indices[i]].normal[0];
    segmented_plane.points[i].normal[1] = input_cloud.points[inliers.indices[i]].normal[1];
    segmented_plane.points[i].normal[2] = input_cloud.points[inliers.indices[i]].normal[2];
    segmented_plane.points[i].curvature = input_cloud.points[inliers.indices[i]].curvature;
    segmented_plane.points[i].label = label_;
    //input_cloud.points.erase(input_cloud.points.begin() + inliers.indices[i]);
  }
  publishPointCloud(segmented_plane);
  ROS_INFO("Segmented Plane published");
  input_cloud = updatePointCloud(input_cloud, inliers);
  //pcl::PointCloud<pcl::PointNormal> pc;
  //pcl::copyPointCloud(input_cloud, inliers, pc);
  //input_cloud = input_cloud - pc;
  publishPointCloud(input_cloud);
  ROS_INFO("Current unsegmented point cloud published");
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void PointCloudSegmentation::pointcloudSegmentationCallBack(const sensor_msgs::PointCloud2& pointcloud2_msg)
{
  //Local variables
  pcl::PointCloud<pcl::PointNormal> input_cloud;
  pcl::ModelCoefficients coefficients;
  pcl::PointIndices inliers;

  ROS_INFO("Received a point cloud.");
  frame_id_ = pointcloud2_msg.header.frame_id;

  pcl::fromROSMsg(pointcloud2_msg, input_cloud);

  while( input_cloud.points.size() > 100 )
  {
    segmentPointCloud(input_cloud);
    sleep(2);
  }

  ROS_INFO("Out of points");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pointcloud_segmentation");
    PointCloudSegmentation pointcloud_segmentation;
    return(0);
}


