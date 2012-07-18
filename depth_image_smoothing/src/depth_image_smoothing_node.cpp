#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <depth_image_smoothing.h>
#include <depth_image_projection.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>

using namespace sensor_msgs;
using namespace message_filters;

ias_uima::pcl::DepthImageSmoothing dis;
ias_uima::pcl::DepthImageProjection dip;
ros::Publisher pub;

void callback(const ImageConstPtr& image1, const ImageConstPtr& image2)
{
  cv_bridge::CvImageConstPtr depth_img = cv_bridge::toCvShare(image1);
  cv_bridge::CvImageConstPtr rgb_img = cv_bridge::toCvShare(image2);

  //cv::imwrite("depth.png", depth_img->image);
  //cv::imwrite("rgb.png", rgb_img->image);

  cv::Mat depth_float;
  depth_img->image.convertTo(depth_float, CV_32F);

  //  float bad_point = std::numeric_limits<float>::quiet_NaN();
  //
  //  cv::Mat mask;
  //  cv::compare(depth_float, 0, mask, cv::CMP_EQ);
  //  depth_float.setTo(bad_point, mask);
  //
  //  cv::compare(depth_float, 2048, mask, cv::CMP_GE);
  //  depth_float.setTo(bad_point, mask);

  //cv::imwrite("depth_float.png", depth_float);

  cv::Mat depth_smoothed = dis.bilateralSmoothing(depth_float);
  //cv::imwrite("depth_smoothed.png", depth_smoothed);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = dip.project(depth_smoothed, rgb_img->image);

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg<pcl::PointXYZRGB>(*cloud, msg);
  msg.header = image2->header;
  //msg.header.frame_id = image2->header.frame_id;
  msg.header.stamp = ros::Time::now();
  //msg.header.frame_id = "/openni_rgb_frame";
  pub.publish(msg);

  //std::cerr << "Got frame" << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  ros::NodeHandle node_handle_private = ros::NodeHandle("~");

  int smoothing_kernel_size = 5, smoothing_iterations = 1;
  double depth_multiplier = 1.0f;

  node_handle_private.getParam("smoothing_kernel_size", smoothing_kernel_size);
  node_handle_private.getParam("smoothing_iterations", smoothing_iterations);
  node_handle_private.getParam("depth_multiplier", depth_multiplier);

  ias_uima::pcl::DepthImageSmoothing::Config c(depth_multiplier, smoothing_iterations, smoothing_kernel_size);
  dis.setConfig(c);

  pub = nh.advertise<sensor_msgs::PointCloud2> ("/camera/smooth/points", 1);

  message_filters::Subscriber<Image> image1_sub(nh, "/camera/depth/image", 1);
  message_filters::Subscriber<Image> image2_sub(nh, "/camera/rgb/image_color", 1);

  typedef sync_policies::ApproximateTime<Image, Image> MySyncPolicy;
  // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image1_sub, image2_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}
