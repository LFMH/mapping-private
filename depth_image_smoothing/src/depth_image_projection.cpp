/*
 * depth_image_projection.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: blodow
 */

#include <depth_image_projection.h>

namespace ias_uima {
namespace pcl {

DepthImageProjection::DepthImageProjection() {
	// TODO Auto-generated constructor stub

}

DepthImageProjection::~DepthImageProjection() {
	// TODO Auto-generated destructor stub
}

::pcl::PointCloud< ::pcl::PointXYZRGB>::Ptr DepthImageProjection::project (const cv::Mat & depth_image, const cv::Mat & rgb_image)
{
	::pcl::PointCloud< ::pcl::PointXYZRGB>::Ptr cloud (new ::pcl::PointCloud< ::pcl::PointXYZRGB>);
	// TODO cloud->header.stamp = time;
	cloud->height = depth_image.rows;
	cloud->width = depth_image.cols;
	cloud->is_dense = false;

	cloud->points.resize(cloud->height * cloud->width);

	register float constant = 1.0f / 585;

	register int centerX = (cloud->width >> 1);
	int centerY = (cloud->height >> 1);

	float bad_point = std::numeric_limits<float>::quiet_NaN();

	register int depth_idx = 0;
	for (int v = -centerY; v < centerY; ++v)
	{
		for (register int u = -centerX; u < centerX; ++u, ++depth_idx)
		{
			::pcl::PointXYZRGB& pt = cloud->points[depth_idx];
			// Check for invalid measurements
			float depth = depth_image.at<float>(v + centerY, u + centerX);
			if (isnan(depth) || depth == 0)
			{
				// not valid
				pt.x = pt.y = pt.z = bad_point;
				continue;
			}
			pt.z = depth;
			pt.x = u * pt.z * constant;
			pt.y = v * pt.z * constant;

			cv::Vec3b rgb = rgb_image.at<cv::Vec3b>(v + centerY, u + centerX);

			pt.r = rgb[0];
			pt.g = rgb[1];
			pt.b = rgb[2];
		}
	}
	return cloud;

}

} // end namespace
} // end namespace
