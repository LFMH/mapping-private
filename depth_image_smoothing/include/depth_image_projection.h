/*
 * depth_image_projection.h
 *
 *  Created on: Jun 1, 2012
 *      Author: blodow
 */

#ifndef DEPTH_IMAGE_PROJECTION_H_
#define DEPTH_IMAGE_PROJECTION_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

namespace ias_uima {
namespace pcl{

class DepthImageProjection {
public:
	DepthImageProjection();
	virtual ~DepthImageProjection();
	::pcl::PointCloud< ::pcl::PointXYZRGB>::Ptr project (const cv::Mat & depth_image, const cv::Mat & rgb_image);
};

}
}

#endif /* DEPTH_IMAGE_PROJECTION_H_ */
