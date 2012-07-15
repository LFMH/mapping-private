/*
 * depth_image_smoothing.cpp
 *
 *  Created on: Jun 1, 2012
 *      Author: blodow
 */

#include <depth_image_smoothing.h>
#include <iostream>
#include <pcl/common/time.h>

namespace ias_uima {
namespace pcl {

DepthImageSmoothing::DepthImageSmoothing()
  : config_ (Config())
  , use_gpu (false)
{
	// TODO: remove this for "production"
	use_gpu = false;
}

DepthImageSmoothing::~DepthImageSmoothing() {
}

void DepthImageSmoothing::setConfig (Config c)
{
  config_ = c;
}

DepthImageSmoothing::Config& DepthImageSmoothing::getConfig ()
{
  return config_;
}

std::vector <cv::Mat> DepthImageSmoothing::computeClampingImages (cv::Mat input)
{
	//::pcl::StopWatch watch;
	//std::cout << "DepthImageSmoothing::computeClampingImages: " << __LINE__ << " " << watch.getTimeSeconds () << std::endl;

	// prepare helper matrices
	cv::Mat1f min_disp (input.size());
	cv::Mat1f max_disp (input.size());
	//std::cout << "DepthImageSmoothing::computeClampingImages: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;

	double interval_size = 0;
	int interval_counter = 0;
	// fill min/max matrices
	for (int _y = 0; _y < input.rows; _y++)
		for (int _x = 0; _x < input.cols; _x++)
		{
			if (input.at<float>(_y,_x) == 0)
			{
				min_disp[_y][_x] = max_disp[_y][_x] = 0;
				continue;
			}
			if (isnan (input.at<float>(_y,_x)))
			{
				min_disp[_y][_x] = max_disp[_y][_x] = 0;
				continue;
			}

			// compute min, max disparity (clamping images)
			float disparity = depth2disparity (float (input.at<float>(_y,_x) * config_.depth_multiplier_), config_.baseline_, config_.focal_length_);
			min_disp[_y][_x] = disparity2depth (disparity + config_.disp_thresh_, config_.baseline_, config_.focal_length_) / config_.depth_multiplier_;
			max_disp[_y][_x] = disparity2depth (disparity - config_.disp_thresh_, config_.baseline_, config_.focal_length_) / config_.depth_multiplier_;

			interval_size += max_disp[_y][_x] - min_disp[_y][_x];
			interval_counter ++;
		}

	//std::cout << "interval average: " << interval_size / interval_counter << std::endl;

	// return clamping images
	std::vector<cv::Mat> ret;
	ret.push_back(min_disp);
	ret.push_back(max_disp);

	//std::cout << "DepthImageSmoothing::computeClampingImages: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;
	return ret;
}

cv::Mat DepthImageSmoothing::bilateralSmoothing (const cv::Mat input)
{
	// TODO: convert zeros to nans?
	//  float nan = std::numeric_limits<float>::quiet_NaN();
	//  input.convertTo (temp, CV_32F);
	//  for (int _y = 0; _y < temp.rows; _y++)
	//    for (int _x = 0; _x < temp.cols; _x++)
	//      if (temp[_y][_x] == 0)
	//        temp[_y][_x] = nan;
//	if (use_gpu)
//		return bilateralSmoothingGPU(input);

	//::pcl::StopWatch watch;
	//std::cout << "DepthImageSmoothing::bilateralSmoothing: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;

	cv::Mat pre (input.rows, input.cols, input.type());
	cv::Mat post (input.rows, input.cols, input.type());
	cv::Mat max_clamped (input.rows, input.cols, input.type());

	//std::cout << "DepthImageSmoothing::bilateralSmoothing: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;
    input.copyTo (pre);
	//std::cout << "DepthImageSmoothing::bilateralSmoothing: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;

	std::vector <cv::Mat> clampingImages = computeClampingImages (input);
	cv::Mat min = clampingImages[0];
	cv::Mat max = clampingImages[1];

	for (unsigned int i = 0; i < config_.smoothing_iterations_; ++i)
	{
		//std::cout << "DepthImageSmoothing::bilateralSmoothing: iteration " << i << " " << watch.getTimeSeconds() << std::endl;
		cv::GaussianBlur (pre, post, cv::Size(config_.smoothing_kernel_size_, config_.smoothing_kernel_size_), 2.0, 2.0);
		cv::max (post, min, max_clamped);
		cv::min (max_clamped, max, pre); // and the cycle repeats
	}

	return pre;
}

cv::Mat DepthImageSmoothing::bilateralSmoothingGPU (cv::Mat input)
{
//	::pcl::StopWatch watch;
//	std::cout << "DepthImageSmoothing::bilateralSmoothing: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;
//
//	cv::Mat output;
//	cv::gpu::GpuMat pre, post, max_clamped;
//	std::cout << "DepthImageSmoothing::bilateralSmoothing: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;
//    pre.upload (input);
//	std::cout << "DepthImageSmoothing::bilateralSmoothing: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;
//
//	cv::gpu::GpuMat buf (input.rows, input.cols, input.type());
//
//	std::vector <cv::Mat> clampingImages = computeClampingImages (input);
//	cv::gpu::GpuMat min;
//	min.upload (clampingImages[0]);
//	std::cout << "DepthImageSmoothing::bilateralSmoothing: " << __LINE__ << " " << watch.getTimeSeconds() << std::endl;
//
//	cv::gpu::GpuMat max;
//	max.upload (clampingImages[1]);
//
//	for (unsigned int i = 0; i < config_.smoothing_iterations_; ++i)
//	{
//		std::cout << "DepthImageSmoothing::bilateralSmoothing: iteration " << i << " " << watch.getTimeSeconds() << std::endl;
//		cv::gpu::GaussianBlur (pre, post, cv::Size(config_.smoothing_kernel_size_, config_.smoothing_kernel_size_), buf, 2.0, 2.0);
//		//cv::gpu::blur (pre, post, cv::Size(config_.smoothing_kernel_size_, config_.smoothing_kernel_size_));
//		cv::gpu::max (post, min, max_clamped);
//		cv::gpu::min (max_clamped, max, pre); // and the cycle repeats
//	}
//
//	pre.download(output);
//	return output;
}

} // end namespace
} // end namespace
