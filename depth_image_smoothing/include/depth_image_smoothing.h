/*
 * depth_image_smoothing.h
 *
 *  Created on: Jun 1, 2012
 *      Author: blodow
 */

#ifndef DEPTHIMAGESMOOTHING_H_
#define DEPTHIMAGESMOOTHING_H_

#include <opencv2/gpu/gpu.hpp>
#include <opencv2/gpu/gpumat.hpp>

namespace ias_uima {
namespace pcl {

class DepthImageSmoothing {
public:
	// configuration object describing filtering behaviour
    struct Config
    {
      Config (float dm = 1.0f, int i = 7, int s = 5)
        : depth_multiplier_ (dm)
        , smoothing_iterations_ (i)
        , smoothing_kernel_size_ (s)
        , baseline_ (0.075)
        , focal_length_ (525)
        , disp_thresh_ (1.0f/8.0f)
      {}

      float depth_multiplier_;
      int smoothing_iterations_;
      int smoothing_kernel_size_; // must be odd
      float baseline_;
      float focal_length_;
      float disp_thresh_;
    };

    void setConfig (Config c);
    Config &getConfig ();

	DepthImageSmoothing();
	virtual ~DepthImageSmoothing();

	template <typename T>
	inline T clamp (T what, T l, T u)
	{
		return std::min<T> (std::max<T> (what, l), u);
	}

	inline float depth2disparity (float depth, float baseline_, float focal_length_)
	{
		return baseline_ * focal_length_ / depth;
	}

	inline
	float disparity2depth (float disparity, float baseline_, float focal_length_)
	{
		return baseline_ * focal_length_ / disparity;
	}

	std::vector<cv::Mat> computeClampingImages (cv::Mat input);

	cv::Mat bilateralSmoothing (cv::Mat input);
	cv::Mat bilateralSmoothingGPU (cv::Mat input);

  protected:
    Config config_;
    bool use_gpu;
};

} // end namespace
} // end namespace

#endif /* DEPTHIMAGESMOOTHING_H_ */
