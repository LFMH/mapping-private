#ifndef _REALTIME_PERCEPTION_URDF_FILTERING_H
#define _REALTIME_PERCEPTION_URDF_FILTERING_H

#include <pcl/cuda/point_cloud.h>
#include <pcl/io/openni_camera/openni_depth_image.h>


namespace realtime_perception
{

  struct TransformationKernel
  {
    double* transformation;
    TransformationKernel (double* transformation)
     : transformation(transformation)
    {}

    __inline__ __host__ __device__ pcl::cuda::PointXYZRGB
      operator () (const pcl::cuda::PointXYZRGB &p);
  };

  struct RotationKernel
  {
    double* transformation;
    RotationKernel (double* transformation)
     : transformation(transformation)
    {}

    __inline__ __host__ __device__ float4
      operator () (const float4 &p);
  };

  struct BackgroundSubtractionKernel
  {
//    int    width, height;
//    int    center_x, center_y;
//    float  constant;
    float disp_thresh;

    BackgroundSubtractionKernel (/*int w, int h, int cx, int cy, float con,*/ float disp_thresh):
      /*width(w), height(h), center_x(cx), center_y(cy), constant(con),*/ disp_thresh(disp_thresh)
    { }

    template <typename Tuple> __inline__ __host__ __device__ float
      operator () (const Tuple &t);
  };

  class BackgroundSubtraction
  {
    public:

      template <template <typename> class Storage> int
      fromGLDepthImage (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image,
                        typename pcl::cuda::StoragePointer<Storage,float>::type gl_depth,
                        float constant, float disp_thresh,
                        typename Storage<float>::type &output,
                        bool downsample = false, int stride = 2);
  };

  
  template <template <typename> class Storage>
  void transformPoints (typename pcl::cuda::PointCloudAOS<Storage>::Ptr &out, const typename pcl::cuda::PointCloudAOS<Storage>::Ptr &in, double* transform);

  template <template <typename> class Storage>
  void transformNormals (boost::shared_ptr<typename Storage<float4>::type> &out, const boost::shared_ptr<typename Storage<float4>::type> &in, double* transform);

  template <template <typename> class Storage>
  void find_extrema (typename Storage<float>::type &array, float& min, float &max);

} // end namespace

#endif
