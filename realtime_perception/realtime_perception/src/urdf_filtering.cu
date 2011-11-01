#include <realtime_perception/urdf_filtering.h>
#include <thrust/count.h>
#include <thrust/extrema.h>

namespace realtime_perception 
{

pcl::cuda::PointXYZRGB
TransformationKernel::operator () (const pcl::cuda::PointXYZRGB &p)
{
  pcl::cuda::PointXYZRGB pt_out;
  pt_out.x = p.x*transformation[0] + p.y*transformation[4] + p.z*transformation[8] + transformation[12], 
  pt_out.y = p.x*transformation[1] + p.y*transformation[5] + p.z*transformation[9] + transformation[13], 
  pt_out.z = p.x*transformation[2] + p.y*transformation[6] + p.z*transformation[10] + transformation[14], 
  pt_out.rgb = p.rgb;
  return pt_out;
}

float4
RotationKernel::operator () (const float4 &p)
{
  return make_float4(
      p.x*transformation[0] + p.y*transformation[4] + p.z*transformation[8] , 
      p.x*transformation[1] + p.y*transformation[5] + p.z*transformation[9] , 
      p.x*transformation[2] + p.y*transformation[6] + p.z*transformation[10],
      p.w);
}

 //////////////////////////////////////////////////////////////////////////
template <typename Tuple> float
BackgroundSubtractionKernel::operator () (const Tuple &t)
{
  float zNear = 0.1f;
  float zFar = 100.0f;
  if (thrust::get<0>(t) == 0) // assume no_sample_value and shadow_value are also 0
    return 98;
    //return false;
  else
  {
    float depth_gl = thrust::get<1>(t);
  //  depth_gl = zNear + ( zFar - zNear ) * depth_gl;
    depth_gl = (zFar * zNear / (zNear - zFar)) / (depth_gl - zFar / (zFar - zNear));
    float depth_ni = ((float)thrust::get<0>(t)) * 0.001f;
    return (fabs(depth_ni - depth_gl));
    //return (fabs(depth_ni - depth_gl) < disp_thresh);
  }
}


template <template <typename> class Storage>
  void find_extrema (typename Storage<float>::type &array, float& min, float &max)
{
  thrust::pair <thrust::detail::normal_iterator<thrust::device_ptr<float> >,
                thrust::detail::normal_iterator<thrust::device_ptr<float> > >
  //thrust::pair<thrust::device_ptr<float> , thrust::device_ptr<float> > 
  result = thrust::minmax_element(array.begin(), array.end());

  min = *result.first;
  max = *result.second;
}



struct isTrue
{
  __host__ __device__
  bool operator()(const bool &x)
  {
    return x;
  }
};


template <template <typename> class Storage>
int BackgroundSubtraction::fromGLDepthImage (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image,
                                              typename pcl::cuda::StoragePointer<Storage,float>::type gl_depth,
                                              float constant, float disp_thresh,
                                              typename Storage<float>::type &output,
                                              bool downsample, int stride)
{
  // Copy the depth data on the card TODO: get this from pcl::io::DisparityToCloud::compute !
  typename Storage<float>::type depth (depth_image->getWidth() * depth_image->getHeight());
  unsigned short* depth_buffer = (unsigned short*)depth_image->getDepthMetaData ().Data ();
  thrust::copy (depth_buffer, (unsigned short*)(&depth_buffer[depth_image->getWidth() * depth_image->getHeight()]), depth.begin ());

  output.resize (depth_image->getWidth() * depth_image->getHeight());

  int image_size = depth_image->getWidth() * depth_image->getHeight ();
  thrust::transform (
      thrust::make_zip_iterator (thrust::make_tuple (depth.begin (), gl_depth)),
      thrust::make_zip_iterator (thrust::make_tuple (depth.begin (), gl_depth)) + image_size,
      output.begin (), 
      BackgroundSubtractionKernel (disp_thresh));

//  return thrust::count_if (output.begin (), output.end (), isTrue());
  return 0;
}

template <template <typename> class Storage>
void transformPoints (typename pcl::cuda::PointCloudAOS<Storage>::Ptr &out, const typename pcl::cuda::PointCloudAOS<Storage>::Ptr &in, double* transform)
{
  out.reset (new typename pcl::cuda::PointCloudAOS<Storage>);

  pcl::cuda::Host<double>::type transform_host (transform, transform + 16);
  typename Storage<double>::type transform_storage = transform_host;
  out->points.resize (in->points.size ());
  out->width = in->width;
  out->height = in->height;
  out->is_dense = in->is_dense;
  thrust::transform (in->points.begin (), in->points.end (), out->points.begin (), TransformationKernel (thrust::raw_pointer_cast(&transform_storage[0])));
}

template <template <typename> class Storage>
void transformNormals (boost::shared_ptr<typename Storage<float4>::type> &out, const boost::shared_ptr<typename Storage<float4>::type> &in, double* transform)
{
  out.reset (new typename Storage<float4>::type);

  pcl::cuda::Host<double>::type transform_host (transform, transform + 16);
  typename Storage<double>::type transform_storage = transform_host;
  out->resize (in->size ());
  thrust::transform (in->begin (), in->end (), out->begin (), RotationKernel (thrust::raw_pointer_cast(&transform_storage[0])));
}

//template int
//BackgroundSubtraction::fromGLDepthImage<pcl::cuda::Host> (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image,
//                                                          pcl::cuda::StoragePointer<pcl::cuda::Host,float>::type gl_depth,
//                                                          float constant, float disp_thresh,
//                                                          pcl::cuda::Host<bool>::type &output,
//                                                          bool downsample, int stride);
//
template int
BackgroundSubtraction::fromGLDepthImage<pcl::cuda::Device> (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image,
                                                            typename pcl::cuda::StoragePointer<pcl::cuda::Device,float>::type gl_depth,
                                                            float constant, float disp_thresh,
                                                            pcl::cuda::Device<float>::type &output,
                                                            bool downsample, int stride);
  

template void find_extrema<pcl::cuda::Device> (thrust::device_vector<float> &array, float& min, float &max);

template 
void transformPoints<pcl::cuda::Device> (typename pcl::cuda::PointCloudAOS<pcl::cuda::Device>::Ptr &out, const typename pcl::cuda::PointCloudAOS<pcl::cuda::Device>::Ptr &in, double* transform);

template 
void transformNormals<pcl::cuda::Device> (boost::shared_ptr<typename pcl::cuda::Device<float4>::type> &out, const boost::shared_ptr<typename pcl::cuda::Device<float4>::type> &in, double* transform);
 
  
} // end namespace
