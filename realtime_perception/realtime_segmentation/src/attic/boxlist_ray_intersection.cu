#include <pcl/pcl_macros.h>

#include "boxlist_ray_intersection.h"
#include <thrust/sequence.h>
#include <iostream>
#include <pcl/cuda/time_cpu.h>
#include <thrust/device_ptr.h>

namespace realtime_perception 
{
  BoxListRayIntersection::BoxListRayIntersection (float3* min, float3* max, int numboxes, float3 origin, float focallength, int width, int height) 
    : min (min), max (max), numboxes (numboxes), origin (origin), focallength (focallength), width (width), height (height)
  {}

  template <typename Tuple> __inline__ __host__ __device__
    int BoxListRayIntersection::operator() (const Tuple &t) const
  {
    Intersection is = intersect_boxes (thrust::get<0>(t));
    if (is.tmin == 0)
    {
      return 0;
    }
    else
    {
      return is.boxid + 1;
    }
  }

  BoxListRayIntersection::Intersection BoxListRayIntersection::intersect_boxes (int pt_index) const
  {
    Intersection ret;
    float min_tmin = ret.tmin = 0;

    for (int i = 0; i < numboxes; i++)
    {
      float3 ray_direction;
      ray_direction.x = pt_index;
      ray_direction.y = pt_index;
      ray_direction.z = pt_index;
      // TODO: compute ray direction from index
      float tmin = hitbox (i, ray_direction);
      if (tmin < min_tmin)
      {
        min_tmin = tmin;
        ret.tmin = tmin;
        ret.boxid = i;
      }
    }
    return ret;
  }

  float BoxListRayIntersection::hitbox (int index, float3 ray_direction) const
  {
    float3 &m1 = min[index];
    float3 &m2 = max[index];

    float tymin, tymax, tzmin, tzmax, tmin, tmax; 
    float flag = 1.0; 
  
     if (ray_direction.x >= 0) 
     {
        tmin = (m1.x - origin.x) / ray_direction.x;
        tmax = (m2.x - origin.x) / ray_direction.x;
     }
     else 
     {
        tmin = (m2.x - origin.x) / ray_direction.x;
        tmax = (m1.x - origin.x) / ray_direction.x;
     }
     if (ray_direction.y >= 0) 
     {
        tymin = (m1.y - origin.y) / ray_direction.y; 
        tymax = (m2.y - origin.y) / ray_direction.y; 
     }
     else 
     {
        tymin = (m2.y - origin.y) / ray_direction.y; 
        tymax = (m1.y - origin.y) / ray_direction.y; 
     }
      
     if ((tmin > tymax) || (tymin > tmax)) flag = -1.0; 
     if (tymin > tmin) tmin = tymin; 
     if (tymax < tmax) tmax = tymax; 
       
     if (ray_direction.z >= 0) 
     {
        tzmin = (m1.z - origin.z) / ray_direction.z; 
        tzmax = (m2.z - origin.z) / ray_direction.z; 
     }
     else 
     {
        tzmin = (m2.z - origin.z) / ray_direction.z; 
        tzmax = (m1.z - origin.z) / ray_direction.z; 
     }
     if ((tmin > tzmax) || (tzmin > tmax)) flag = -1.0; 
     if (tzmin > tmin) tmin = tzmin; 
     if (tzmax < tmax) tmax = tzmax; 
       
     return tmin;
  }

  template <template <typename> class Storage> void
  BoxListRayIntersection::BoxListRayIntersection (float3* min, float3* max, int numboxes, float3 origin, float focallength, int width, int height) 
  URDF::compute (const boost::shared_ptr<openni_wrapper::DepthImage>& depth_image,
                             const boost::shared_ptr<openni_wrapper::Image>& rgb_image,
                             float constant,
                             typename PointCloudAOS<Storage>::Ptr &output,
                             bool downsample, int stride, int smoothing_nr_iterations, int smoothing_filter_size) 
  {
  thrust::transform (make_zip_iterator (make_tuple (counting_iterator<int>(0))), 
                     make_zip_iterator (make_tuple (counting_iterator<int>(0))) + output->width *output->height,
                     output->point.begin (),
                     BoxListRayIntersection (min, max, numboxes, origin, focallength, output->width, output->height));
  }

};

