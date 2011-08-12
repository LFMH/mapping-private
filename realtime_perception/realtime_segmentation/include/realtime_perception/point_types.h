// Author: Nico Blodow <blodow@cs.tum.edu>, 2011
// BSD-License

#ifndef __REALTIME_PERCEPTION_POINT_TYPES_H
#define __REALTIME_PERCEPTION_POINT_TYPES_H

#include "pcl/ros/register_point_struct.h"

struct PointXYZRGBNormalRegion
{
  PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
  PCL_ADD_NORMAL4D;
  union
  {
    struct
    {
      float rgb;
      float curvature;
    };
    float data_c[4];
  };
  int region;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZRGBNormalRegion,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, rgb, rgb)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z)
    (float, curvature, curvature)
    (int,   region, region)
)
#endif
