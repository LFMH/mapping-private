#ifndef REALTIME_PERCEPTION_BOXLIST_RAY_INTERSECTION_H_
#define REALTIME_PERCEPTION_BOXLIST_RAY_INTERSECTION_H_

namespace realtime_perception
{
  struct BoxListRayIntersection
  {
    // helper struct for data passing
    typedef struct 
    {
      int boxid;
      int tmin;
    } Intersection;

    // box list
    float3* min;
    float3* max;
    int numboxes;

    // sensor / depth image description
    float3 origin;
    float focallength;
    int width;
    int height;
    
    BoxListRayIntersection (float3* min, float3* max, int numboxes, float3 origin, float focallength, int width, int height); 
    
    template <typename Tuple> __inline__ __host__ __device__
      int operator() (const Tuple &t) const;

    __inline__ __host__ __device__
      Intersection intersect_boxes (int pt_index) const;

    __inline__ __host__ __device__
      float hitbox (int index, float3 ray_direction) const;
  //Ray r, vec3 m1, vec3 m2, out float tmin, out float tmax) 
  };

} // namespace

#endif
