#ifndef DOS_PCL_ROS_CLOUD_COLLISION_H_
#define DOS_PCL_ROS_CLOUD_COLLISION_H_

#include <boost/shared_ptr.hpp>
#include <urdf/model.h>
#include <pcl/pcl_base.h>

namespace dos_pcl_ros
{

template <typename PointT>
class CloudCollision
{
  public:
  typedef boost::shared_ptr<const pcl::PointCloud<PointT> > PointCloudConstPtr;
  virtual int check (const boost::shared_ptr<urdf::Collision> &collision, const urdf::Vector3 &offset, std::map<std::string, XmlRpc::XmlRpcValue> params, const PointCloudConstPtr &cloud, std::vector<int> remaining_indices, std::vector<int> &indices, double threshold, std::vector<int> &indices_expand) = 0;
};

template <typename PointT>
class CloudBoxCollision : virtual public CloudCollision <PointT>
{
  public: 
  typedef typename CloudCollision<PointT>::PointCloudConstPtr PointCloudConstPtr;
  int check (const boost::shared_ptr<urdf::Collision> &collision, const urdf::Vector3 &offset, std::map<std::string, XmlRpc::XmlRpcValue> params, const PointCloudConstPtr &cloud, std::vector<int> remaining_indices, std::vector<int> &indices, double threshold, std::vector<int> &indices_expand)
  {
    if (!collision)
      return 0;

    boost::shared_ptr<urdf::Geometry> geom = collision->geometry;
    boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast <urdf::Box> (geom);   

    threshold = params["threshold"];

    double min_x = offset.x + collision->origin.position.x - (box->dim.x*0.5) - threshold;
    double min_y = offset.y + collision->origin.position.y - (box->dim.y*0.5) - threshold;
    double min_z = offset.z + collision->origin.position.z - (box->dim.z*0.5) - threshold;
    double max_x = offset.x + collision->origin.position.x + (box->dim.x*0.5) + threshold;
    double max_y = offset.y + collision->origin.position.y + (box->dim.y*0.5) + threshold;
    double max_z = offset.z + collision->origin.position.z + (box->dim.z*0.5) + threshold;
    if (params.count ("operation") > 0)
    {
      if (std::string(params["operation"]) == "segment_objects"
       || std::string(params["operation"]) == "fit_door"
       || std::string(params["operation"]) == "fit_drawer")
      {
        ROS_ASSERT (params.count ("search_expand_axis") > 0);
        ROS_ASSERT (params.count ("search_expand_distance") > 0);
        double axis_x = params["search_expand_axis"][0];
        double axis_y = params["search_expand_axis"][1];
        double axis_z = params["search_expand_axis"][2];
        double search_expand_distance = params["search_expand_distance"];

        double min_x_expand, min_y_expand, min_z_expand;
        double max_x_expand, max_y_expand, max_z_expand;

        min_x_expand = (axis_x < 0.0)? min_x + axis_x * search_expand_distance: min_x;
        min_y_expand = (axis_y < 0.0)? min_y + axis_y * search_expand_distance: min_y;
        min_z_expand = (axis_z < 0.0)? min_z + axis_z * search_expand_distance: min_z;
        max_x_expand = (axis_x > 0.0)? max_x + axis_x * search_expand_distance: max_x;
        max_y_expand = (axis_y > 0.0)? max_y + axis_y * search_expand_distance: max_y;
        max_z_expand = (axis_z > 0.0)? max_z + axis_z * search_expand_distance: max_z;

        ROS_INFO ("min, max expand: %f %f %f %f %f %f",
            min_x_expand, min_y_expand, min_z_expand,
            max_x_expand, max_y_expand, max_z_expand);

        for (unsigned int cp = 0; cp < remaining_indices.size (); cp++)
        {
          PointT p = cloud->points[remaining_indices[cp]];

          // Check if the point is invalid
          if (isnan (p.x) || isnan (p.y) || isnan (p.z))
            continue;
          if ((p.x >= min_x && p.x <= max_x) &&
              (p.y >= min_y && p.y <= max_y) &&
              (p.z >= min_z && p.z <= max_z))
            indices.push_back (cp);
          if ((p.x >= min_x_expand && p.x <= max_x_expand) &&
                   (p.y >= min_y_expand && p.y <= max_y_expand) &&
                   (p.z >= min_z_expand && p.z <= max_z_expand))
            indices_expand.push_back (cp);
        }
      }
      else
      {
        if (std::string (params["operation"]) == "delete" && cloud)
        {
          for (unsigned int cp = 0; cp < cloud->points.size (); cp++)
          {
            PointT p = cloud->points[cp];

            // Check if the point is invalid
            if (isnan (p.x) || isnan (p.y) || isnan (p.z))
            {
              indices.push_back (cp);
              continue;
            }

            if ((p.x >= min_x && p.x <= max_x) &&
                (p.y >= min_y && p.y <= max_y) &&
                (p.z >= min_z && p.z <= max_z))
              indices.push_back (cp);
          }
        }
      }
      std::cerr << "performed" << std::string (params["operation"]) << " -- indices: " << indices.size() << " -- indices_expand: " << indices_expand.size() << std::endl;
    }
    else
    {
      for (unsigned int cp = 0; cp < remaining_indices.size (); cp++)
      {
        PointT p = cloud->points[remaining_indices[cp]];

        // Check if the point is invalid
        if (isnan (p.x) || isnan (p.y) || isnan (p.z))
          continue;

        if ((p.x >= min_x && p.x <= max_x) &&
            (p.y >= min_y && p.y <= max_y) &&
            (p.z >= min_z && p.z <= max_z))
          indices.push_back (cp);
      }
    }

    return 0;
  }
};
}

#endif

