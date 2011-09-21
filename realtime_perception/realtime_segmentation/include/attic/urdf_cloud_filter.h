#ifndef REALTIME_PERCEPTION_URDF_CLOUD_FILTER_H_
#define REALTIME_PERCEPTION_URDF_CLOUD_FILTER_H_

#include "pcl_ros/pcl_nodelet.h"
#include "pcl/ros/conversions.h"
#include "pcl_ros/filters/filter.h"
#include "pcl_ros/transforms.h"
#include "pcl/point_types.h"
#include "pcl_ros/publisher.h"
#include "urdf/model.h"

#include <pcl/sample_consensus/sac.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model.h>
#include <pcl/sample_consensus/sac_model_perpendicular_plane.h>

#include "common.h"
#include "cloud_collision.h"

// boost
#include <XmlRpcException.h>
#include <boost/regex.hpp>  // Boost.Regex lib

// messages
#include <sensor_msgs/JointState.h>

namespace realtime_perception
{
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief @b URDFCloudFilter filters points that are coincident with a user-specified part of a URDF description
    * \author Nico Blodow
    */

  template <typename PointT>
    class URDFCloudFilter
  {
    typedef sensor_msgs::PointCloud2 PointCloud2;
    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
    typedef typename pcl::SampleConsensusModelPerpendicularPlane<PointT>::Ptr SampleConsensusModelPerpendicularPlanePtr;

//    typedef PointCloudAOS<Storage> PointCloud;
//    typedef typename PointCloud::Ptr PointCloudPtr;
//    typedef typename PointCloud::ConstPtr PointCloudConstPtr;
//
//    typedef typename Storage<int>::type Indices;
//    typedef boost::shared_ptr<typename Storage<int>::type> IndicesPtr;
//    typedef boost::shared_ptr<const typename Storage<int>::type> IndicesConstPtr;

    public:
      // stuff needed to set up the search regions
      typedef std::vector<boost::shared_ptr<urdf::Link> > LinkVector;

      typedef struct SearchOperation_ {
        boost::regex re;                                    // what links are we interested in?
        std::string pub_topic_re;                           // do we want to publish something?
        std::string pub_topic;                              // do we want to publish something?
        std::map <std::string, XmlRpc::XmlRpcValue> params; // other params of this operation

        // less compare operator to make sure delete operations are first
        bool operator< (const struct SearchOperation_ &rhs) const
        {
          std::map <std::string, XmlRpc::XmlRpcValue>::const_iterator it = params.find ("operation");
          if (it == params.end())
            return false;
          std::map <std::string, XmlRpc::XmlRpcValue>::const_iterator it_rhs = rhs.params.find ("operation");
          if (it_rhs->second == std::string("delete") && it_rhs->second == std::string("delete"))
            return false;
          return (it->second == std::string("delete"));
        }
      } SearchOperation;

      typedef struct TargetFrames_{
        std::string frame;                  // what frame do we transform the PC in?
        SearchOperation op;                 // what are we looking for?
        urdf::Vector3 translation;          // what are we looking for?
        boost::shared_ptr<urdf::Link> link; // what links do we focus on?

        // less compare operator to make sure delete operations are first
        bool operator< (const struct TargetFrames_ &rhs) const
          { return op < rhs.op; }
      } TargetFrames;

    public:
      //void filter (const PointCloudConstPtr &input, const IndicesConstPtr &indices, PointCloud &output);

      void onInit (ros::NodeHandle &nh);

    private:
      void loadParams (ros::NodeHandle &nh);
      void initURDFModel (ros::NodeHandle &nh);
      void loadURDFModel (TiXmlElement* root_element, urdf::Model &descr, ros::NodeHandle &nh, bool visual = true, bool collision = true);

      bool walk_back_links (const boost::shared_ptr<urdf::Link> l,
                            const boost::shared_ptr<urdf::Link> original_l,
                            const SearchOperation sop,
                            const std::string stop,
                                  urdf::Vector3 &offset);

      void perform_delete (typename std::vector<TargetFrames>::iterator &it,
                           std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds,
                           std::vector<int> &indices_to_be_deleted);

      void perform_fit_drawer (typename std::vector<TargetFrames>::iterator &it,
                           std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds);

      void perform_fit_door (typename std::vector<TargetFrames>::iterator &it,
                           std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds);

      void perform_segment_objects (typename std::vector<TargetFrames>::iterator &it,
                           std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds,
                           std::vector<int> &indices_expand);

//      double computePrincipalComponentAngle (const PointCloud &cloud, Eigen::Vector4f &axis);
//
//      void principalComponent
//          (const Eigen::Matrix3f &covariance_matrix, const Eigen::Vector4f &point,
//           Eigen::Vector4f &principal_component);

      std::vector <int> remaining_indices_;

      std::string stop_link_;
      std::string description_param_;
      std::string model_description_;
      std::string tf_prefix_;
      double wait_for_tf_; 
      bool use_indices_;

    public:
      std::vector <TargetFrames> target_frames_;
      std::vector <SearchOperation> search_operations_;

    private:
      // collision check
      boost::shared_ptr<realtime_perception::CloudBoxCollision<PointT> > box_collision_;

      ros::Publisher joint_states_pub_;
      std::map <std::string, pcl_ros::Publisher<pcl::PointXYZ> > publishers_;

      double threshold_;
      //DEPRECATED pcl_ros::Publisher <typename pcl::PointXYZ> pub_output_;
  };
}

#include "impl/urdf_cloud_filter.hpp"

#endif  //#ifndef REALTIME_PERCEPTION_URDF_CLOUD_FILTER_H_

