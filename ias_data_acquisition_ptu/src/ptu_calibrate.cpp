/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
  * \author Romain Thibaux, Radu Bogdan Rusu 
  *
  * @b ptu_calibrate attempts to estimate pan-tilt calibration values as ROS
  * parameters based on point cloud planar segmentation.
  */
// ROS core
#include <ros/ros.h>
// Messages
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
// PCL stuff
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ros/conversions.h>
#include "pcl/sample_consensus/method_types.h"
#include "pcl/sample_consensus/model_types.h"
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/surface/convex_hull.h>

#include <pcl_ros/publisher.h>

//#include <Eigen3/LeastSquares>
//#include <Eigen3/Geometry/Hyperplane>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <dp_ptu47_pan_tilt_stage/PanTiltStamped.h>

typedef pcl::PointXYZ Point;
typedef pcl::PointCloud<Point> PointCloud;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud::ConstPtr PointCloudConstPtr;

// TODO: in the future we should auto-detect the wall, or detect the location of the only
//       moving object, the table
// Equation of a boundary between the table and the wall, in base_link frame
// 'wp' stands for 'wall protection'
// Points on the plane satisfy wp_normal.dot(x) + wp_offset == 0
const tf::Vector3 wp_normal(1, 0, 0);
const double wp_offset = -1.45;

// Waits for a point cloud with pan-tilt close to (0,0) and deduces the position of ptu_base
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class PTUCalibrator 
{
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, dp_ptu47_pan_tilt_stage::PanTiltStamped> SyncPolicy;

  public:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    PTUCalibrator (const ros::NodeHandle &nh) : nh_ (nh), stem_height_ (0), head_to_ptu_valid_ (false), 
                                                cloud_pantilt_sync_ (SyncPolicy (20)), sac_distance_ (0.03)
    {
      cloud_pub_.advertise (nh_, "table_inliers", 1);
      //nh_.getParam("ptu/stem_height", stem_height_);  // Initial guess, which will be refined
      //nh_.getParam("ptu/table_size", table_size_);
      //printf("Read table size: %f\n", table_size_);
      
      vgrid_.setFilterFieldName ("z");
      vgrid_.setFilterLimits (0, 2.0);
      vgrid_.setLeafSize (.01, .01, .01);

      seg_.setOptimizeCoefficients (true);
      seg_.setModelType (pcl::SACMODEL_PLANE);
      seg_.setMethodType (pcl::SAC_RANSAC);
      seg_.setDistanceThreshold (sac_distance_);

      proj_.setModelType (pcl::SACMODEL_PLANE);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    virtual ~PTUCalibrator () 
    {
      for (size_t i = 0; i < table_coeffs_.size (); ++i) 
        delete table_coeffs_[i];
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      init (double tolerance)  // tolerance: how close to (0,0) is good enough?
    {
      std::string point_cloud_topic = nh_.resolveName ("/camera/depth/points2");
      std::string pan_tilt_topic = nh_.resolveName ("/dp_ptu47/pan_tilt_status_stamped");

      // TODO?: Initialize the /ptu/base_link frame with a weak prior centered around a
      // PI/2 rotation about z relative to /base_link
      real_part_ = 0;
      imaginary_part_ = 0;

      // To find the transform from base_link to /ptu/base_link
      point_cloud_sub_.subscribe (nh_, point_cloud_topic, 20);
      pan_tilt_sub_.subscribe (nh_, pan_tilt_topic, 20);
      cloud_pantilt_sync_.connectInput (point_cloud_sub_, pan_tilt_sub_);
      cloud_pantilt_sync_.registerCallback (boost::bind (&PTUCalibrator::ptuFinderCallback, this, _1, _2));

      marker_publisher_ = nh_.advertise<visualization_msgs::Marker>("/ptu_calibration_table", 0);

//       ROS_INFO ("[%s] Creating tf timer", getName ().c_str ());
//       timer_  = nh_.createTimer (ros::Duration (0.1), boost::bind (&PTUCalibrator::timerCallback, this, _1));  // Call it every 0.1s after that
    }
    
  private:
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      ptuFinderCallback (const sensor_msgs::PointCloud2ConstPtr &cloud_in, const dp_ptu47_pan_tilt_stage::PanTiltStampedConstPtr &pan_tilt)
    {
      ROS_INFO_STREAM ("[" << getName ().c_str () << "] Received pair: cloud time " << cloud_in->header.stamp << ", pan-tilt " << pan_tilt->pan_angle << "/" << pan_tilt->tilt_angle << " time " << pan_tilt->header.stamp);

//       // Transform the wall protection equation to narrow_stereo_optical_frame
//       tf::Stamped<tf::Vector3> wp_normal_optical (tf::Vector3 (0, 0, 0), cloud_in->header.stamp, "/base_link");
//       double wp_offset_optical (0);
//       if (!tf_listener_.waitForTransform ("/narrow_stereo_optical_frame", cloud_in->header.frame_id, cloud_in->header.stamp, ros::Duration (1.0)))
//         ROS_WARN ("[%s] Could not transform table/wall boundary equation to /narrow_stereo_optical_frame", getName ().c_str ());
//       else 
//       {
//         const tf::Stamped<tf::Vector3> wp_normal_base_link (wp_normal, cloud_in->header.stamp, "/base_link");
//         tf_listener_.transformVector ("/narrow_stereo_optical_frame", wp_normal_base_link, wp_normal_optical);
//         const tf::Stamped<tf::Point> base_link_origin_base_link (tf::Point (0, 0, 0), cloud_in->header.stamp, "/base_link");
//         tf::Stamped<tf::Point> base_link_origin_optical;
//         tf_listener_.transformPoint ("/narrow_stereo_optical_frame", base_link_origin_base_link, base_link_origin_optical);
//         wp_offset_optical = wp_offset - wp_normal_optical.dot (base_link_origin_optical);
//       }

//       // Downsample + filter the input dataser
//       PointCloud cloud_raw, cloud;
//       pcl::fromROSMsg (*cloud_in, cloud_raw);
//       vgrid_.setInputCloud (boost::make_shared<PointCloud> (cloud_raw));
//       vgrid_.filter (cloud);

//       // Remove points from the wall (?)
//       pcl::PointIndices selection;
//       for (size_t i = 0; i < cloud.points.size (); ++i) 
//       {
//         const Point &p = cloud.points[i];
//         if (wp_offset_optical + wp_normal_optical.x () * p.x + wp_normal_optical.y () * p.y + wp_normal_optical.z () * p.z > 0) 
//           // This point is beyond the wall protection boundary, it probably does not belong to the table
//           continue;
//         selection.indices.push_back (i);
//       }

//       // Fit a plane (the table)
//       pcl::ModelCoefficients table_coeff;
//       pcl::PointIndices table_inliers;
//       seg_.setInputCloud (boost::make_shared<PointCloud> (cloud));
//       seg_.setIndices (boost::make_shared<pcl::PointIndices> (selection));
//       seg_.segment (table_inliers, table_coeff);
//       ROS_INFO ("[%s] Table model: [%f, %f, %f, %f] with %d inliers.", getName ().c_str (), 
//                 table_coeff.values[0], table_coeff.values[1], table_coeff.values[2], table_coeff.values[3], (int)table_inliers.indices.size ());

//       // Project the table inliers using the planar model coefficients
//       PointCloud cloud_projected;
//       proj_.setInputCloud (boost::make_shared<PointCloud> (cloud));
//       proj_.setIndices (boost::make_shared<pcl::PointIndices> (table_inliers));
//       proj_.setModelCoefficients (boost::make_shared<pcl::ModelCoefficients> (table_coeff));
//       proj_.filter (cloud_projected);
//       cloud_pub_.publish (cloud_projected);
      
//       // Re-orient the plane towards up
//       //ROS_INFO("Reorienting plane");
//       const tf::Stamped<tf::Vector3> vertical_base_link (tf::Vector3 (0, 0, 1), cloud.header.stamp, "/base_link");
//       tf::Stamped<tf::Vector3> vertical_optical;
//       tf_listener_.transformVector ("/narrow_stereo_optical_frame", vertical_base_link, vertical_optical);
//       double cosToVertical = table_coeff.values[0] * vertical_optical.x () + table_coeff.values[1] * vertical_optical.y () + table_coeff.values[2] * vertical_optical.z ();
//       if (cosToVertical < 0)
//       {
//         for (int i = 0; i < 4; ++i)
//           table_coeff.values[i] *= -1;
//         cosToVertical = -cosToVertical;
//       }
//       double measured_tilt = acos (cosToVertical);
//       if ( (abs (measured_tilt - pan_tilt->tilt_angle * M_PI / 180.0) > (M_PI / 16.0)) &&
//            (abs (measured_tilt + pan_tilt->tilt_angle * M_PI / 180.0) > (M_PI / 16.0)) )
//       {
//         ROS_INFO_STREAM("Table plane (" << measured_tilt << ") not close enough to expected tilt (" << pan_tilt->tilt_angle * M_PI / 180.0 << "), discarded");
//         return;
//       }

//       // Is the plane oriented towards the camera?
//       if (table_coeff.values[2] >= 0) 
//       {
//         // Oriented away from the camera, its detection will be very unstable
//         ROS_INFO ("Table plane pointing away from the camera");
//         return;
//       }

//       // Reconstruct the 2D Convex Hull of the table
//       PointCloud cloud_hull;
//       chull_.setInputCloud (boost::make_shared<PointCloud> (cloud_projected));
//       chull_.reconstruct (cloud_hull);
      
//       visualization_msgs::Marker marker;
//       marker.header = cloud.header;
//       //marker.header.stamp = ros::Time::now ();
//       marker.ns = "plane_detection";
//       marker.id = 1;
//       marker.type = visualization_msgs::Marker::LINE_STRIP;
//       marker.action = visualization_msgs::Marker::ADD;
//       marker.pose.position.x = 0;
//       marker.pose.position.y = 0;
//       marker.pose.position.z = 0;
//       marker.pose.orientation.x = 0.0;
//       marker.pose.orientation.y = 0.0;
//       marker.pose.orientation.z = 0.0;
//       marker.pose.orientation.w = 1.0;
//       marker.scale.x = 0.005;
//       marker.color.r = 1.0f;
//       marker.color.g = 1.0f;
//       marker.color.b = 0.0f;
//       marker.color.a = 1.0;
//       marker.lifetime = ros::Duration (10);
//       for (unsigned int i = 0; i < cloud_hull.points.size (); ++i) 
//       {
//         geometry_msgs::Point p;
//         p.x = cloud_hull.points[i].x;
//         p.y = cloud_hull.points[i].y;
//         p.z = cloud_hull.points[i].z;
//         marker.points.push_back (p);
//       }
//       marker_publisher_.publish(marker);
          
//       // Transform the plane equation into the base_link frame
//       //ROS_INFO("Converting the plane to base_link");
//       const tf::Stamped<tf::Vector3> table_normal_optical (tf::Vector3 (table_coeff.values[0], table_coeff.values[1], table_coeff.values[2]),
//                                                            cloud.header.stamp, "/narrow_stereo_optical_frame");
//       tf::Stamped<tf::Vector3> table_normal_base_link;
//       tf_listener_.transformVector("/base_link", table_normal_optical, table_normal_base_link);
//       const tf::Stamped<tf::Point> optical_origin_optical (tf::Point (0, 0, 0), cloud.header.stamp, "/narrow_stereo_optical_frame");
//       tf::Stamped<tf::Point> optical_origin_base_link;
//       tf_listener_.transformPoint ("/base_link", optical_origin_optical, optical_origin_base_link);
//       double table_offset_base_link = table_coeff.values[3] - table_normal_base_link.dot (optical_origin_base_link);

//       // Store this data point
//       //ROS_INFO("Storing a data point");
//       table_coeffs_.push_back (new Eigen3::Vector4d (table_normal_base_link.x (), table_normal_base_link.y (), table_normal_base_link.z (), -table_offset_base_link));
//       pan_tilts_.push_back (pan_tilt);

//       updateCalibration();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//     void 
//       timerCallback(const ros::TimerEvent &timer_event)
//     {
//       if (!head_to_ptu_valid_)
//         return;
//       // INVARIANT: The transform has been initialized
//       tf::StampedTransform head_to_ptu_stamped (head_to_ptu_, timer_event.current_expected, "/base_link", "/ptu/base_link");
//       transform_broadcaster_.sendTransform (head_to_ptu_stamped);
//       tf_listener_.setTransform (head_to_ptu_stamped, "Timer");  // Make it immediately available to the listener
//     }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      updateCalibration () 
    {
      //show_data();
      if (table_coeffs_.size () <= 1) 
      {
        ROS_INFO ("[%s] Not enough data points to calibrate", getName ().c_str ());
        return;
      }
      // Check whether the system is degenerate (we do not have enough data)
      double best_deviation = 0;
      const Eigen3::Vector3d &table_normal0 = table_coeffs_[0]->block<3,1> (0, 0);
      for (size_t i = 1; i < table_coeffs_.size (); ++i) 
      {
        const Eigen3::Vector3d &table_normali = table_coeffs_[i]->block<3,1> (0, 0);
        double c = table_normal0.dot (table_normali);  // cosine
        if (c > 1) 
        {
          ROS_ERROR_STREAM ("c = " << c << " for data point " << i);
          //return;
          exit (-1);
        }
        double deviation = 1 - c*c;  // sine squared
        if (deviation > best_deviation)
          best_deviation = deviation;
      }
      if (best_deviation < 0.2)
      {
        ROS_INFO_STREAM ("[" << getName ().c_str () << "] Not enough variability in the data points to calibrate (deviation " << best_deviation << ")");
        return;
      }

      // Calibrate the /ptu/base_link center and the stem height
      //TODO: I commented this out until we find an alternative for seemingly deprecated linearRegression function
      Eigen3::Vector4d x;
      Eigen3::Vector4d *x_ptr = &x;
      Eigen3::Vector4d **data_array = &(table_coeffs_[0]);
      //      Eigen3::linearRegression((int) table_coeffs_.size (), data_array, x_ptr, 3);
      // x[0,1,2] is now the center of rotation of the table
      // and x[3] is the distance stem height
      stem_height_ = x[3];
      //ROS_INFO_STREAM("Found position [" << x[0] << ", " << x[1] << ", " << x[2] << "] and stem height " << x[3]);

      // Calibrate the pan angle (find the 0 pan position)
      // This for loop is unnecessary, but for debugging it is nice to see the whole
      // calculation being redone every time
      real_part_ = 0;
      imaginary_part_ = 0;
      for (size_t i = 0; i < table_coeffs_.size (); ++i) 
      {
        const Eigen3::Vector4d &coeff = *table_coeffs_[i];
        double pan = pan_tilts_[i]->pan_angle * M_PI / 180.0;
        double tilt = pan_tilts_[i]->tilt_angle * M_PI / 180.0;
        if (tilt < 0)
          pan = pan + M_PI;
        // The pan angle determines the position of the x axis of /ptu/turret
        // but the normal of the plane (our data) is oriented towards -y of /ptu/turret
        // (or +y if the tilt is negative, hence the PI radians rotation above)
        // So to obtain the angle of the normal in /ptu/turret, we must
        // subtract PI/2 to get from x to -y
        double u = pan - M_PI / 2;
        real_part_ += coeff[0] * cos (u) + coeff[1] * sin (u);
        imaginary_part_ += coeff[1] * cos (u) - coeff[0] * sin (u);
      }

      double modulus = sqrt (real_part_*real_part_ + imaginary_part_*imaginary_part_);
      if (modulus / stem_height_ < 0.2)
        // modulus / stem_height_ is a measure of confidence in the angle determination
        return;

      double pan_offset = atan2 (imaginary_part_, real_part_);
      //ROS_INFO_STREAM("Found pan offset " << pan_offset);
      // pan_offset is the amount of rotation about z that /ptu/base_link is rotated
      // with respect to /base_link. In the original setup of the table, this was
      // very close to PI/2

      // Generate /narrow_stereo_optical_frame --> /ptu/base_link transform
      tf::Point rotation_center (x[0], x[1], x[2]);
      tf::Quaternion rotation (0, 0, sin (pan_offset / 2), cos (pan_offset / 2));
      head_to_ptu_.setOrigin (rotation_center);
      head_to_ptu_.setRotation (rotation);
      head_to_ptu_valid_ = true;

      nh_.setParam ("ptu/center/x", rotation_center[0]);
      nh_.setParam ("ptu/center/y", rotation_center[1]);
      nh_.setParam ("ptu/center/z", rotation_center[2]);
      nh_.setParam ("ptu/rotation/x", rotation[0]);
      nh_.setParam ("ptu/rotation/y", rotation[1]);
      nh_.setParam ("ptu/rotation/z", rotation[2]);
      nh_.setParam ("ptu/rotation/w", rotation[3]);
      nh_.setParam ("ptu/stem_height", stem_height_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void 
      show_data () const
    {
      ROS_INFO ("Data stored:");
      for (size_t i = 0; i < table_coeffs_.size (); ++i)
        ROS_INFO_STREAM ("[" << (*table_coeffs_[i])[0] << ", " << (*table_coeffs_[i])[1] << ", " << (*table_coeffs_[i])[2] << "]");
    }

    ros::NodeHandle nh_;  // Do we need to keep it?
    tf::TransformBroadcaster transform_broadcaster_;
    tf::TransformListener tf_listener_;
    double stem_height_;  // Distance from the center of rotation to the center of the table
    //double table_size_;  // Size of one side of the table (the table is square)
    bool save_to_files_;

    // Parameters related to the location of the pan tilt unit base
    ros::Timer timer_;
    double real_part_;
    double imaginary_part_;
    tf::Transform head_to_ptu_;
    bool head_to_ptu_valid_;
   
   

    message_filters::Subscriber<sensor_msgs::PointCloud2> point_cloud_sub_;
    message_filters::Subscriber<dp_ptu47_pan_tilt_stage::PanTiltStamped> pan_tilt_sub_;
    message_filters::Synchronizer<SyncPolicy> cloud_pantilt_sync_;

    std::vector<Eigen3::Vector4d *> table_coeffs_;
    std::vector<dp_ptu47_pan_tilt_stage::PanTiltStampedConstPtr> pan_tilts_;

    ros::Publisher marker_publisher_;
    pcl_ros::Publisher<Point> cloud_pub_;

    // PCL objects
    pcl::VoxelGrid<Point> vgrid_;                   // Filtering + downsampling object
    pcl::SACSegmentation<Point> seg_;               // Planar segmentation object
    pcl::ProjectInliers<Point> proj_;               // Inlier projection object
  //    pcl::ConvexHull2D<Point, Point> chull_;         // 2D convex hull estimation object
    double sac_distance_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Get a string representation of the name of this class. */
    std::string getName () const { return ("PTUCalibrator"); }
};




/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "extract_single_object_cluster");
  ros::NodeHandle nh;
  PTUCalibrator ptu_calibrator (nh);
  ptu_calibrator.init (5);  // 5 degrees tolerance
  ros::spin ();
}
/* ]--- */
