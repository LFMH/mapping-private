/*
 * Copyright (c) 2009 Dejan Pangercic <pangercic -=- cs.tum.edu>
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: detect_planes.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
   @mainpage

   @htmlinclude manifest.html

   @detect_planes detects oriented planes in point cloud data.

**/

// ROS core
#include <ros/node.h>
#include <ros/node_handle.h>
// ROS messages
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>

// Sample Consensus
#include <point_cloud_mapping/sample_consensus/sac.h>
#include <point_cloud_mapping/sample_consensus/msac.h>
#include <point_cloud_mapping/sample_consensus/ransac.h>
#include <point_cloud_mapping/sample_consensus/sac_model_plane.h>

#include <angles/angles.h>

// Kd Tree
#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// Cloud geometry
#include <point_cloud_mapping/geometry/angles.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/point.h>
#include <point_cloud_mapping/geometry/distances.h>
#include <point_cloud_mapping/geometry/nearest.h>
#include <point_cloud_mapping/geometry/transforms.h>
#include <point_cloud_mapping/geometry/statistics.h>

#include <sys/time.h>
#include <mapping_srvs/GetPlaneClusters.h>

#define DEBUG 1
//#define DEBUG_D false

using namespace std;
using namespace ros;
using namespace sensor_msgs;
using namespace geometry_msgs;
using namespace mapping_msgs;
using namespace mapping_srvs;

// Comparison operator for a vector of vectors
bool
compareRegions (const std::vector<int> &a, const std::vector<int> &b)
{
  return (a.size () < b.size ());
}

class DetectPlanes
{
protected:
  ros::NodeHandle nh_;
public:

  // ROS messages
  PointCloudConstPtr cloud_in_;

  PointCloud cloud_down_;
  PointCloud cloud_annotated_;
  Point32 axis_;

  // Parameters
  string input_cloud_topic_;
  int k_;
  double max_z_;
  int clusters_min_pts_;

  bool need_cloud_data_;

  double sac_distance_threshold_, eps_angle_;

  double delta_z_, object_min_dist_from_table_;

  double min_angle_, max_angle_;

  Subscriber cloud_sub_;
  Publisher cloud_table_pub_, pmap_pub_, cloud_filtered_pub_;
  ServiceServer detect_planes_service_;

  int downsample_factor_;
  ServiceServer get_detect_planes_service_;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //Detects one dominant plane in a point cloud
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  DetectPlanes ()
  {
    // 0.198669 0 0.980067 0 0 -1 0 0 0.980067 0 -0.198669 0 0 0 0 1
    axis_.x = 0; axis_.y = 1; axis_.z = 0;

    nh_.param ("~downsample_factor", downsample_factor_, 4); // Use every nth point
    nh_.param ("~search_k_closest", k_, 2);                  // 5 k-neighbors by default
    nh_.param ("~k_max_z", max_z_, 0.03);

    nh_.param ("~normal_eps_angle", eps_angle_, 15.0);       // 15 degrees
    eps_angle_ = angles::from_degrees (eps_angle_);          // convert to radians

    nh_.param ("~clusters_min_pts", clusters_min_pts_, 10);  // 10 points
    
    nh_.param ("~table_delta_z", delta_z_, 0.03);                              // consider objects starting at 3cm from the table
            
    nh_.param ("~filtering_min_angle", min_angle_, 10.0);
    nh_.param ("~filtering_max_angle", max_angle_, 170.0);

    nh_.param ("~input_cloud_topic", input_cloud_topic_, string ("/cloud_pcd"));
    detect_planes_service_ = nh_.advertiseService("/detect_planes_service", &DetectPlanes::detect_planes_service, this);

    // This should be set to whatever the leaf_width factor is in the downsampler
    nh_.param ("~sac_distance_threshold", sac_distance_threshold_, 0.01);     // 3 cm

    cloud_table_pub_ = nh_.advertise<PointCloud> ("cloud_table", 1);
    pmap_pub_ = nh_.advertise<PolygonalMap> ("pmap", 1);
    get_detect_planes_service_ = nh_.advertiseService("get_detect_planes_service", &DetectPlanes::detect_planes_service, this);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  updateParametersFromServer ()
  {
    nh_.getParam ("~input_cloud_topic", input_cloud_topic_);
    nh_.getParam ("~downsample_factor", downsample_factor_);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  detect_planes_service (GetPlaneClusters::Request &req, GetPlaneClusters::Response &resp)
  {
    ROS_INFO ("Service request initiated.");
    updateParametersFromServer ();

    // Subscribe to a point cloud topic
    need_cloud_data_ = true;
    cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &DetectPlanes::cloud_cb, this);

    // Wait until the scan is ready, sleep for 10ms
    ros::Duration tictoc (0, 10000000);
    while (need_cloud_data_)
      {
        //tictoc.sleep ();
        ros::spinOnce ();
      }

    detectPlanes (*cloud_in_, resp);
    ROS_INFO ("Service request terminated.");
    return (true);
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // PointCloud message callback
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  cloud_cb (const PointCloudConstPtr& cloud)
  {
    if (!need_cloud_data_)
      return;

    ROS_INFO ("PointCloud message received on %s", input_cloud_topic_.c_str ());
    cloud_in_ = cloud;
    need_cloud_data_ = false;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //main function in the node
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  void
  detectPlanes (const PointCloud &cloud, GetPlaneClusters::Response &resp)
  {
    ros::Time ts = ros::Time::now ();

    // Create a downsampled representation of the cloud
    cloud_down_.header = cloud.header;
    cloud_down_.points.resize(cloud.points.size());
    cloud_down_.points = cloud.points;
    // Viewpoint value in the point cloud frame should be 0,0,0
    PointStamped viewpoint_cloud;
    viewpoint_cloud.point.x = viewpoint_cloud.point.y = viewpoint_cloud.point.z = 0.0;
    cloud_geometry::nearest::computePointCloudNormals (cloud_down_, cloud, k_, viewpoint_cloud);
    ROS_INFO("cloud, cloud_down_ sizes %lu,%lu", (unsigned long)cloud.points.size(), (unsigned long)cloud_down_.points.size());
    // ---[ Select points whose normals are parallel to the Y-axis (pointing upwards)
    //indices from pcd, correspond to row numbers
    vector<int> indices_z;
    cloud_geometry::getPointIndicesAxisParallelNormals (cloud_down_, 0, 1, 2, eps_angle_, axis_, indices_z);

#ifdef DEBUG_D      
    unsigned int ii = indices_z.size();
    for (unsigned int i=0; i<ii; i++)
      {
	ROS_INFO ("indices_z %d %f %f %f", indices_z[i], cloud_down_.points.at(indices_z[i]).x, cloud_down_.points.at(indices_z[i]).y, 
		  cloud_down_.points.at(indices_z[i]).z);
      }
#endif

#ifdef DEBUG
    ROS_INFO ("Number of points with normals parallel to Z: %d.", (int)indices_z.size ());
#endif
    // Find the best plane in this cluster (later, we can optimize and process more clusters individually)
    vector<int> inliers_down;
    //coefficients of plane equation
    vector<double> coeff;
    fitSACPlane (&cloud_down_, indices_z, inliers_down, coeff, viewpoint_cloud, sac_distance_threshold_);
    resp.a = coeff[0]; resp.b = coeff[1]; resp.c = coeff[2]; resp.d = coeff[3];
    resp.planeId = cloud_down_.header.seq;
    // Obtain the bounding 2D polygon of the table
    Polygon table;
    cloud_geometry::areas::convexHull2D (cloud_down_, inliers_down, coeff, table);
    cloud_geometry::getPointCloud (cloud_down_, inliers_down, cloud_annotated_);   // downsampled version
    cloud_geometry::nearest::computeCentroid (cloud_down_, inliers_down, resp.pcenter);
    // Send the table
    cloud_table_pub_.publish (cloud_annotated_);
    PolygonalMap pmap;
    pmap.header = cloud.header;
    pmap.polygons.resize (1);
    pmap.polygons[0] = table;
    pmap_pub_.publish (pmap);
    return;
  }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  fitSACPlane (PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
	       const PointStamped &viewpoint_cloud, double dist_thresh)
  {
    if ((int)indices.size () < clusters_min_pts_)
      {
        inliers.resize (0);
        coeff.resize (0);
        return (false);
      }

    // Create and initialize the SAC model
    sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
    sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
    sac->setMaxIterations (200);
    sac->setProbability (0.99);
    model->setDataSet (points, indices);

    // Search for the best plane
    if (sac->computeModel ())
      {
        if ((int)sac->getInliers ().size () < clusters_min_pts_)
	  {
	    //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
	    inliers.resize (0);
	    coeff.resize (0);
	    return (false);
	  }

        sac->computeCoefficients (coeff);     // Compute the model coefficients
        sac->refineCoefficients (coeff);      // Refine them using least-squares
       
        model->selectWithinDistance (coeff, dist_thresh, inliers);

        cloud_geometry::angles::flipNormalTowardsViewpoint (coeff, points->points.at (inliers[0]), viewpoint_cloud);

        // Project the inliers onto the model
        model->projectPointsInPlace (inliers, coeff);
      }
    return (true);
  }

};

/* ---[ */
int
main (int argc, char** argv)
{
  ros::init (argc, argv, "detect_planes");

  DetectPlanes p;
  ros::spin ();

  return (0);
}
/* ]--- */
