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
 * $Id: detect_boxes.cpp 17089 2009-06-15 18:52:12Z pangercic $
 *
 */

/**
@mainpage

@htmlinclude manifest.html

@detect_boxes detects boxes in point cloud data.

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

#include <mapping_srvs/GetBoxes.h>


#define COLS 278
#define ROWS 1295
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

class DetectBoxes
{
  protected:
    ros::NodeHandle nh_;
  public:

    // ROS messages
    PointCloudConstPtr cloud_in_;

    PointCloud cloud_down_;
    Point leaf_width_;
    PointCloud cloud_annotated_;
    Point32 axis_;

    // Parameters
  string input_cloud_topic_;
  int k_;
  double max_z_;
  int clusters_min_pts_;
  
  int object_cluster_min_pts_;
  double object_cluster_tolerance_;

  bool need_cloud_data_;

  double sac_distance_threshold_, eps_angle_;

  double delta_z_, object_min_dist_from_table_;

  double min_angle_, max_angle_;

  Subscriber cloud_sub_;
  Publisher cloud_table_pub_, cloud_clusters_pub_, pmap_pub_, pmap_box_wall_pub_, cloud_filtered_pub_;
  ServiceServer detect_boxes_service_;

  int downsample_factor_;
  ServiceServer get_detect_boxes_service_;

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    DetectBoxes ()  
    {
      // 0.198669 0 0.980067 0 0 -1 0 0 0.980067 0 -0.198669 0 0 0 0 1
      axis_.x = 0; axis_.y = 0; axis_.z = 1;

      nh_.param ("~downsample_factor", downsample_factor_, 4); // Use every nth point
      nh_.param ("~search_k_closest", k_, 2);                  // 5 k-neighbors by default
      nh_.param ("~k_max_z", max_z_, 0.03);

      nh_.param ("~normal_eps_angle", eps_angle_, 15.0);       // 15 degrees
      eps_angle_ = angles::from_degrees (eps_angle_);          // convert to radians

      nh_.param ("~clusters_min_pts", clusters_min_pts_, 10);  // 10 points

      nh_.param ("~object_cluster_tolerance", object_cluster_tolerance_, 0.07);   // 7cm between two objects
      nh_.param ("~object_cluster_min_pts", object_cluster_min_pts_, 30);         // 30 points per object cluster

      nh_.param ("~table_delta_z", delta_z_, 0.03);                              // consider objects starting at 3cm from the table
      nh_.param ("~object_min_distance_from_table", object_min_dist_from_table_, 0.1); // objects which have their support more 10cm from the table will not be considered
      
      nh_.param ("~filtering_min_angle", min_angle_, 10.0);
      nh_.param ("~filtering_max_angle", max_angle_, 170.0);

      nh_.param ("~input_cloud_topic", input_cloud_topic_, string ("/cloud_pcd"));
      detect_boxes_service_ = nh_.advertiseService("/detect_boxes_service", &DetectBoxes::detect_boxes_service, this);

      // This should be set to whatever the leaf_width factor is in the downsampler
      nh_.param ("~sac_distance_threshold", sac_distance_threshold_, 0.01);     // 3 cm

      cloud_table_pub_ = nh_.advertise<PointCloud> ("cloud_table", 1);
      cloud_clusters_pub_ = nh_.advertise<PointCloud> ("cloud_clusters", 1);
      cloud_filtered_pub_ = nh_.advertise<PointCloud> ("cloud_filtered", 1);
      pmap_pub_ = nh_.advertise<PolygonalMap> ("pmap", 1);
      pmap_box_wall_pub_ = nh_.advertise<PolygonalMap> ("box_wall", 1);
      get_detect_boxes_service_ = nh_.advertiseService("get_detect_boxes_service", &DetectBoxes::detect_boxes_service, this);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   void
      updateParametersFromServer ()
    {
      nh_.getParam ("~input_cloud_topic", input_cloud_topic_);
      nh_.getParam ("~downsample_factor", downsample_factor_);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /** \brief Obtain a 24-bit RGB coded value from 3 independent <r, g, b> channel values
      * \param r the red channel value
      * \param g the green channel value
      * \param b the blue channel value
      */
    inline double
      getRGB (float r, float g, float b)
    {
      int res = (int(r * 255) << 16) | (int(g*255) << 8) | int(b*255);
      double rgb = *(float*)(&res);
      return (rgb);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      detect_boxes_service (GetBoxes::Request &req, GetBoxes::Response &resp)
    {
      ROS_INFO ("Service request initiated.");
      updateParametersFromServer ();

      // Subscribe to a point cloud topic
      need_cloud_data_ = true;
      cloud_sub_ = nh_.subscribe (input_cloud_topic_, 1, &DetectBoxes::cloud_cb, this);

      // Wait until the scan is ready, sleep for 10ms
      ros::Duration tictoc (0, 10000000);
      while (need_cloud_data_)
      {
        //tictoc.sleep ();
        ros::spinOnce ();
      }

      detectBoxes (*cloud_in_, resp);
      //to come
      //detectBoxes(*cloud_in_, resp);
      ROS_INFO ("Service request terminated.");
      return (true);
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // PointCloud message callback
    void
      cloud_cb (const PointCloudConstPtr& cloud)
    {
      if (!need_cloud_data_)
        return;

      ROS_INFO ("PointCloud message received on %s", input_cloud_topic_.c_str ());
      // if (cloud->points.size () != SR_ROWS * SR_COLS)
      //ROS_ERROR ("Number of points in the input point cloud: %d. This node is optimized for SwissRanger SR3k/4k (176x144) data!", (int)cloud->points.size ());
      cloud_in_ = cloud;
      need_cloud_data_ = false;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    void
      detectBoxes (const PointCloud &cloud, GetBoxes::Response &resp)
    {
      ros::Time ts = ros::Time::now ();

      // Create a downsampled representation of the cloud
      cloud_down_.header = cloud.header;
      // Viewpoint value in the point cloud frame should be 0,0,0
      Point32 viewpoint_cloud;
      viewpoint_cloud.x = viewpoint_cloud.y = 0.0;
      viewpoint_cloud.z = 1.0;

      // Estimate point normals and copy the relevant data
      cloud_geometry::nearest::computeOrganizedPointCloudNormalsWithFiltering 
        (cloud_down_, cloud, k_, downsample_factor_, COLS, ROWS, max_z_, min_angle_, 
         max_angle_, viewpoint_cloud);

      // ---[ Select points whose normals are perpendicular to the Z-axis
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

      // Filter the original pointcloud data with the same min/max angle for jump edges
      PointCloud cloud_filtered;// = cloud;
      cloud_geometry::nearest::filterJumpEdges (cloud, cloud_filtered, 1, COLS, ROWS, min_angle_, max_angle_, viewpoint_cloud);

      // Refine plane
      vector<int> inliers (cloud_filtered.points.size ());
      int j = 0;
      for (unsigned int i = 0; i < cloud_filtered.points.size (); i++)
      {
        double dist_to_plane = cloud_geometry::distances::pointToPlaneDistance (cloud_filtered.points[i], coeff);
        if (dist_to_plane < sac_distance_threshold_)
          inliers[j++] = i;
      }
      inliers.resize (j);

      // Obtain the bounding 2D polygon of the table
      Polygon table;
      cloud_geometry::areas::convexHull2D (cloud_down_, inliers_down, coeff, table);
#ifdef DEBUG
      PolygonalMap pmap;
      pmap.header = cloud.header;
      pmap.polygons.resize (1);
      pmap.polygons[0] = table;
      pmap_pub_.publish (pmap);
#endif
      cloud_filtered_pub_.publish (cloud_filtered);
      
      // Find the object clusters supported by the table
      Point32 min_p, max_p;
      cloud_geometry::statistics::getMinMax (cloud_filtered, inliers, min_p, max_p);
#ifdef DEBUG_D
      ROS_INFO ("min, max p: %g, %g, %g, %g, %g, %g", min_p.x, min_p.y, min_p.z, max_p.x, max_p.y, max_p.z);
#endif
      vector<int> object_inliers;
      vector<vector<int> > object_clusters;
      findObjectClusters (cloud_filtered, coeff, table, axis_, min_p, max_p, object_inliers, object_clusters, resp);
      ROS_INFO ("Nr. of clusters found: %d", object_clusters.size());

#ifdef DEBUG
      // Send the clusters
      cloud_clusters_pub_.publish (cloud_annotated_);
           
      // Send the table
      cloud_geometry::getPointCloud (cloud_down_, inliers_down, cloud_annotated_);   // downsampled version
      //cloud_geometry::getPointCloud (cloud_filtered, inliers, cloud_annotated_);              // full version
      cloud_table_pub_.publish (cloud_annotated_);
#endif
      ROS_INFO ("Results estimated in %g seconds.", (ros::Time::now () - ts).toSec ());
           
      cloud_geometry::nearest::computeCentroid (cloud_filtered, inliers, resp.table_center);
      //find planes in object cluster(s)
      vector<vector<int> >inliers_box_wall;
      //coefficients of box_wall(s) plane equation
      vector<vector<double> >coeff_box_wall;
      fitSACPlane3(&cloud_filtered, object_clusters, inliers_box_wall, coeff_box_wall, viewpoint_cloud, sac_distance_threshold_);
      Polygon box_wall1, box_wall2, box_wall3;
      cloud_geometry::areas::convexHull2D (cloud_filtered, inliers_box_wall[0], coeff_box_wall[0], box_wall1);
      cloud_geometry::areas::convexHull2D (cloud_filtered, inliers_box_wall[1], coeff_box_wall[1], box_wall2);
      cloud_geometry::areas::convexHull2D (cloud_filtered, inliers_box_wall[2], coeff_box_wall[2], box_wall3);
#ifdef DEBUG
      PolygonalMap pmap_box_wall;
      pmap_box_wall.header = cloud.header;
      pmap_box_wall.polygons.resize (3);
      pmap_box_wall.polygons[0] = box_wall1;
      pmap_box_wall.polygons[1] = box_wall2;
      pmap_box_wall.polygons[2] = box_wall3;
      pmap_box_wall_pub_.publish (pmap_box_wall);
#endif

      //fill in service data
      resp.boxes.resize(1);
      resp.boxes[0].plane0.resize(4);
      resp.boxes[0].plane1.resize(4);
      resp.boxes[0].plane2.resize(4); 
      for (unsigned int i = 0; i < 4; i++)
        resp.boxes[0].plane0[i] = coeff_box_wall[0].at(i);
      for (unsigned int i = 0; i < 4; i++)
        resp.boxes[0].plane1[i] = coeff_box_wall[1].at(i);
      for (unsigned int i = 0; i < 4; i++)
        resp.boxes[0].plane2[i] = coeff_box_wall[2].at(i);
    
      //calculate and store angles between planes
      double box_wall_angles[3];
      for (unsigned int i = 0; i < 3; i++)
        {
          switch (i)
            {
            case 0:
              box_wall_angles[i] = cloud_geometry::angles::getAngleBetweenPlanes (coeff_box_wall[0], coeff_box_wall[1]);
              resp.boxes[0].angle01 =  box_wall_angles[i]; 
              break;
            case 1:
              box_wall_angles[i] = cloud_geometry::angles::getAngleBetweenPlanes (coeff_box_wall[1], coeff_box_wall[2]);
              resp.boxes[0].angle12 =  box_wall_angles[i]; 
              break;
            case 2:
             box_wall_angles[i] = cloud_geometry::angles::getAngleBetweenPlanes (coeff_box_wall[0], coeff_box_wall[2]);
             resp.boxes[0].angle02 =  box_wall_angles[i]; 
            }
          // ROS_INFO("angle: %f", angles::to_degrees(box_wall_angles[i]));
        }
      return;
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
    void
      findObjectClusters (const PointCloud &cloud, const vector<double> &coeff, const Polygon &table,
                          const Point32 &axis, const Point32 &min_p, const Point32 &max_p, vector<int> &object_indices, 
                          vector<vector<int> > &object_clusters, GetBoxes::Response &resp)
    {
      int nr_p = 0;
      Point32 pt;
      //save in here all points that do not fly in thin air and project onto a table 
      object_indices.resize (cloud.points.size ());

      // Iterate over the entire cloud to extract the object clusters
      for (unsigned int i = 0; i < cloud.points.size (); i++)
      {
        // Select all the points in the given bounds - check all axes
        if ( axis.x == 1 && ( cloud.points.at (i).y < min_p.y || cloud.points.at (i).y > max_p.y || cloud.points.at (i).z < min_p.z || cloud.points.at (i).z > max_p.z ) )
          continue;

        else if ( axis.y == 1 && ( cloud.points.at (i).x < min_p.x || cloud.points.at (i).x > max_p.x || cloud.points.at (i).z < min_p.z || cloud.points.at (i).z > max_p.z ) )
          continue;

        else if ( axis.z == 1 && ( cloud.points.at (i).x < min_p.x || cloud.points.at (i).x > max_p.x || cloud.points.at (i).y < min_p.y || cloud.points.at (i).y > max_p.y ) )
          continue;

        // Calculate the distance from the point to the plane
        double dist_to_plane = coeff.at (0) * cloud.points.at (i).x +
                               coeff.at (1) * cloud.points.at (i).y +
                               coeff.at (2) * cloud.points.at (i).z +
                               coeff.at (3) * 1;
        // Calculate the projection of the point on the plane
        pt.x = cloud.points.at (i).x - dist_to_plane * coeff.at (0);
        pt.y = cloud.points.at (i).y - dist_to_plane * coeff.at (1);
        pt.z = cloud.points.at (i).z - dist_to_plane * coeff.at (2);

        if (dist_to_plane > delta_z_ && cloud_geometry::areas::isPointIn2DPolygon (pt, table))
        {
          object_indices[nr_p] = i;
          nr_p++;
        }
      }
      object_indices.resize (nr_p);

      // Find the clusters
      nr_p = 0;
      //vector<vector<int> > object_clusters;
      cloud_geometry::nearest::extractEuclideanClusters (cloud, object_indices, object_cluster_tolerance_,
                                                         object_clusters, -1, -1, -1, -1, object_cluster_min_pts_);

#ifdef DEBUG
        int total_nr_pts = 0;
        for (unsigned int i = 0; i < object_clusters.size (); i++)
          total_nr_pts += object_clusters[i].size ();

        cloud_annotated_.header = cloud.header;
        cloud_annotated_.points.resize (total_nr_pts);
        cloud_annotated_.channels.resize (1);
        cloud_annotated_.channels[0].name = "rgb";
        cloud_annotated_.channels[0].values.resize (total_nr_pts);
        ROS_INFO ("Number of clusters found: %d", (int)object_clusters.size ());
#endif

      geometry_msgs::Point32 min_p_cluster, max_p_cluster;
      
      ////resp.oclusters.resize (object_clusters.size ());
      for (unsigned int i = 0; i < object_clusters.size (); i++)
      {
#ifdef DEBUG
        float rgb = getRGB (rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0), rand () / (RAND_MAX + 1.0));
#endif
        vector<int> object_idx = object_clusters.at (i);

        // Check whether this object cluster is supported by the table or just flying through thin air
        cloud_geometry::statistics::getMinMax (cloud, object_idx, min_p_cluster, max_p_cluster);
        // Select all the points in the given bounds - check all axes
        if ( axis.x == 1 && ( min_p_cluster.x > max_p.x + object_min_dist_from_table_ ) )
          continue;        
        if ( axis.y == 1 && ( min_p_cluster.y > max_p.y + object_min_dist_from_table_ ) )
          continue;        
        if ( axis.z == 1 && ( min_p_cluster.z > max_p.z + object_min_dist_from_table_ ) )
          continue;        

        // Process this cluster and extract the centroid and the bounds
        for (unsigned int j = 0; j < object_idx.size (); j++)
        {
          object_indices[nr_p] = object_idx.at (j);
#ifdef DEBUG          
            cloud_annotated_.points[nr_p] = cloud.points.at (object_idx.at (j));
            cloud_annotated_.channels[0].values[nr_p] = rgb;
#endif
          nr_p++;
        }
        ////cloud_geometry::statistics::getMinMax (cloud, object_idx, resp.oclusters[i].min_bound, resp.oclusters[i].max_bound);
        ////cloud_geometry::nearest::computeCentroid (cloud, object_idx, resp.oclusters[i].center);
      }
      object_indices.resize (nr_p);
#ifdef DEBUG
        cloud_annotated_.points.resize (nr_p);
        cloud_annotated_.channels[0].values.resize (nr_p);
#endif
        
    }


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    bool
      fitSACPlane (PointCloud *points, vector<int> &indices, vector<int> &inliers, vector<double> &coeff,
                   const Point32 &viewpoint_cloud, double dist_thresh)
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


   //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  bool
  fitSACPlane3 (PointCloud *points, vector<vector<int> > &object_clusters, vector<vector<int> >&inliers, 
                vector<vector<double> >&coeff, const Point32 &viewpoint_cloud, double dist_thresh)
  {
    vector<int> inliers_;
    vector<double> coeff_;
    for (unsigned int i = 0; i < object_clusters.size (); i++)
         {
           vector<int> object_idx = object_clusters.at (i);
           if ((int)object_idx.size () < clusters_min_pts_)
             {
               inliers_.resize (0);
               coeff_.resize (0);
               return (false);
             }
           
           // Create and initialize the SAC model
           sample_consensus::SACModelPlane *model = new sample_consensus::SACModelPlane ();
           sample_consensus::SAC *sac             = new sample_consensus::MSAC (model, sac_distance_threshold_);
           sac->setMaxIterations (200);
           sac->setProbability (0.99);
           model->setDataSet (points, object_idx);
           
           // Search for the best plane
           for (unsigned int i = 0; i < 3; i++)
             {
               if (sac->computeModel (2))
                 {
                   if ((int)sac->getInliers ().size () < clusters_min_pts_)
                     {
                       //ROS_ERROR ("fitSACPlane: Inliers.size (%d) < sac_min_points_per_model (%d)!", sac->getInliers ().size (), sac_min_points_per_model_);
                       inliers_.resize (0);
                       coeff_.resize (0);
                       return (false);
                     }
                   
                   sac->computeCoefficients (coeff_);     // Compute the model coefficients
                   sac->refineCoefficients (coeff_);      // Refine them using least-squares
                   // ROS_INFO("box wall coeff %f,%f,%f, %f",coeff_[0], coeff_[1], coeff_[2], coeff_[3]);
                   model->selectWithinDistance (coeff_, dist_thresh, inliers_);
                   cloud_geometry::angles::flipNormalTowardsViewpoint (coeff_, points->points.at (inliers_[0]), viewpoint_cloud);
                   
                   // Project the inliers onto the model
                   model->projectPointsInPlace (inliers_, coeff_);
                   sac->removeInliers();
                 }
               inliers.push_back(inliers_);
               coeff.push_back(coeff_);
             }
         }
       return (true);
  }
};

/* ---[ */
int
  main (int argc, char** argv)
{
  ros::init (argc, argv, "detect_boxes_sr");

  DetectBoxes p;
  ros::spin ();

  return (0);
}
/* ]--- */
