/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2009, Willow Garage
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
 *   * Neither the name of the Willow Garage nor the names of its
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
 *********************************************************************/

#include <stdlib.h>

#include <vector>
#include <iostream>

#include <point_cloud_mapping/kdtree/kdtree_ann.h>

// include all descriptors for you
#include <ias_descriptors_3d/all_descriptors.h>

//computeCentroid
#include <point_cloud_mapping/geometry/nearest.h>

//ros
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/PointStamped.h>
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/Point32.h>
#include <ros/ros.h>

//cloud_algos
#include <cloud_algos/cloud_algos.h>

//bullet
#include <tf/tf.h>

#include <tools/transform.h>
#include <angles/angles.h>

#include <ias_table_msgs/TriangularMesh.h>


using namespace std;


class BoxFit
{
 public:

  ///////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Contructor
   */
  BoxFit(ros::NodeHandle &n) :
    n_(n)
  {
    n_.param("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
    n_.param("output_box_topic", output_box_topic_, std::string("/box"));
    n_.param("output_mesh_topic", output_mesh_topic_, std::string("output_mesh"));
    n_.param("output_vtk_file", output_vtk_file_, std::string(""));
    box_pub_ = n_.advertise<visualization_msgs::Marker>("box", 0 );
    clusters_sub_ = n_.subscribe(input_cloud_topic_, 1, &BoxFit::cloud_cb, this);
    mesh_pub_ = n_.advertise<ias_table_msgs::TriangularMesh> (output_mesh_topic_, 1);
    coeff_.resize(15);
    r_ = g_ = 0.0;
    b_ = 1.0;
  }


  ///////////////////////////////////////////////////////////////////////////////
  /**
   * \brief Destructor
   */
  ~BoxFit()
  {
    
  }

  ///////////////////////////////////////////////////////////////////////////////
  /**
   * \brief cloud(cluster) callback
   */
  void cloud_cb (const sensor_msgs::PointCloudConstPtr& cloud)
  {
    ROS_INFO ("PointCloud message received on %s with %d points", input_cloud_topic_.c_str (), (int)cloud->points.size());
    lock.lock ();
    points_ = *cloud;
    lock.unlock ();

    find_model(points_, coeff_);
    triangulate_box(coeff_);
    //write the mesh to vtk file
    if (output_vtk_file_ != "")
      write_vtk_file(output_vtk_file_, mesh_);
    //publish fitted box on marker topic
    ROS_INFO ("Publishing box on topic %s.", n_.resolveName (output_box_topic_).c_str ());
    publish_model_rviz(coeff_);
  }


 ////////////////////////////////////////////////////////////////////////////////
  /**
   * actual model fitting happens here
   */
  void find_model(const sensor_msgs::PointCloud &cloud, std::vector<double> &coeff)
  {
    cloud_geometry::nearest::computeCentroid (points_, box_centroid_);
    coeff[0] = box_centroid_.x;
    coeff[1] = box_centroid_.y;
    coeff[2] = box_centroid_.z;


    // ----------------------------------------------
    // Read point cloud data and create Kd-tree that represents points.
    // We will compute features for all points in the point cloud.
    cloud_kdtree::KdTreeANN data_kdtree(cloud);
    std::vector<const geometry_msgs::Point32*> interest_pts;
    interest_pts.reserve(cloud.points.size());
    //std::vector<const geometry_msgs::Point32*> interest_pts(data.points.size());
    for (size_t i = 0 ; i < cloud.points.size() ; i++)
    {
      interest_pts.push_back(&(cloud.points[i]));
    }
    
    // ----------------------------------------------
    // SpectralAnalysis is not a descriptor, it is a class that holds
    // intermediate data to be used/shared by different descriptors.
    // It performs eigen-analyis of local neighborhoods and extracts
    // eigenvectors and values.  Here, we set it to look at neighborhoods
    // within a radius of 5.0 around each interest point.
    double r = 0.1;
    SpectralAnalysis sa(r);
    BoundingBoxSpectral bbox_spectral(r, sa);
    OrientationTangent o_tangent(1, 0, 0, sa);
    // ----------------------------------------------
    // Put all descriptors into a vector
    vector<Descriptor3D*> descriptors_3d;
    //   descriptors_3d.push_back(&shape_spectral);
    //   descriptors_3d.push_back(&spin_image1);
    //   descriptors_3d.push_back(&spin_image2);
    //   descriptors_3d.push_back(&o_normal);
    descriptors_3d.push_back(&o_tangent);
    //   descriptors_3d.push_back(&position);
    descriptors_3d.push_back(&bbox_spectral);
    
    // ----------------------------------------------
    // Iterate over each descriptor and compute features for each point in the point cloud.
    // The compute() populates a vector of vector of floats, i.e. a feature vector for each
    // interest point.  If the features couldn't be computed successfully for an interest point,
    // its feature vector has size 0
    unsigned int nbr_descriptors = descriptors_3d.size();
    vector<std::vector<std::vector<float> > > all_descriptor_results(nbr_descriptors);
    for (unsigned int i = 0 ; i < nbr_descriptors ; i++)
    {
      descriptors_3d[i]->compute(cloud, data_kdtree, interest_pts, all_descriptor_results[i]);
    }
    

    std::vector<float>& ot = all_descriptor_results[0][0];
    cout << "Orientation tangent size: " <<  ot.size() << endl;
    for (size_t i = 0 ; i < ot.size() ; i++)
      cerr << "Orientation tangent value(s): " << ot[i] << endl;

    //   // ----------------------------------------------
    //   // Print out the bounding box dimension features for the first point 0
    std::vector<float>& pt0_bbox_features = all_descriptor_results[1][0];
    cout << "Bounding box features size: " <<  pt0_bbox_features.size() << endl;
    for (size_t i = 0 ; i < pt0_bbox_features.size() ; i++)
    {
      if (i < 12)
      {     
        coeff[i+3] = pt0_bbox_features[i];
      }
      else
        ROS_WARN("Box dimensions bigger than 3 - unusual");
    }
    ROS_INFO("Box dimensions x: %f, y: %f, z: %f ", pt0_bbox_features[0],  pt0_bbox_features[1],  pt0_bbox_features[2]);
    ROS_INFO("Eigen vectors: \n\t%f %f %f \n\t%f %f %f \n\t%f %f %f", pt0_bbox_features[3], pt0_bbox_features[4], 
             pt0_bbox_features[5], pt0_bbox_features[6], pt0_bbox_features[7], pt0_bbox_features[8], 
             pt0_bbox_features[9], pt0_bbox_features[10],pt0_bbox_features[11]);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief triangulates box
   */  
  void triangulate_box(std::vector<double> &coeff)
  {
    geometry_msgs::Point32 current_point;
    ias_table_msgs::Triangle triangle;
    mesh_.points.resize(0);
    mesh_.triangles.resize(0);
    mesh_.header = points_.header;
    for (int i = -1; i <= 1; i = i+2)
    {
      for (int j = -1; j <= 1; j = j+2)
      {
        for (int k = -1; k <= 1; k = k+2)
        {
          current_point.x = coeff[0] + i * coeff[3]/2;
          current_point.y = coeff[1] + j * coeff[4]/2;
          current_point.z = coeff[2] + k * coeff[5]/2;
          mesh_.points.push_back(current_point);
        } 
      }
    }
    
    triangle.i = 0, triangle.j = 1, triangle.k = 2;
    mesh_.triangles.push_back(triangle);
    triangle.i = 0, triangle.j = 1, triangle.k = 4;
    mesh_.triangles.push_back(triangle);
    triangle.i = 0, triangle.j = 2, triangle.k = 6;
    mesh_.triangles.push_back(triangle);
    triangle.i = 0, triangle.j = 6, triangle.k = 4;
    mesh_.triangles.push_back(triangle);
    triangle.i = 1, triangle.j = 4, triangle.k = 5;
    mesh_.triangles.push_back(triangle);
    triangle.i = 5, triangle.j = 4, triangle.k = 6;
    mesh_.triangles.push_back(triangle);
    triangle.i = 5, triangle.j = 7, triangle.k = 6;
    mesh_.triangles.push_back(triangle);
    triangle.i = 7, triangle.j = 6, triangle.k = 2;
    mesh_.triangles.push_back(triangle);
    triangle.i = 7, triangle.j = 3, triangle.k = 2;
    mesh_.triangles.push_back(triangle);
    triangle.i = 1, triangle.j = 2, triangle.k = 3;
    mesh_.triangles.push_back(triangle);
    triangle.i = 1, triangle.j = 3, triangle.k = 7;
    mesh_.triangles.push_back(triangle);
    triangle.i = 1, triangle.j = 5, triangle.k = 7;
    mesh_.triangles.push_back(triangle);
    mesh_pub_.publish(mesh_);
  }
  

    ////////////////////////////////////////////////////////////////////////////////
  // \brief write TriangularMesh to vtk file
  void write_vtk_file(std::string output, ias_table_msgs::TriangularMesh &mesh_)
  {
    /* writing VTK file */
    FILE *f;
    f = fopen(output.c_str(),"w");
    fprintf (f, "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS %ld float\n",mesh_.points.size());
    unsigned long i;
    
    for (i=0; i<mesh_.points.size(); i++)
    {
      fprintf (f,"%f %f %f ", mesh_.points[i].x, mesh_.points[i].y, mesh_.points[i].z);
      fprintf (f,"\n");
    }
    
   //   fprintf(f,"VERTICES %ld %ld\n", mesh_.points.size(), 2*mesh_.points.size());
//      for (i=0; i<mesh_.points.size(); i++)
//        fprintf(f,"1 %ld\n", i);
    
//     ROS_INFO("vector: %ld, nr: %d  ", triangles.size(), nr_tr);
    
    fprintf(f,"\nPOLYGONS %ld %ld\n", mesh_.triangles.size(), 4*mesh_.triangles.size());
    for (unsigned long i=0; i< mesh_.triangles.size(); i++)
    {
      if ((unsigned long)mesh_.triangles[i].i  >= mesh_.points.size() || mesh_.triangles[i].i < 0 ||  isnan(mesh_.triangles[i].i))
        ;
      else if  ((unsigned long)mesh_.triangles[i].j  >= mesh_.points.size() || mesh_.triangles[i].j < 0 ||  isnan(mesh_.triangles[i].j))
        ;
      else if  ((unsigned long)mesh_.triangles[i].k  >= mesh_.points.size() || mesh_.triangles[i].k < 0 ||  isnan(mesh_.triangles[i].k))
        ;
      else if ( mesh_.triangles[i].i == mesh_.triangles[i].j || mesh_.triangles[i].i == mesh_.triangles[i].k ||
                mesh_.triangles[i].j == mesh_.triangles[i].k)
        ;
      else
        fprintf(f,"3 %d %d %d\n",mesh_.triangles[i].i, mesh_.triangles[i].k, mesh_.triangles[i].j);
    }
  }


  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief publishes model marker (to rviz)
   */  
  void publish_model_rviz (std::vector<double> &coeff)
  {
    btMatrix3x3 box_rot (coeff[6], coeff[7], coeff[8],
                         coeff[9], coeff[10], coeff[11],
                         coeff[12], coeff[13], coeff[14]);
    btMatrix3x3 box_rot_trans (box_rot.transpose());
    btQuaternion qt;
    box_rot_trans.getRotation(qt);

    visualization_msgs::Marker marker;
    //marker.header.frame_id = "base_link";
    //marker.header.stamp = ros::Time();
    // marker.ns = "my_namespace";
    marker.header = points_.header;
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = coeff[0];
    marker.pose.position.y = coeff[1];
    marker.pose.position.z = coeff[2];
    marker.pose.orientation.x = qt.x();
    marker.pose.orientation.y = qt.y();
    marker.pose.orientation.z = qt.z();
    marker.pose.orientation.w = qt.w();
    marker.scale.x = coeff[3];
    marker.scale.y = coeff[4];
    marker.scale.z = coeff[5];
    marker.color.a = 0.3;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    box_pub_.publish( marker );
  }


  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /** \brief Create synthetic point cloud
   * \param point cloud
   */
  void create_point_cloud(sensor_msgs::PointCloud& data)
  {
    data.header.frame_id = "/base_link";
    unsigned int nbr_pts = 2000;
    data.points.resize(nbr_pts);
    data.channels.resize(3);
    data.channels[0].name = "r";
    data.channels[1].name = "g";
    data.channels[2].name = "b";
    data.channels[0].values.resize(nbr_pts);
    data.channels[1].values.resize(nbr_pts);
    data.channels[2].values.resize(nbr_pts);

    for (unsigned int i = 0 ; i < nbr_pts ; i++)
    {
      data.points[i].x = rand() % 40;
      data.points[i].y = rand() % 20;
      data.points[i].z = rand() % 50;
      data.channels[0].values[i] = r_;
      data.channels[1].values[i] = g_;
      data.channels[2].values[i] = b_;
    }
  }


  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief spin
   */
  bool spin ()
  {
    ros::Duration delay(5.0);
    while (n_.ok())
    { 
      ros::spinOnce();
      delay.sleep();
    }
    return true;
  }
  
protected:
  sensor_msgs::PointCloud points_;
 
  ros::NodeHandle n_;
  //publishes original, input cloud
  ros::Publisher cloud_pub_;
   //subscribes to input cloud (cluster)
  ros::Subscriber clusters_sub_;
  //model rviz publisher
  ros::Publisher box_pub_;
  //box coefficients: cx, cy, cz, dx, dy, dz, e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z  
  std::vector<double> coeff_;
  geometry_msgs::Point32 box_centroid_;
  std::string input_cloud_topic_, output_box_topic_;
  //point color
  float r_, g_, b_;
  //lock point cloud
  boost::mutex lock;
  //triangular mesh
  ias_table_msgs::TriangularMesh mesh_;
  //mesh publisher
  ros::Publisher mesh_pub_;
  //publish mesh on topic
  std::string output_mesh_topic_,  output_vtk_file_;
};


int
main (int argc, char** argv)
{
  ros::init(argc, argv, "box_fit");
  ros::NodeHandle n("~");
  BoxFit box(n);
  box.spin();
  return 0;
}

