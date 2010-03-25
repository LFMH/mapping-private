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
#include <cloud_algos/box_fit_algo.h>

//bullet
#include <tf/tf.h>

#include <tools/transform.h>
#include <angles/angles.h>

#include <ias_table_msgs/TriangularMesh.h>


using namespace std;
using namespace cloud_algos;

void BoxEstimation::init (ros::NodeHandle& nh)
{
  nh_ = nh;
  nh_.param("input_cloud_topic", input_cloud_topic_, std::string("/cloud_pcd"));
  nh_.param("output_box_topic", output_box_topic_, std::string("/box"));
  nh_.param("output_mesh_topic", output_mesh_topic_, std::string("output_mesh"));
  nh_.param("output_vtk_file", output_vtk_file_, std::string(""));
  box_pub_ = nh_.advertise<visualization_msgs::Marker>("box", 0 );
  mesh_pub_ = nh_.advertise<ias_table_msgs::TriangularMesh> (output_mesh_topic_, 1);
  coeff_.resize(15);
  r_ = g_ = 0.0;
  b_ = 1.0;
}

void BoxEstimation::pre  ()
{
}

void BoxEstimation::post ()
{
}

std::vector<std::string> BoxEstimation::requires ()
{
  std::vector<std::string> ret;
  ret.push_back (std::string("index"));
  return ret;
}

std::vector<std::string> BoxEstimation::provides ()
{
  return std::vector<std::string>();
}

std::string BoxEstimation::process (const boost::shared_ptr<const InputType> input)
{
  ROS_INFO ("PointCloud message received on %s with %d points", input_cloud_topic_.c_str (), (int)input->points.size ());
  find_model (input, coeff_);
  triangulate_box (input, coeff_);
  //publish fitted box on marker topic
  ROS_INFO ("Publishing box on topic %s.", nh_.resolveName (output_box_topic_).c_str ());
  publish_marker (input, coeff_);
  return std::string ("ok");
}

boost::shared_ptr<const BoxEstimation::OutputType> BoxEstimation::output () 
{
  return mesh_;
};

////////////////////////////////////////////////////////////////////////////////
/**
 * actual model fitting happens here
 */
void BoxEstimation::find_model(boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff)
{
  cloud_geometry::nearest::computeCentroid (*cloud, box_centroid_);
  coeff[0] = box_centroid_.x;
  coeff[1] = box_centroid_.y;
  coeff[2] = box_centroid_.z;

  // ----------------------------------------------
  // Read point cloud data and create Kd-tree that represents points.
  // We will compute features for all points in the point cloud.
  std::vector<const geometry_msgs::Point32*> interest_pts;
  interest_pts.reserve(cloud->points.size());
  //std::vector<const geometry_msgs::Point32*> interest_pts(data.points.size());
  for (size_t i = 0 ; i < cloud->points.size() ; i++)
  {
    interest_pts.push_back(&(cloud->points[i]));
  }
  
  cloud_kdtree::KdTreeANN data_kdtree(*cloud);
  
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
    descriptors_3d[i]->compute(*cloud, data_kdtree, interest_pts, all_descriptor_results[i]);
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
void BoxEstimation::triangulate_box(boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff)
{
  geometry_msgs::Point32 current_point;
  ias_table_msgs::Triangle triangle;
  mesh_ = boost::shared_ptr <BoxEstimation::OutputType> (new BoxEstimation::OutputType);
  mesh_->points.resize(8);
  mesh_->triangles.resize(12);
  mesh_->header = cloud->header;

  int counter = 0;

  // create box vertices
  for (int i = -1; i <= 1; i = i+2)
  {
    for (int j = -1; j <= 1; j = j+2)
    {
      for (int k = -1; k <= 1; k = k+2)
      {
        current_point.x = coeff[0] + i * coeff[3]/2;
        current_point.y = coeff[1] + j * coeff[4]/2;
        current_point.z = coeff[2] + k * coeff[5]/2;
        mesh_->points[counter++] = current_point;
      } 
    }
  }
  
  // fill in the box sides (2 triangles per side)
  counter = 0;
  triangle.i = 0, triangle.j = 1, triangle.k = 2;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 0, triangle.j = 1, triangle.k = 4;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 0, triangle.j = 2, triangle.k = 6;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 0, triangle.j = 6, triangle.k = 4;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 4, triangle.k = 5;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 5, triangle.j = 4, triangle.k = 6;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 5, triangle.j = 7, triangle.k = 6;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 7, triangle.j = 6, triangle.k = 2;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 7, triangle.j = 3, triangle.k = 2;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 2, triangle.k = 3;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 3, triangle.k = 7;
  mesh_->triangles[counter++] = triangle;
  triangle.i = 1, triangle.j = 5, triangle.k = 7;
  mesh_->triangles[counter++] = triangle;
}

////////////////////////////////////////////////////////////////////////////////
/**
 * \brief publishes model marker (to rviz)
 */  
void BoxEstimation::publish_marker (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff)
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
  marker.header = cloud->header;
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

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <BoxEstimation> (argc, argv);
}
#endif

