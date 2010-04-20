#ifndef CLOUD_ALGOS_BOX_ESTIMATION_H
#define CLOUD_ALGOS_BOX_ESTIMATION_H
#include <cloud_algos/cloud_algos.h>
#include <mapping_msgs/PolygonalMap.h>
#include <ias_visualization_msgs/PositionStringList.h>
#include <ias_table_msgs/TriangularMesh.h>
#include <point_cloud_mapping/geometry/point.h>

namespace cloud_algos
{

class BoxEstimation : public CloudAlgo
{
 public:
  BoxEstimation () { };
  typedef ias_table_msgs::TriangularMesh OutputType;
  typedef sensor_msgs::PointCloud InputType;

  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  static std::string default_output_topic ()
    {return std::string ("mesh_box");};

  static std::string default_node_name () 
    {return std::string ("box_estimation_node");};

  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>);
  boost::shared_ptr<const OutputType> output ();

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief function for actual model fitting
   * \param cloud de-noisified input point cloud message
   * \param coeff box to-be-filled-in coefficients(15 elements):
   * box center: cx, cy, cz, 
   * box dimensions: dx, dy, dz, 
   * box eigen axes: e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z  
   */
  void find_model (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief makes triangular mesh out of box coefficients
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void triangulate_box (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);

  ////////////////////////////////////////////////////////////////////////////////
  /**
   * \brief publish box as marker for rvis visualisation
   * \param cloud input point cloud message
   * \param coeff box coefficients (see find_model function):
   */
  void publish_marker (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);
 
 private: 
  boost::shared_ptr<OutputType> mesh_;
  std::vector<int> inliers_;
  std::vector<int> outliers_;

  ros::NodeHandle nh_;

 protected: 
  //model rviz publisher
  ros::Publisher box_pub_;
  //box coefficients: cx, cy, cz, dx, dy, dz, e1_x, e1y, e1z, e2_x, e2y, e2z, e3_x, e3y, e3z  
  std::vector<double> coeff_;
  geometry_msgs::Point32 box_centroid_;
  //publish box as marker
  std::string output_box_topic_;
  //point color
  float r_, g_, b_;
  //lock point cloud
  boost::mutex lock;
};

}
#endif


