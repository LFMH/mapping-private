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
  void find_model (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);
  void triangulate_box (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);
  void publish_marker (boost::shared_ptr<const sensor_msgs::PointCloud> cloud, std::vector<double> &coeff);
 
 private: 
  boost::shared_ptr<OutputType> mesh_;
  std::vector<int> inliers_;
  std::vector<int> outliers_;

  ros::NodeHandle nh_;

 protected:
 
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
  //mesh publisher
  ros::Publisher mesh_pub_;
  //publish mesh on topic
  std::string output_mesh_topic_,  output_vtk_file_;
};

}
#endif


