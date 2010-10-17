#ifndef CLOUD_ALGOS_MLS_FIT_H
#define CLOUD_ALGOS_MLS_FIT_H
#include <cloud_algos/cloud_algos.h>

// Eigen
#include <Eigen3/Core>
// if needed:
#include <Eigen3/LU> // matrix inversion
#include <Eigen3/Geometry> // cross product

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

// Sample Consensus
//#include <point_cloud_mapping/sample_consensus/sac.h>
//#include <point_cloud_mapping/sample_consensus/msac.h>
//#include <point_cloud_mapping/sample_consensus/ransac.h>

namespace cloud_algos
{

// Moving Least Squares implementation
class MovingLeastSquares : public CloudAlgo
{
 public:

  // Input/output type
  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  // Options
  enum GenerationMethod {COPY, FILLING, CLOSE, SURROUNDING, ADAPTIVE, FIXED_K};
  GenerationMethod point_generation_; // method for generating the points to be fit to the underlying surface
  enum TangentMethod {PCA, WPCA, IWPCA, SAC};
  TangentMethod approximating_tangent_; // method for getting the approximate tangent plane of the underlying surface
  double radius_;           // search radius for getting the nearest neighbors
  //double radius_curvature_; // to have the curvature estimates in the same interval as normal estimation using PCA would provide it, set it to the same radius that would be used there
  int max_nn_;              // maximum number of nearest neighbors to consider
  int order_;               // order of the polynomial to be fit
  bool filter_points_;      // should the generated/provided points be filtered based on proximity to the underlying surface?
  bool polynomial_fit_;     // should the surface and normal be approximated using a polynomial, or do only the tangent estimation?
  bool compute_moments_;    // switch to compute moment invariants (j1, j2, j3)
  double sqr_gauss_param_;  // parameter for distance based weighting of neighbors (radius_*radius_ works ok)
  geometry_msgs::PointStamped *viewpoint_cloud_; // viewpoint towards which the normals have to point

  // Topic name to advertise
  static std::string default_output_topic ()
    {return std::string ("cloud_mls");}

  // Topic name to subscribe to
  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  // Node name
  static std::string default_node_name ()
    {return std::string ("mls_fit_node");}

  // Algorithm methods
  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  boost::shared_ptr<const OutputType> output (); // TODO: return OutputType or this?

  // Setter functions - TODO: first test, and if useful request setter for epsion_ !!! (bucket size not important)
  void setKdTree (cloud_kdtree::KdTree *kdtree)
    {kdtree_ = kdtree;}
  void setCloud2Fit (boost::shared_ptr<sensor_msgs::PointCloud> cloud_fit)
    {cloud_fit_ = cloud_fit;}

  // Clearing previously set data
  void clear ()
  {
    points_indices_.clear ();
    points_sqr_distances_.clear ();
    if (kdtree_ != NULL)
    {
      delete kdtree_;
      kdtree_ = NULL;
    }
//    if (cloud_fit_ != NULL)
//    {
//      delete cloud_fit_;
//      cloud_fit_ = NULL;
//    }
    if (viewpoint_cloud_ != NULL)
    {
      delete viewpoint_cloud_;
      viewpoint_cloud_ = NULL;
    }
  }

  // Constructor-Destructor
  MovingLeastSquares () : CloudAlgo ()
  {
    kdtree_ = NULL;
    //cloud_fit_ = NULL; // not needed with a smart pointer
    viewpoint_cloud_ = NULL;

    point_generation_ = COPY;
    approximating_tangent_ = PCA;
    radius_ = 0.03;
    //radius_curvature_ = 0.02;
    max_nn_ = 300;
    order_ = 2;
    filter_points_ = false;
    polynomial_fit_ = true;
    compute_moments_ = false;
    sqr_gauss_param_ = radius_*radius_;
    viewpoint_cloud_ = NULL;
  }
  ~MovingLeastSquares ()
  {
    clear ();
  }

  ros::Publisher createPublisher (ros::NodeHandle& nh)
  {
    ros::Publisher p = nh.advertise<OutputType> (default_output_topic (), 5);
    return p;
  }
 private:

  // ROS stuff
  ros::NodeHandle nh_;
  //ros::Publisher pub_;

  // ROS messages
  boost::shared_ptr<sensor_msgs::PointCloud> cloud_fit_;

  /// TODO add setters for the vectors too, passed by &

  // Kd-tree stuff
  cloud_kdtree::KdTree *kdtree_;
  std::vector<std::vector<int> > points_indices_;
  std::vector<std::vector<float> > points_sqr_distances_;

  // number of coefficients, to be computed from the provided order_
  int nr_coeff_;

  // TODO should these be floats instead? would that work with double vectors we use/get from elsewhere

  /*
  // diagonal weight matrix
  Eigen3::MatrixXd weight_;
  // matrix for the powers of the polynomial representation evaluated for the neighborhood
  Eigen3::MatrixXd P_;
  // vector for storing function values
  Eigen3::VectorXd f_vec_;
  // vector for storing coefficients
  Eigen3::VectorXd c_vec_;
  // matrices for intermediary results
  Eigen3::MatrixXd P_weight_;
  Eigen3::MatrixXd P_weight_Pt_;
  Eigen3::MatrixXd inv_P_weight_Pt_;
  //Eigen3::MatrixXd inv_P_weight_Pt_P_weight_;
  */
};

}
#endif

