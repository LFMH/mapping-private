#ifndef CLOUD_ALGOS_PFH_H
#define CLOUD_ALGOS_PFH_H
#include <cloud_algos/cloud_algos.h>

// Eigen
//#include <Eigen3/Array>

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

namespace cloud_algos
{

class PointFeatureHistogram : public CloudAlgo
{
 public:

  // Input/output type
  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  // Options
  double radius_;     // search radius for getting the nearest neighbors
  int max_nn_;        // maximum number of nearest neighbors to consider
  int quantum_;       // number of divisions in a feature's definition interval
  bool use_dist_;     // enable to use distance as a feature
  bool combine_;      // enable to count co-occurrences of features
  bool differential_; // enable to let histogram values for a feature be relative to the previous one - if not combined
  bool check_flip_;   // enable to make source-target selection consistent
  bool abs_angles_;   // enable to use absolute values of angles instead of the 'directional' values
  bool average_;      // enable to create the final histogram by a weighted average of the neighboring ones (useful if only star-like connections were used)
  int point_label_ ;  // set the value for the class label (-1 if not known / to be classified)

  // Topic name to subscribe to
  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}

  // Topic name to advertise
  static std::string default_output_topic ()
    {return std::string ("cloud_pfh");};

  // Node name
  static std::string default_node_name () 
    {return std::string ("pfh_node");};

  // Algorithm methods
  void init (ros::NodeHandle&);
  void pre  ();
  void post ();
  std::vector<std::string> requires ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();

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
  }

  // Constructor-Destructor
  PointFeatureHistogram () : CloudAlgo ()
  {
    kdtree_ = NULL;

    // set default values for parameters to produce FPFHs
    radius_ = 0.03;
    max_nn_ = 100;
    quantum_ = 9;
    use_dist_ = 0;
    combine_ = false;
    differential_ = false;
    check_flip_ = true;
    abs_angles_ = false;
    average_ = true;
    point_label_ = -1;
  }
  ~PointFeatureHistogram ()
  {
    clear ();
  }

  // Compute the index of each feature - merge 1 into the last [s,s+1) interval
  static inline int getFeatureIndice (int &quantum, double &feature)
  {
    return std::max (0, std::min (quantum-1, (int) floor (quantum * feature)));
  }

  // Features for a point pair
  static inline bool getPointPairFeatures (const boost::shared_ptr<const PointFeatureHistogram::InputType> cloud,
      int nxIdx, int sIdx, int tIdx, double *u, double *v, double *w, double *tmp, double *dP2P1,
      double &alpha, double &beta, double &gamma, double &delta, double max_dist=1, bool check_flip = true, bool abs_angles = false, bool verbose = false)
  {
    // P2 - P1 (2nd point XYZ coordinates - 1st point XYZ coordinate)
    dP2P1[0] = cloud->points[tIdx].x - cloud->points[sIdx].x;
    dP2P1[1] = cloud->points[tIdx].y - cloud->points[sIdx].y;
    dP2P1[2] = cloud->points[tIdx].z - cloud->points[sIdx].z;

    // If distance was not provided, compute it
    if (delta <= 0)
    {
      // Delta : Euclidean norm = || P2 - P1 ||
      double delta_sqr = dP2P1[0]*dP2P1[0] + dP2P1[1]*dP2P1[1] + dP2P1[2]*dP2P1[2];
      if (delta_sqr == 0)
      {
        ROS_ERROR ("getPointPairFeatures: Delta (Euclidean distance) between %d and %d is 0!", sIdx, tIdx);
        return false;
      }
      delta = sqrt (delta_sqr);
    }

    // Gamma : U * dP2P1 / || dP2P1 ||
    // Angle between U and dP2P1
    /// @note: dot product of two normalized vectors returns results in [-1,1]
    double angle2 = - (cloud->channels[nxIdx+0].values[tIdx] * dP2P1[0] +
                       cloud->channels[nxIdx+1].values[tIdx] * dP2P1[1] +
                       cloud->channels[nxIdx+2].values[tIdx] * dP2P1[2]) / delta;
    int source = sIdx, target = tIdx;
    bool do_flip = false;
    if (check_flip)
    {
      gamma = (cloud->channels[nxIdx+0].values[sIdx] * dP2P1[0] +
               cloud->channels[nxIdx+1].values[sIdx] * dP2P1[1] +
               cloud->channels[nxIdx+2].values[sIdx] * dP2P1[2]) / delta;
      if (acos (gamma) > acos (angle2))
        do_flip = true;
    }

    /// @NOTE: FPFH always uses the flipped one !!!
    if (!check_flip || do_flip)
    {
      source = tIdx;
      //source = cp;
      target = sIdx;
      dP2P1[0] = -dP2P1[0];
      dP2P1[1] = -dP2P1[1];
      dP2P1[2] = -dP2P1[2];
      gamma = angle2;
    }

    /// @NOTE: FPFH isn't doing this
    if (abs_angles)
      gamma = fabs (gamma);

    // ---[ Coordinate system (U x V x W)
    // U = N1
    u[0] = cloud->channels[nxIdx+0].values[source];
    u[1] = cloud->channels[nxIdx+1].values[source];
    u[2] = cloud->channels[nxIdx+2].values[source];

    // cross product (dP2P1 x u)
    tmp[0] = dP2P1[1] * u[2] - dP2P1[2] * u[1];
    tmp[1] = dP2P1[2] * u[0] - dP2P1[0] * u[2];
    tmp[2] = dP2P1[0] * u[1] - dP2P1[1] * u[0];

    // check "angle" between U and (P2-P1)
    double dP2P1U_norm = sqrt (tmp[0] * tmp[0] + tmp[1] * tmp[1] + tmp[2] * tmp[2]);
    if (dP2P1U_norm == 0)
    {
      ROS_ERROR ("getPointPairFeatures: Angle between normal of %d and the direction vector to %d is 0!", source, target);
      return false;
    }

    // V = (P2-P1) x U / || [(P2-P1) x U]
    v[0] = tmp[0] / dP2P1U_norm;
    v[1] = tmp[1] / dP2P1U_norm;
    v[2] = tmp[2] / dP2P1U_norm;

    // cross product (W = U x V)
    w[0] = u[1] * v[2] - u[2] * v[1];
    w[1] = u[2] * v[0] - u[0] * v[2];
    w[2] = u[0] * v[1] - u[1] * v[0];
    // ]---

    // Beta : V * N2;
    /// Polar angle from V: 0->PI
    /// @note: dot product of two normalized vectors returns results in [-1,1]
    beta = v[0] * cloud->channels[nxIdx+0].values[target] + v[1] * cloud->channels[nxIdx+1].values[target] + v[2] * cloud->channels[nxIdx+2].values[target];

    /// @NOTE: FPFH isn't doing this
    if (abs_angles)
      beta = fabs (beta);

    // Alpha : arctan (W * N2, U * N2) i.e. angle of N2 in the X=U, Y=W coord system
    /// Azimuthal angle in the U-W plane: 0->2PI
    /// @note: atan2 return results in (-PI,PI)
    if (abs_angles)
      alpha = atan2 (fabs ( w[0] * cloud->channels[nxIdx+0].values[target] + w[1] * cloud->channels[nxIdx+1].values[target] + w[2] * cloud->channels[nxIdx+2].values[target] ),
                     fabs ( u[0] * cloud->channels[nxIdx+0].values[target] + u[1] * cloud->channels[nxIdx+1].values[target] + u[2] * cloud->channels[nxIdx+2].values[target] ) );
    else
      alpha = atan2 (w[0] * cloud->channels[nxIdx+0].values[target] + w[1] * cloud->channels[nxIdx+1].values[target] + w[2] * cloud->channels[nxIdx+2].values[target],
                     u[0] * cloud->channels[nxIdx+0].values[target] + u[1] * cloud->channels[nxIdx+1].values[target] + u[2] * cloud->channels[nxIdx+2].values[target]);

    #if DEBUG
      //double f1d, f2d, f3d;
      //f1d = RAD2DEG (alpha);
      //f2d = RAD2DEG (acos (beta));
      //f3d = RAD2DEG (acos (gamma));
      //cerr << "features: " << alpha << "/" << f1d << " " << beta << "/" << f2d << " " << gamma << "/" << f3d << " " << delta << endl;
    #endif


    // Moving values ideally in the [0,1) interval - see usage of max/min/floor below!
    delta = delta / max_dist;
    if (abs_angles)
      alpha = alpha / (M_PI/2);
    else
    {
      // Normalize the alpha angle feature to fall from (-PI,PI) into (0,1)
      alpha = (alpha + M_PI) / (2.0 * M_PI);
      // Normalize the beta and gamma angle features to fall from [-1,1] into [0,1]
      beta  = (beta + 1.0) / 2.0;
      gamma = (gamma + 1.0) / 2.0;
    }

    #if DEBUG
      //cerr << "normalized: " << alpha << " " << beta << " " << gamma << " " << delta << endl;
    #endif

    return true;
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
  boost::shared_ptr<sensor_msgs::PointCloud> cloud_pfh_;

  // Kd-tree stuff
  cloud_kdtree::KdTree *kdtree_;
  std::vector<std::vector<int> > points_indices_;
  std::vector<std::vector<float> > points_sqr_distances_;

  // PFH stuff
  int nr_features_;
  int nr_bins_;
};

}
#endif

