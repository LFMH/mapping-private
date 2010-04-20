#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/radius_estimation.h>

using namespace std;
using namespace cloud_algos;

void LocalRadiusEstimation::init (ros::NodeHandle& nh)
{
  nh_ = nh;
}

void LocalRadiusEstimation::pre ()
{
  nh_.param("radius", radius_, radius_);
  nh_.param("max_nn", max_nn_, max_nn_);
  nh_.param("plane_radius", plane_radius_, plane_radius_);
  nh_.param("distance_div", distance_div_, distance_div_);
  nh_.param("point_label", point_label_, point_label_);
  nh_.param("rmin2curvature", rmin2curvature_, rmin2curvature_);
}

void LocalRadiusEstimation::post ()
{

}

std::vector<std::string> LocalRadiusEstimation::requires ()
{
  std::vector<std::string> requires;
  // requires 3D coordinates
  requires.push_back("x");
  requires.push_back("y");
  requires.push_back("z");
  // requires normals
  requires.push_back("nx");
  requires.push_back("ny");
  requires.push_back("nz");
  return requires;
}

std::vector<std::string> LocalRadiusEstimation::provides ()
{
  std::vector<std::string> provides;
  provides.push_back ("r_min");
  provides.push_back ("r_max");
  provides.push_back ("r_dif");
  //if (point_label_ != -1)
  provides.push_back ("point_label");
  return provides;
}

std::string LocalRadiusEstimation::process (const boost::shared_ptr<const LocalRadiusEstimation::InputType>& cloud)
{
  // TODO where to put this?
  clear ();

  // Check if normals exist
  int nxIdx = getChannelIndex(cloud, "nx");
  //int nxIdx = -1;
  //for (unsigned int d = 0; d < cloud->channels.size (); d++)
  //  if (cloud->channels[d].name == "nx")
  //    { nxIdx = d; break; }
  if (nxIdx == -1)
  {
    ROS_ERROR ("[LocalRadiusEstimation] Provided point cloud does not have normals. Use the normal_estimation or mls_fit first!");
    return std::string("missing normals");
  }
  /// @NOTE: we assume that if nx exists then ny and nz are the channels following it

  // Timers
  ros::Time global_time = ros::Time::now ();
  ros::Time ts;

  // Copy the original PCD
  cloud_radius_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
  cloud_radius_->header   = cloud->header;
  cloud_radius_->points   = cloud->points;
  cloud_radius_->channels = cloud->channels;

  // Allocate and name the extra needed channels (don't forget to allocate space for values too!)
  int rIdx = cloud_radius_->channels.size ();
  cloud_radius_->channels.resize (rIdx + 2 + 1);
  cloud_radius_->channels[rIdx+0].name = "r_min";
  cloud_radius_->channels[rIdx+1].name = "r_max";
  cloud_radius_->channels[rIdx+2].name = "r_dif";
  int labelIdx = cloud_radius_->channels.size ();
  cloud_radius_->channels.resize (labelIdx  + 1);
  cloud_radius_->channels[labelIdx].name = "point_label";

  // Allocate space for the extra needed channel values
  for (size_t d = rIdx; d < cloud_radius_->channels.size (); d++)
  {
    cloud_radius_->channels[d].values.resize (cloud_radius_->points.size ());
    ROS_INFO ("[LocalRadiusEstimation] Added channel: %s", cloud_radius_->channels[d].name.c_str ());
  }

  // Allocate space for minimum and maximum angle values in each distance bin
  vector<vector<double> > min_max_angle_by_dist (distance_div_);
  for (int di=0; di<distance_div_; di++)
    min_max_angle_by_dist[di].resize (2);

  // Create Kd-Tree
  if (kdtree_ == NULL)
  {
    ts = ros::Time::now ();
    /// @NOTE: kd-tree for the original PCD... doesn't matter since they are the same
    kdtree_ = new cloud_kdtree::KdTreeANN (*cloud);
    ROS_INFO ("[LocalRadiusEstimation] Kd-tree created in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Allocate enough space for point indices and distances
  points_indices_.resize (cloud_radius_->points.size ());
  points_sqr_distances_.resize (cloud_radius_->points.size ());

  // Get the nearest neighbors for all points to be fitted
  ts = ros::Time::now ();
  int k_min = max_nn_, k_max = 0;
  for (size_t cp = 0; cp < cloud_radius_->points.size (); cp++)
  {
    kdtree_->radiusSearch (cp, radius_, points_indices_[cp], points_sqr_distances_[cp], max_nn_);
    int k = points_indices_[cp].size ();
    if (k > k_max) k_max = k;
    if (k < k_min) k_min = k;
  }
  ROS_INFO ("[LocalRadiusEstimation] Nearest neighbors in a radius of %g (maxed at %d) found in %g seconds.", radius_, max_nn_, (ros::Time::now () - ts).toSec ());
  ROS_INFO ("[LocalRadiusEstimation] Neighborhood sizes varied from %d to %d.", k_min, k_max);

  int cIdx = getChannelIndex(cloud, "curvature");
  if (rmin2curvature_)
    if (cIdx == -1)
      ROS_ERROR ("[LocalRadiusEstimation] Overwriting of curvature values was requested but the channel doesn't exist!");
    else
      ROS_WARN ("[LocalRadiusEstimation] Overwriting curvature values with the estimated minimal local radius (r_min)!");

  // Estimate radiuses
  ts = ros::Time::now ();
  // TODO parallelize!
  for (size_t cp = 0; cp < cloud_radius_->points.size (); cp++)
  {
    // Initialize minimum and maximum angle values in each distance bin
    for (int di=0; di<distance_div_; di++)
    {
      min_max_angle_by_dist[di][0] = +DBL_MAX;
      min_max_angle_by_dist[di][1] = -DBL_MAX;
    }

    // Go though all neighbors
    for (size_t ni = 1; ni < points_indices_[cp].size (); ni++)
    {
      // compute angle between the two lines going through the normalized normals (disregard orientation!)
      double cosine = cloud_radius_->channels[nxIdx+0].values[cp] * cloud_radius_->channels[nxIdx+0].values[points_indices_[cp][ni]] +
                      cloud_radius_->channels[nxIdx+1].values[cp] * cloud_radius_->channels[nxIdx+1].values[points_indices_[cp][ni]] +
                      cloud_radius_->channels[nxIdx+2].values[cp] * cloud_radius_->channels[nxIdx+2].values[points_indices_[cp][ni]];
      // TODO: function for dot product of 2 normals?
      /// @NOTE: this should take care of NaN:
      if (cosine > 1) cosine = 1;
      if (cosine < -1) cosine = -1;
      double angle = acos (cosine);
      if (angle > M_PI/2) angle = M_PI - angle;
      //cerr << cloud_geometry::angles::to_degrees(angle) << " and ";

      // Compute point to point distance
      double dist = sqrt (points_sqr_distances_[cp][ni]);

      // compute bin
      int bin_d = (int) floor (distance_div_ * dist / radius_);

      // update min-max values for distance bins
      if (min_max_angle_by_dist[bin_d][0] > angle) min_max_angle_by_dist[bin_d][0] = angle;
      if (min_max_angle_by_dist[bin_d][1] < angle) min_max_angle_by_dist[bin_d][1] = angle;
    }

    // Estimate radius from min and max lines
    double Amint_Amin = 0, Amint_d = 0;
    double Amaxt_Amax = 0, Amaxt_d = 0;
    for (int di=0; di<distance_div_; di++)
    {
      // compute the members of D'*D*r = D'*A
      if (min_max_angle_by_dist[di][1] >= 0)
      {
        double p_min = min_max_angle_by_dist[di][0];
        double p_max = min_max_angle_by_dist[di][1];
        double f = (di+0.5)*radius_/distance_div_;
        Amint_Amin += p_min * p_min;
        Amint_d += p_min * f;
        Amaxt_Amax += p_max * p_max;
        Amaxt_d += p_max * f;
      }
    }
    double max_radius;
    if (Amint_Amin == 0) max_radius = plane_radius_;
    else max_radius = min (Amint_d/Amint_Amin, plane_radius_);
    double min_radius;
    if (Amaxt_Amax == 0) min_radius = plane_radius_;
    else min_radius = min (Amaxt_d/Amaxt_Amax, plane_radius_);

    // Save extra dimensions
    cloud_radius_->channels[rIdx+0].values[cp] = min_radius;
    cloud_radius_->channels[rIdx+1].values[cp] = max_radius;
    cloud_radius_->channels[rIdx+2].values[cp] = max_radius - min_radius;

    // Overwrite curvature values if needed
    if (rmin2curvature_ && cIdx != -1)
      cloud_radius_->channels[cIdx].values[cp] = min_radius;

    // add class label
    if (point_label_ != -1)
      cloud_radius_->channels[labelIdx].values[cp] = point_label_;
    else
    {
      /// TODO: "heuristics"
    }
  }
  ROS_INFO ("[LocalRadiusEstimation] Radius estimation done in %g seconds.", (ros::Time::now () - ts).toSec ());

  // Finish
  ROS_INFO ("[LocalRadiusEstimation] Processed point cloud in %g seconds.", (ros::Time::now () - global_time).toSec ());
  return std::string("ok");
}

boost::shared_ptr<const LocalRadiusEstimation::OutputType> LocalRadiusEstimation::output ()
  {return cloud_radius_;}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <LocalRadiusEstimation> (argc, argv);
}
#endif
