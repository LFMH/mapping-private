#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/noise_removal.h>

using namespace std;
using namespace cloud_algos;

void StatisticalNoiseRemoval::init (ros::NodeHandle& nh)
{
  nh_ = nh;
}

void StatisticalNoiseRemoval::pre ()
{
  nh_.param("alpha", alpha_, alpha_);
  nh_.param("neighborhood_size", neighborhood_size_, neighborhood_size_);
  nh_.param("min_nr_pts", min_nr_pts_, min_nr_pts_);
}

void StatisticalNoiseRemoval::post ()
{

}

std::vector<std::string> StatisticalNoiseRemoval::requires ()
{
  std::vector<std::string> requires;
  // requires 3D coordinates
  requires.push_back("x");
  requires.push_back("y");
  requires.push_back("z");
  return requires;
}

std::vector<std::string> StatisticalNoiseRemoval::provides ()
{
  std::vector<std::string> provides;
  // provides filtered points
  provides.push_back("x");
  provides.push_back("y");
  provides.push_back("z");
  return provides;
}

std::string StatisticalNoiseRemoval::process (const boost::shared_ptr<const StatisticalNoiseRemoval::InputType>& cloud)
{
  // TODO where to put this?
  clear ();

  // Checks
  if (neighborhood_size_ < 2 || alpha_ < 0)
  {
    if (verbosity_level_ > -2) ROS_ERROR ("[StatisticalNoiseRemoval] A STD limit of %g and/or a neighborhood of size %d makes no sense!", alpha_, neighborhood_size_);
    return std::string("ERROR: Not enough neighbors requested!");
  }

  // Checks
  if (neighborhood_size_ > (int)cloud->points.size ())
  {
    if (verbosity_level_ > -2) ROS_ERROR ("[StatisticalNoiseRemoval] %d nearest neighbors (including self) requested, but only %d points in total!", neighborhood_size_, (int)cloud->points.size ());
    return std::string("ERROR: Not enough points in the cloud (or too many neighbors requested)!");
  }

  // Timers
  ros::Time global_time = ros::Time::now ();
  ros::Time ts;

  // Create Kd-Tree
  if (kdtree_ == NULL)
  {
    ts = ros::Time::now ();
    /// @NOTE: kd-tree for the original PCD... doesn't matter since they are the same
    kdtree_ = new cloud_kdtree::KdTreeANN (*cloud);
    if (verbosity_level_ > 0) ROS_INFO ("[StatisticalNoiseRemoval] Kd-tree created in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Allocate enough space for point indices and distances
  points_indices_.resize (cloud->points.size ());
  points_sqr_distances_.resize (cloud->points.size ());

  // Get the nearest neighbors for all points to be fitted
  ts = ros::Time::now ();
  //int k_min = max_nn_, k_max = 0;
  for (size_t cp = 0; cp < cloud->points.size (); cp++)
  {
    kdtree_->nearestKSearch (cp, neighborhood_size_, points_indices_[cp], points_sqr_distances_[cp]);
    //kdtree_->radiusSearch (cp, radius_, points_indices_[cp], points_sqr_distances_[cp], max_nn_);
    //int k = points_indices_[cp].size ();
    //if (k > k_max) k_max = k;
    //if (k < k_min) k_min = k;
  }
  if (verbosity_level_ > 0) ROS_INFO ("[StatisticalNoiseRemoval] %d nearest neighbors (including self) for all points found in %g seconds.", neighborhood_size_, (ros::Time::now () - ts).toSec ());
  //ROS_INFO ("[StatisticalNoiseRemoval] Nearest neighbors in a radius of %g (maxed at %d) found in %g seconds.", radius_, max_nn_, (ros::Time::now () - ts).toSec ());
  //ROS_INFO ("[StatisticalNoiseRemoval] Neighborhood sizes varied from %d to %d.", k_min, k_max);

  // Go through all the points and compute statistics
  ts = ros::Time::now ();
  vector<double> avg_distances (cloud->points.size (), 0.0);
  // TODO parallelize!
  for (size_t cp = 0; cp < cloud->points.size (); cp++)
  {
    // for all neighbors (so skip the first in points_indices_[cp] because it is cp itself)
    for (int ni = 1; ni < neighborhood_size_; ni++)
    {
      avg_distances[cp] += sqrt (points_sqr_distances_[cp][ni]);
    }
    avg_distances[cp] /= neighborhood_size_-1;
  }
  double sum = 0, sq_sum = 0;
  for (size_t cp = 0; cp < cloud->points.size (); cp++)
  {
    sum += avg_distances[cp];
    sq_sum += avg_distances[cp] * avg_distances[cp];
  }
  double mean = sum / cloud->points.size ();
  double variance = sq_sum / cloud->points.size () - mean * mean;
  double stddev = sqrt (variance);
  if (verbosity_level_ > 0) ROS_INFO ("[StatisticalNoiseRemoval] Computed mean (%g) and std (%g) in %g seconds.", mean, stddev, (ros::Time::now () - ts).toSec ());

  // Copy the necessary data from the original PCD
  ts = ros::Time::now ();
  cloud_denoise_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
  cloud_denoise_->header = cloud->header;
  cloud_denoise_->points.reserve(cloud->points.size ());
  cloud_denoise_->channels = cloud->channels;
  int point_count = 0;
  for (size_t cp = 0; cp < cloud->points.size (); cp++)
  {
    if (fabs (avg_distances[cp] - mean) < alpha_ * stddev)
    {
      cloud_denoise_->points.push_back (cloud->points[cp]);
      for (unsigned d = 0; d < cloud->channels.size (); d++)
        cloud_denoise_->channels[d].values[point_count] = cloud->channels[d].values[cp];
      point_count++;
    }
  }
  for (unsigned d = 0; d < cloud->channels.size (); d++)
    cloud_denoise_->channels[d].values.resize (point_count);
  if (verbosity_level_ > 0) ROS_INFO ("[StatisticalNoiseRemoval] Selected %d/%d points (%g\%%) in %g seconds.", point_count, (int)cloud->points.size (), 100*(point_count/(double)cloud->points.size ()), (ros::Time::now () - ts).toSec ());

  // Finish
  if (verbosity_level_ > 0) ROS_INFO ("[StatisticalNoiseRemoval] Cleared cloud in %g seconds.", (ros::Time::now () - global_time).toSec ());

  // Check if the size limit is satisfied
  if (point_count < min_nr_pts_)
  {
    //cloud_denoise_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
    if (verbosity_level_ > -1) ROS_WARN ("[StatisticalNoiseRemoval] Number of points smaller than the threshold of %d. Invalid output!", min_nr_pts_);
    output_valid_ = false;
    return std::string("output size check failed (see min_nr_pts parameter)");
  }
  else
  {
    output_valid_ = true;
    return std::string("ok");
  }
}

boost::shared_ptr<const StatisticalNoiseRemoval::OutputType> StatisticalNoiseRemoval::output ()
  {return cloud_denoise_;}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <StatisticalNoiseRemoval> (argc, argv);
}
#endif
