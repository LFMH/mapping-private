#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/mls_fit.h>

void MovingLeastSquares::init (ros::NodeHandle &nh)
{
  nh_ = nh;
  pub_ = nh_.advertise <sensor_msgs::PointCloud> ("vis_mls_fit", 1);
}

std::vector<std::string> MovingLeastSquares::pre ()
{
  std::vector<std::string> requires;
  // requires 3D coordinates
  requires.push_back("x");
  requires.push_back("y");
  requires.push_back("z");
  return requires;
}

std::vector<std::string> MovingLeastSquares::post ()
{
  // TODO clear/implement these
  std::vector<std::string> provides(3);
  // provides updated 3D coordinates
  provides.push_back ("x");
  provides.push_back ("y");
  provides.push_back ("z");
  // provides normals computed by the user-defined method complexity
  provides.push_back ("nx");
  provides.push_back ("ny");
  provides.push_back ("nz");
  // provides curvature
  provides.push_back ("c");
  // provides number of neighbors
  provides.push_back ("k");
  // provides boundary information if required
  provides.push_back ("bp");
  return provides;
}

std::string MovingLeastSquares::process (sensor_msgs::PointCloudConstPtr cloud)
{
  // Figure out the viewpoint value in the cloud_frame frame
  geometry_msgs::PointStamped viewpoint_cloud;
  getCloudViewPoint (cloud->header.frame_id, viewpoint_cloud, tf_);

  // Timers
  ros::Time global_time = ros::Time::now ();
  ros::Time ts;

  // Conditions for faster nearest neighbor search
  bool copied_points = false;
  bool complete_tree = false; /// @TODO: if this actually turns out to help, we should specify this also when the tree is set

  // Create Kd-Tree
  if (kdtree_ == NULL)
  {
    ts = ros::Time::now ();
    complete_tree = true;
    /// @TODO: should we also set epsilon / bucket size?
    kdtree_ = new cloud_kdtree::KdTreeANN (cloud);
    ROS_INFO ("Kd-tree created in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Check if points have been provided to be fit to the cloud
  if (cloud_fit_ == NULL)
  {
    // decide on the method used to generate the points
    ts = ros::Time::now ();
    switch (point_generation_)
    {
      // TODO get implementations from _Clouds/src/ResamplingMLS/RMLS.h
      //case FILLING:     cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case CLOSE:       cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case SURROUNDING: cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case ADAPTIVE:    cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case FIXED_K:     cloud_fit_ = generateDownsampled(cloud, step_); break;
      default: // COPY
      {
        copied_points = true;
        cloud_fit_->header   = cloud->header;
        cloud_fit_->points   = cloud->points;
        cloud_fit_->channels = cloud->channels;
      }
    }
    ROS_INFO ("Initial point positions generated in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Allocate the extra needed channels
  if (compute_moments_)
    cloud_normals_.channels.resize (original_chan_size + 7);     // Allocate 7 more channels
  else
    cloud_normals_.channels.resize (original_chan_size + 4);     // Allocate 4 more channels
  cloud_normals_.channels[original_chan_size + 0].name = "nx";
  cloud_normals_.channels[original_chan_size + 1].name = "ny";
  cloud_normals_.channels[original_chan_size + 2].name = "nz";
  cloud_normals_.channels[original_chan_size + 3].name = "curvature";
  if (compute_moments_)
  {
    cloud_normals_.channels[original_chan_size + 4].name = "j1";
    cloud_normals_.channels[original_chan_size + 5].name = "j2";
    cloud_normals_.channels[original_chan_size + 6].name = "j3";
  }
  for (unsigned int d = original_chan_size; d < cloud_normals_.channels.size (); d++)
  {
    if (downsample_ != 0)
      cloud_normals_.channels[d].values.resize (cloud_down_.points.size ());
    else
      cloud_normals_.channels[d].values.resize (cloud->points.size ());
  }

  // Allocate enough space for point indices and distances
  points_indices_.resize (cloud_fit_.points.size ());
  points_sqr_distances_.resize (cloud_fit_.points.size ());
  /*for (size_t i = 0; i < cloud_fit_.points.size (); i++)
  {
    // needed only for K searches
    points_indices_[i].resize (k_);
    points_sqr_distances_[i].resize (k_);
  }*/

  // Get the nearest neighbors for all points to be fitted
  ts = ros::Time::now ();
  /// @NOTE: the indexed version seems faster, as the point does not have to be converted to ANNpoint
  /// BUT: we are not using the PCD represented by the internal ANNpointArray, but the new one!
  /// ALSO: if the KD-Tree was created on only a subset of the cloud it doesn't work!
  if (complete_tree && copied_points)
  {
    /// @TODO: check if this is really faster and leave it out if not...
    for (size_t i = 0; i < cloud_normals_.points.size (); i++)
      kdtree_->radiusSearch (i, radius_, points_indices_[i], points_sqr_distances_[i], max_nn_);
  }
  else
  {
    vector<geometry_msgs::Point32>::iterator pit = cloud_fit_.points.begin ();
    vector<vector<int> >::iterator iit = points_indices_.begin ();
    vector<vector<int> >::iterator dit = points_sqr_distances_.begin ();
    for (/*to avoid long/multiple lines*/; pit != cloud_fit_.points.end (); pit++, iit++, dit++)
      kdtree_->radiusSearch (*pit, radius_, *iit, *dit, max_nn_);
  }
  ROS_INFO ("Nearest neighbors found in %g seconds.\n", (ros::Time::now () - ts).toSec ());

  // Go through all the points that have to be fitted
  // TODO #pragma omp parallel for schedule(dynamic)
  // TODO boost::mutex m_lock_; .lock () / .unlock ()
  for (int i = 0; i < (int)cloud_normals_.points.size (); i++)
  {
    // STEP1: Check neighborhood

    // Compute the point normals (nx, ny, nz), surface curvature estimates (c), and moment invariants (j1, j2, j3)
    Eigen::Vector4d plane_parameters;
    double curvature, j1, j2, j3;
    cloud_geometry::nearest::computePointNormal (cloud_normals_, points_indices_[i], plane_parameters, curvature);

    if (compute_moments_)
      cloud_geometry::nearest::computeMomentInvariants (cloud_normals_, points_indices_[i], j1, j2, j3);

    cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud_normals_.points[i], viewpoint_cloud);

    cloud_normals_.channels[original_chan_size + 0].values[i] = plane_parameters (0);
    cloud_normals_.channels[original_chan_size + 1].values[i] = plane_parameters (1);
    cloud_normals_.channels[original_chan_size + 2].values[i] = plane_parameters (2);
    cloud_normals_.channels[original_chan_size + 3].values[i] = curvature;
    if (compute_moments_)
    {
      cloud_normals_.channels[original_chan_size + 4].values[i] = j1;
      cloud_normals_.channels[original_chan_size + 5].values[i] = j2;
      cloud_normals_.channels[original_chan_size + 6].values[i] = j3;
    }
  }


  // Finish
  ROS_INFO ("MLS fit done in %g seconds.", (ros::Time::now () - global_time).toSec ());
  pub_.publish (cloud_fit_);
  return std::string("ok");
}

MovingLeastSquares::OutputType MovingLeastSquares::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <MovingLeastSquares> (argc, argv);
}
#endif


