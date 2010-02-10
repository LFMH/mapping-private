#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/mls_fit.h>

#define THETA(cp,index)  exp( - (points_sqr_distances_[(cp)][(index)]) / (sqr_gauss_param_) )

/// TODO request this version from Radu :)
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/** \brief Get a u-v-n coordinate system that lies on a plane defined by its normal
  * \param plane_coeff the plane coefficients (containing n, the plane normal)
  * \param u the resultant u direction
  * \param v the resultant v direction
  */
inline void
  getCoordinateSystemOnPlane (const Eigen::Vector4d &plane_coeff, Eigen::Vector3d &u, Eigen::Vector3d &v)
{
  // Initialize normalized u vector with "random" values not parallel with normal
  u (1) = 0;
  if (fabs (plane_coeff (0)) != 1)
  {
    u (0) = 1;
    u (2) = 0;
  }
  else
  {
    u (0) = 0;
    u (2) = 1;
  }

  // Compute the v vector and normalize it
  v (0) = plane_coeff (1) * u (2) - plane_coeff (2) * u (1);
  v (1) = plane_coeff (2) * u (0) - plane_coeff (0) * u (2);
  v (2) = plane_coeff (0) * u (1) - plane_coeff (1) * u (0);
  double v_length = sqrt (v (0) * v (0) + v (1) * v (1) + v (2) * v (2));
  v (0) /= v_length;
  v (1) /= v_length;
  v (2) /= v_length;

  // Recompute u and normalize it
  u (0) = v (1) * plane_coeff (2) - v (2) * plane_coeff (1);
  u (1) = v (2) * plane_coeff (0) - v (0) * plane_coeff (2);
  u (2) = v (0) * plane_coeff (1) - v (1) * plane_coeff (0);
  double u_length = sqrt (u (0) * u (0) + u (1) * u (1) + u (2) * u (2));
  u (0) /= u_length;
  u (1) /= u_length;
  u (2) /= u_length;
}

void MovingLeastSquares::init (ros::NodeHandle &nh)
{
  // node handler and publisher
  nh_ = nh;
  pub_ = nh_.advertise <sensor_msgs::PointCloud> ("vis_mls_fit", 1);

  // allocating space for "global" matrices - TODO here?
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;
  weight_ = MatrixXf::Zero (max_nn_, max_nn_);
  P_.resize (nr_coeff_, max_nn_);
  f_vec_.resize (max_nn_);
  c_vec_.resize (nr_coeff_);
  P_weight_.resize (nr_coeff_, max_nn_);
  P_weight_Pt_.resize (nr_coeff_, nr_coeff_);
  inv_P_weight_Pt_.resize (nr_coeff_, nr_coeff_);
  inv_P_weight_Pt_P_weight_.resize (nr_coeff_, max_nn_);
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
  provides.push_back ("curvature");
  // provides number of neighbors
  //provides.push_back ("k");
  // provides boundary information if required
  //provides.push_back ("bp");
  // provides moment invariants if needed
  provides.push_back ("j1");
  provides.push_back ("j2");
  provides.push_back ("j3");
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
  bool copied_points = false; /// @TODO: this might be taken from point_generation_ (if safe.. since it's the default switch)
  bool complete_tree = false; /// TODO: if this actually turns out to help, we should specify this also when the tree is set

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
      /// @TODO get implementations from _Clouds/src/ResamplingMLS/RMLS.h (not important for general use) AND add averaging color, intensities, etc.
      //case FILLING:     cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case CLOSE:       cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case SURROUNDING: cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case ADAPTIVE:    cloud_fit_ = generateDownsampled(cloud, step_); break;
      //case FIXED_K:     cloud_fit_ = generateDownsampled(cloud, step_); break;
      default: // COPY
      {
        copied_points = true;
        filter_points_ = false; // no use of filtering them if they are the same as the measurements
        cloud_fit_->header   = cloud->header;
        cloud_fit_->points   = cloud->points;
        cloud_fit_->channels = cloud->channels;
      }
    }
    ROS_INFO ("Initial point positions generated in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Allocate the extra needed channels
  size_t original_chan_size = cloud_normals_.channels.size ();
  if (compute_moments_)
    cloud_fit_->channels.resize (original_chan_size + 7);
  else
    cloud_fit_->channels.resize (original_chan_size + 4);

  // Name the extra channels
  cloud_fit_->channels[original_chan_size + 0].name = "nx";
  cloud_fit_->channels[original_chan_size + 1].name = "ny";
  cloud_fit_->channels[original_chan_size + 2].name = "nz";
  cloud_fit_->channels[original_chan_size + 3].name = "curvature";
  if (compute_moments_)
  {
    cloud_fit_->channels[original_chan_size + 4].name = "j1";
    cloud_fit_->channels[original_chan_size + 5].name = "j2";
    cloud_fit_->channels[original_chan_size + 6].name = "j3";
  }

  // Resize the extra channels
  for (size_t d = original_chan_size; d < cloud_fit_.channels.size (); d++)
  {
    cloud_fit_->channels[d].values.resize (cloud_fit_->points.size ());
    ROS_INFO ("Added channel: %s", cloud_fit_->channels[d].name.c_str ());
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
  /// TODO done for parallelism - evaluate usefulness and maybe do it for other shared variables as well !!!
  ts = ros::Time::now ();
  /// @NOTE: the indexed version seems faster, as the point does not have to be converted to ANNpoint
  /// BUT: we are not using the PCD represented by the internal ANNpointArray, but the new one!
  /// ALSO: if the KD-Tree was created on only a subset of the cloud it doesn't work!
  if (complete_tree && copied_points)
  {
    /// TODO: check if this is really faster and leave it out if not...
    for (size_t i = 0; i < cloud_fit_->points.size (); i++)
      kdtree_->radiusSearch (i, radius_, points_indices_[i], points_sqr_distances_[i], max_nn_);
  }
  else
  {
    vector<geometry_msgs::Point32>::iterator pit = cloud_fit_.points.begin ();
    vector<vector<int> >::iterator iit = points_indices_.begin ();
    vector<vector<float> >::iterator dit = points_sqr_distances_.begin ();
    for (/*to avoid long/multiple lines*/; pit != cloud_fit_->points.end (); pit++, iit++, dit++)
      kdtree_->radiusSearch (*pit, radius_, *iit, *dit, max_nn_);
  }
  /// TODO compare to fixed k search with k = average neighborhood size
  ROS_INFO ("Nearest neighbors found in %g seconds.\n", (ros::Time::now () - ts).toSec ());

  // Go through all the points that have to be fitted
  ts = ros::Time::now ();
  ros::Time ts_tmp;
  double sum_ts_filter = 0.0;
  double sum_ts_tangent = 0.0;
  double sum_ts_polynomial = 0.0;
  double sum_ts_extra = 0.0;
  /// TODO #pragma omp parallel for schedule(dynamic) and boost::mutex m_lock_; .lock () / .unlock ()
  ///      which combination of parallelism/global variables is best?
  for (size_t cp = 0; cp < viewpoint_cloud_->points.size (); cp++)
  {
    // STEP1: Check neighborhood
    if (filter_points_)
    {
      ts_tmp = ros::Time::now ();
      /// @TODO get it from _Clouds/src/ResamplingMLS/RMLS.h (not important for general use)
      sum_ts_filter += (ros::Time::now () - ts_tmp).toSec ();
    }

    // STEP2: Get a good plane approximating the local surface and project point onto it (+ other features)
    ts_tmp = ros::Time::now ();
    Eigen::Vector4d plane_parameters; /// @NOTE: both () and [] is defined for Eigen vectors !!!
    double curvature, j1, j2, j3;
    /// @NOTE: while the neighborhood of the points from cloud_fit is used, the plane has to be computed from cloud
    switch (approximating_tangent_)
    {
      /// @TODO get them from _Clouds/src/ResamplingMLS/RMLS.h (not important for general use)
      //case WPCA:
      //case IWPCA:
      //case SAC:
      default: // PCA
      {
        /// TODO replace this and the moment invariant computing function by more optimal ones
        ///      that don't re-allocate normal and take/return centroid/cov-matrix
        cloud_geometry::nearest::computePointNormal (*cloud, points_indices_[cp], plane_parameters, curvature);
        // projecting point to be fitted onto the plane - TODO make inline referenced utility function
        double distance = plane_parameters(0)*cloud_fit_->points[cp].x + plane_parameters[1]*cloud_fit_->points[cp].y + plane_parameters[2]*cloud_fit_->points[cp].z + plane_parameters[3];
        cloud_fit_->points[cp].x -= distance * plane_parameters[0];
        cloud_fit_->points[cp].y -= distance * plane_parameters[1];
        cloud_fit_->points[cp].z -= distance * plane_parameters[2];
      }
    }
    sum_ts_tangent += (ros::Time::now () - ts_tmp).toSec ();

    // STEP3: Perform polynomial fitting
    if (polynomial_fit_)
    {
      // initialize timer
      ts_tmp = ros::Time::now ();

      // update neighborhood, since point was projected
      /// TODO: leaving this out, as it is not as important

      // TODO maybe put all these variables as global - unless it messes up parallelism !!!
      /// @NOTE: probably not worth having the result of plane_parameters.start<3>() saved...

      // get local coordinate system
      Eigen::Vector3d v = plane_parameters.start<3>().unitOrthogonal ();
      Eigen::Vector3d u = plane_parameters.start<3>().cross (v);
      // TODO is this faster or: getCoordinateSystemOnPlane (plane_parameters, u, v);

      // build up matrices for getting the coefficients
      Eigen::Vector3d de_meaned;
      double u_coord, v_coord, u_pow, v_pow;
      for (size_t i = points_sqr_distances_[cp].begin (); dit != points_sqr_distances_[cp].end (); dit++)
      {
        // (re-)compute weights
        weights_(i,i) = THETA(cp,i);

        // transforming coordinates
        de_meaned[0] = cloud->points[points_indices_[cp][i]].x - cloud_fit_->points[cp].x;
        de_meaned[1] = cloud->points[points_indices_[cp][i]].y - cloud_fit_->points[cp].y;
        de_meaned[2] = cloud->points[points_indices_[cp][i]].z - cloud_fit_->points[cp].z;
        // TODO which one is better, faster? dot, * or macro?
        u_coord = de_meaned.dot (u);
        v_coord = de_meaned.dot (v);
        // plane_parameters has 4 values, need only first 3
        f_vec_(i) = de_meaned.dot (plane_parameters.start<3>());
        //cerr << "f_vec[" << i << "] = " << f_vec[i] << endl;

        // compute members
        int j = 0;
        u_pow = 1;
        for (int ui=0; ui<=order_; ui++)
        {
          v_pow = 1;
          for (int vi=0; vi<=order_-ui; vi++)
          {
            P_(j++,i) = u_pow * v_pow;
            //cerr << (j-1) << ": u^" << ui << " * v^" << vi << " = " << P(j-1,i) << endl;
            v_pow *= v_coord;
          }
          u_pow *= u_coord;
        }
        //cerr << "==============" << endl;
      }

      // compute coefficients
      P_weight_ = P_ * weight_;
      P_weight_Pt_.part<Eigen::SelfAdjoint>() = P_weight_ * P_.transpose (); // TODO optimize: result is symmetrical... maybe concat with prev and use lazy()
      inv_P_weight_Pt_.computeInverse(&P_weight_Pt_); /// @NOTE: according to documentation, this is the optimal way (no alloc!)
      inv_P_weight_Pt_P_weight_ = inv_P_weight_Pt_ * P_weight_;
      c_vec_ = inv_P_weight_Pt_P_weight_ * f_vec_;

      // project point onto the surface - TODO: make (at least approximate) orthogonal projection
      if (c_vec_[0]*c_vec_[0] < points_sqr_distances_[cp][i]*3) /// maybe make some different check here !!!
      {
        //print_info(stderr, "Projection onto MLS surface along Darboux normal to the height at (0,0) of: "); print_value(stderr, "%g\n", c_vec[0]);
        cloud_fit_->points[cp].x += c_vec[0] * plane_parameters[0];
        cloud_fit_->points[cp].y += c_vec[0] * plane_parameters[1];
        cloud_fit_->points[cp].z += c_vec[0] * plane_parameters[2];
      }
      else
      {
        /// @NOTE: simple approximation of orthogonal projection, in case adjusting the height at (0,0) seems inappropriate
        // get the u and v coordinates of the closest point
        u_coord = P_(order_+1,0); // value of u^1*v^0 = u
        v_coord = P_(1,0);        // value of u^0*v^1 = v

        // compute the value of the polynomial at the nearest point
        double height = 0.0;
        double du = 0.0; // the partial derivative of the polynomial w.r.t. u at (u_coord,v_coord)
        double dv = 0.0; // the partial derivative of the polynomial w.r.t. v at (u_coord,v_coord)
        int j = 0;
        u_pow = 1;
        for (int ui=0; ui<=order_; ui++)
        {
          v_pow = 1;
          for (int vi=0; vi<=order_-ui; vi++)
          {
            height += c_vec_[j++] * u_pow * v_pow;
            v_pow *= v_coord;
          }
          u_pow *= u_coord;
        }

        // move the point to the corresponding surface point
        Eigen::Vector3d movement = u_coord * u + v_coord * v + h_coord * plane_parameters.start<3>();
        cloud_fit_->points[cp].x += movement[0];
        cloud_fit_->points[cp].y += movement[1];
        cloud_fit_->points[cp].z += movement[2];

        // IMPORTANT: update c_vec[order_+1] and c_vec[1] to hold
      }

      // "stop" timer
      sum_ts_polynomial += (ros::Time::now () - ts_tmp).toSec ();

      // re-compute surface normal - TODO: update curvature as well after polynomial fit?
      ts_tmp = ros::Time::now ();
      // c_vec[order_+1] and c_vec[1] is the partial derivative of the polynomial w.r.t. u and v respectively,
      // evaluated at the current point - which is the origin in case of projection along the normals, or (u_coord,v_coord)
      Eigen::Vector3d n_a = u + plane_parameters.start<3>() * c_vec[order_+1];
      Eigen::Vector3d n_b = v + plane_parameters.start<3>() * c_vec[1];
      plane_parameters.start<3>() = n_a.cross (nb).normalize ();
      /*ANNpoint n = annAllocPt(3);
      n[0] = -c_vec[order+1][0];
      n[1] = -c_vec[1][0];
      n[2] = 1;*/

      /**
       *  5   4   3   2   1   0
       * 2,0 1,1 1,0 0,2 0,1 0,0
       *  a   b   d   c   e   f
      **/

      /// NOT SURE THIS IS WORKING !!! CHECK IN PAPER
      /*if (order == 2)
      {
        // Calculating Gaussian curvature K and mean curvature H
        double d_sum = 1 + SQR(c_vec[3]);
        double e_sum = 1 + SQR(c_vec[1]);
        double div = SQR(d_sum + e_sum - 1);
        mls_point[10] = (4*(c_vec[5])*(c_vec[2]) - SQR(c_vec[4])) / div;
        mls_point[11] = (c_vec[5]*e_sum - c_vec[4]*c_vec[3]*c_vec[1] + c_vec[2]*d_sum) / div;
        //cerr << "K=" << mls_point[10] << ", H=" << mls_point[11] << endl;
      }*/
    }

    // STEP4: Compute local features
    if (viewpoint_cloud_) /// @NOTE: the point where the normal is placed is from cloud_fit
      cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud_fit_->points[cp], viewpoint_cloud_);
    cloud_fit_->channels[original_chan_size + 0].values[cp] = plane_parameters (0);
    cloud_fit_->channels[original_chan_size + 1].values[cp] = plane_parameters (1);
    cloud_fit_->channels[original_chan_size + 2].values[cp] = plane_parameters (2);
    cloud_fit_->channels[original_chan_size + 3].values[cp] = curvature;
    sum_ts_tangent += (ros::Time::now () - ts_tmp).toSec ();
    if (compute_moments_)
    {
      ts_tmp = ros::Time::now ();
      /// @NOTE: the moments have to be computed from the original cloud OR from cloud_fit *after* fitting (TODO?)
      cloud_geometry::nearest::computeMomentInvariants (*cloud, points_indices_[cp], j1, j2, j3);
      cloud_fit_->channels[original_chan_size + 4].values[cp] = j1;
      cloud_fit_->channels[original_chan_size + 5].values[cp] = j2;
      cloud_fit_->channels[original_chan_size + 6].values[cp] = j3;
      sum_ts_extra += (ros::Time::now () - ts_tmp).toSec ();
    }
    // TODO: bool cloud_geometry::nearest::isBoundaryPoint (const sensor_msgs::PointCloud &points, int q_idx, const std::vector<int> &neighbors, const Eigen::Vector3d& u, const Eigen::Vector3d& v, double angle_threshold)
  }
  ROS_INFO ("Fitted points in %g seconds.\n", (ros::Time::now () - ts).toSec ());
  ROS_INFO ("- time spent filtering points: %g seconds.\n", sum_ts_filter);
  ROS_INFO ("- time spent approximating tangent: %g seconds.\n", sum_ts_tangent);
  ROS_INFO ("- time spent computing polynomial: %g seconds.\n", sum_ts_polynomial);
  ROS_INFO ("- time spent computing extra features: %g seconds.\n", sum_ts_extra);

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
