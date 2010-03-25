#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/mls_fit.h>

#define THETA(cp,index)  exp( - (points_sqr_distances_[(cp)][(index)]) / (sqr_gauss_param_) )

using namespace std;
using namespace cloud_algos;


/// TODO request this version from Radu? :)
/*
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Get a u-v-n coordinate system that lies on a plane defined by its normal
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
*/

void MovingLeastSquares::init (ros::NodeHandle &nh)
{
  // node handler and publisher
  // TODO needed?
  //nh_ = nh;
  //pub_ = nh_.advertise <sensor_msgs::PointCloud> ("vis_mls_fit", 1);
}

void MovingLeastSquares::pre ()
{
  cloud_fit_.reset ();
}

void MovingLeastSquares::post ()
{

}

std::vector<std::string> MovingLeastSquares::requires ()
{
  std::vector<std::string> requires;
  // requires 3D coordinates
  requires.push_back("x");
  requires.push_back("y");
  requires.push_back("z");
  return requires;
}

std::vector<std::string> MovingLeastSquares::provides ()
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

std::string MovingLeastSquares::process (const boost::shared_ptr<const MovingLeastSquares::InputType> &cloud)
{
  // TODO where to put this?
  clear ();
  nr_coeff_ = (order_ + 1) * (order_ + 2) / 2;

  // Defines for experiments, should work with all commented
  //#define PARALLEL
  //#define PARTIAL_TIMES
  //#define GLOBAL
  //#define INVERSE

  #ifdef GLOBAL
  // allocating space for "global" matrices - TODO here?
  //Eigen::MatrixXd weight_ = Eigen::MatrixXd::Zero (max_nn_, max_nn_);
  Eigen::VectorXd weight_vec_(max_nn_);
  Eigen::MatrixXd P_(nr_coeff_, max_nn_);
  Eigen::VectorXd f_vec_(max_nn_);
  Eigen::VectorXd c_vec_; //(nr_coeff_);
  Eigen::MatrixXd P_weight_; //(nr_coeff_, max_nn_);
  Eigen::MatrixXd P_weight_Pt_(nr_coeff_, nr_coeff_);
  Eigen::MatrixXd inv_P_weight_Pt_(nr_coeff_, nr_coeff_);
  //Eigen::MatrixXd inv_P_weight_Pt_P_weight_; //(nr_coeff_, max_nn_);
  /*//weight_ = Eigen::MatrixXd::Zero (max_nn_, max_nn_);
  weight_vec_.resize (max_nn_);
  P_.resize (nr_coeff_, max_nn_);
  f_vec_.resize (max_nn_);
  P_weight_Pt_.resize (nr_coeff_, nr_coeff_);
  inv_P_weight_Pt_.resize (nr_coeff_, nr_coeff_);*/
  #endif

  // TODO Figure out the viewpoint value in the cloud_frame frame
  //geometry_msgs::PointStamped viewpoint_cloud;
  //getCloudViewPoint (cloud->header.frame_id, viewpoint_cloud, tf_);
  if (viewpoint_cloud_ == NULL)
  {
    /// @NOTE: assume sensor position to be at the origin - TODO modify point_cloud_assembler...
    viewpoint_cloud_ = new geometry_msgs::PointStamped ();
    viewpoint_cloud_->point.x = 0;
    viewpoint_cloud_->point.y = 0;
    viewpoint_cloud_->point.z = 0;
  }

  // Timers
  ros::Time global_time = ros::Time::now ();
  ros::Time ts;

  // Conditions for faster nearest neighbor search
  bool copied_points = false; /// TODO: this might be taken from point_generation_ (if safe.. since it's the default switch)
  bool complete_tree = false; /// TODO: if this actually turns out to help, we should specify this also when the tree is set

  // Create Kd-Tree
  if (kdtree_ == NULL)
  {
    ts = ros::Time::now ();
    complete_tree = true;
    kdtree_ = new cloud_kdtree::KdTreeANN (*cloud);
    ROS_INFO ("Kd-tree created in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Check if points have been provided to be fit to the cloud
  if (not cloud_fit_)  // REAL TODO: is this OK?
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
        ROS_INFO ("Initial point positions are copied from the original cloud.");
        copied_points = true;
        filter_points_ = false; // no use of filtering them if they are the same as the measurements
        cloud_fit_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud);
        cloud_fit_->header   = cloud->header;
        cloud_fit_->points   = cloud->points;
        cloud_fit_->channels = cloud->channels;
      }
    }
    ROS_INFO ("Initial point positions generated in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Allocate the extra needed channels
  size_t original_chan_size = cloud_fit_->channels.size ();
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
  for (size_t d = original_chan_size; d < cloud_fit_->channels.size (); d++)
  {
    cloud_fit_->channels[d].values.resize (cloud_fit_->points.size ());
    ROS_INFO ("Added channel: %s", cloud_fit_->channels[d].name.c_str ());
  }

  // Allocate enough space for point indices and distances
  points_indices_.resize (cloud_fit_->points.size ());
  points_sqr_distances_.resize (cloud_fit_->points.size ());
  /*for (size_t i = 0; i < cloud_fit_->points.size (); i++)
  {
    // needed only for K searches
    points_indices_[i].resize (k_);
    points_sqr_distances_[i].resize (k_);
  }*/

  // Get the nearest neighbors for all points to be fitted -- for parallelism
  ts = ros::Time::now ();
  /// @NOTE: the indexed version seems faster, as the point does not have to be converted to ANNpoint
  /// BUT: we are not using the PCD represented by the internal ANNpointArray, but the new one!
  /// ALSO: if the KD-Tree was created on only a subset of the cloud it doesn't work!
  if (complete_tree && copied_points)
  {
    /// @NOTE: this is a tiny bit faster :)
    for (size_t i = 0; i < cloud_fit_->points.size (); i++)
      kdtree_->radiusSearch (i, radius_, points_indices_[i], points_sqr_distances_[i], max_nn_);
  }
  else
  {
    vector<geometry_msgs::Point32>::iterator pit = cloud_fit_->points.begin ();
    vector<vector<int> >::iterator iit = points_indices_.begin ();
    vector<vector<float> >::iterator dit = points_sqr_distances_.begin ();
    for (/*to avoid long/multiple lines*/; pit != cloud_fit_->points.end (); pit++, iit++, dit++)
      kdtree_->radiusSearch (*pit, radius_, *iit, *dit, max_nn_);
  }
  ROS_INFO ("Nearest neighbors found in %g seconds.", (ros::Time::now () - ts).toSec ());

  // Go through all the points that have to be fitted
  ts = ros::Time::now ();
  #ifdef PARTIAL_TIMES
  ros::Time ts_tmp;
  double sum_ts_filter = 0.0;
  double sum_ts_tangent = 0.0;
  double sum_ts_polynomial = 0.0;
  double sum_ts_extra = 0.0;
  #endif
  /// TODO #pragma omp parallel for schedule(dynamic) and boost::mutex m_lock_; .lock () / .unlock ()
  ///      which combination of parallelism/global variables is best?
  #ifndef PARALLEL
  #ifdef INVERSE
  int zero_determinant = 0, not_inverse = 0;
  double max_bad_det = 0.0, min_good_det = 0.01;
  #endif
  int k_min = max_nn_, k_max = 0, not_enough_nn = 0, nr_orthogonal = 0;
  #else
  #pragma omp parallel for schedule(dynamic)
  #endif
  for (size_t cp = 0; cp < cloud_fit_->points.size (); cp++)
  {
    // STEP0: Init
    int k = points_indices_[cp].size (); // TODO: save k as a channel, and set it + min-max when creating neighborhoods
    #ifndef PARALELL
    if (k > k_max) k_max = k;
    if (k < k_min) k_min = k;
    #endif
    // TODO: perhaps move nn search here if not parallelized

    // STEP1: Check neighborhood
    if (filter_points_)
    {
      #ifdef PARTIAL_TIMES
      ts_tmp = ros::Time::now ();
      #endif
      /// @TODO get it from _Clouds/src/ResamplingMLS/RMLS.h (not important for general use)
      #ifdef PARTIAL_TIMES
      sum_ts_filter += (ros::Time::now () - ts_tmp).toSec ();
      #endif
    }
    //cerr << "original point: " << cloud_fit_->points[cp].x << " " << cloud_fit_->points[cp].y << " " << cloud_fit_->points[cp].z << endl;

    // STEP2: Get a good plane approximating the local surface and project point onto it (+ other features)
    #ifdef PARTIAL_TIMES
    ts_tmp = ros::Time::now ();
    #endif
    Eigen::Vector4d plane_parameters; /// @NOTE: both () and [] is defined for Eigen vectors !!!
    double curvature, j1, j2, j3;
    /// @NOTE: while the neighborhood of the points from cloud_fit is used, the plane has to be computed from cloud
    switch (approximating_tangent_)
    {
      /// TODO get them from _Clouds/src/ResamplingMLS/RMLS.h
      //case WPCA:
      //case IWPCA:
      //case SAC:
      default: // PCA
      {
        /// TODO replace this and the moment invariant computing function by more optimal ones
        ///      that don't re-allocate normal and take/return centroid/cov-matrix
        cloud_geometry::nearest::computePointNormal (*cloud, points_indices_[cp], plane_parameters, curvature);
        // projecting point to be fitted onto the plane - TODO make inline referenced utility function
        double distance = plane_parameters[0]*cloud_fit_->points[cp].x + plane_parameters[1]*cloud_fit_->points[cp].y + plane_parameters[2]*cloud_fit_->points[cp].z + plane_parameters[3];
        cloud_fit_->points[cp].x -= distance * plane_parameters[0];
        cloud_fit_->points[cp].y -= distance * plane_parameters[1];
        cloud_fit_->points[cp].z -= distance * plane_parameters[2];
      }
    }
    #ifdef PARTIAL_TIMES
    sum_ts_tangent += (ros::Time::now () - ts_tmp).toSec ();
    #endif
    //cerr << "plane_parameters:" << endl << plane_parameters << endl;
    //cerr << "projd point: " << cloud_fit_->points[cp].x << " " << cloud_fit_->points[cp].y << " " << cloud_fit_->points[cp].z << endl;

    // STEP3: Perform polynomial fitting
    if (k < nr_coeff_)
    {
      #ifndef PARALELL
        not_enough_nn++;
      #endif
    }
    else
    if (polynomial_fit_)
    {
      // initialize timer
      #ifdef PARTIAL_TIMES
      ts_tmp = ros::Time::now ();
      #endif

      // update neighborhood, since point was projected
      //kdtree_->radiusSearch (cloud_fit_->points[cp], radius_, points_indices_[cp], points_sqr_distances_[cp], max_nn_);
      //k = points_indices_.size ();
      /// TODO: previous has a bug... and:
      /// @NOTE: updating only distances for the weights right now, for speed
      for (size_t ni = 0; ni < points_indices_[cp].size (); ni++)
      {
        points_sqr_distances_[cp][ni] = (cloud->points[cp].x - cloud_fit_->points[cp].x) * (cloud->points[cp].x - cloud_fit_->points[cp].x)
                                      + (cloud->points[cp].y - cloud_fit_->points[cp].y) * (cloud->points[cp].y - cloud_fit_->points[cp].y)
                                      + (cloud->points[cp].z - cloud_fit_->points[cp].z) * (cloud->points[cp].z - cloud_fit_->points[cp].z);
      }

      //#ifdef PARALELL
      #ifndef GLOBAL
      //Eigen::MatrixXd weight_ = Eigen::MatrixXd::Zero (k, k);
      Eigen::VectorXd weight_vec_(k);
      Eigen::MatrixXd P_(nr_coeff_, k);
      Eigen::VectorXd f_vec_(k);
      Eigen::VectorXd c_vec_; //(nr_coeff_);
      Eigen::MatrixXd P_weight_; //(nr_coeff_, k);
      Eigen::MatrixXd P_weight_Pt_(nr_coeff_, nr_coeff_);
      #ifdef INVERSE
      Eigen::MatrixXd inv_P_weight_Pt_(nr_coeff_, nr_coeff_);
      #endif
      //Eigen::MatrixXd inv_P_weight_Pt_P_weight_; //(nr_coeff_, k);
      /*//weight_ = Eigen::MatrixXd::Zero (k, k);
      weight_vec_.resize (k);
      P_.resize (nr_coeff_, k);
      f_vec_.resize (k);
      P_weight_Pt_.resize (nr_coeff_, nr_coeff_);
      #ifdef INVERSE
      inv_P_weight_Pt_.resize (nr_coeff_, nr_coeff_);
      #endif
      */
      #endif

      // get local coordinate system (Darboux frame)
      Eigen::Vector3d v = plane_parameters.start<3>().unitOrthogonal ();
      Eigen::Vector3d u = plane_parameters.start<3>().cross (v);

      // --[ Build up matrices for getting the coefficients ]--

      // go through neighbors, transform them in the local coordinate system, save height and the evaluation of the polynome's terms
      Eigen::Vector3d de_meaned;
      double u_coord, v_coord, u_pow, v_pow;
      for (size_t i = 0; i < points_indices_[cp].size (); i++)
      {
        // (re-)compute weights
        weight_vec_(i) = THETA(cp,i);

        // transforming coordinates
        de_meaned[0] = cloud->points[points_indices_[cp][i]].x - cloud_fit_->points[cp].x;
        de_meaned[1] = cloud->points[points_indices_[cp][i]].y - cloud_fit_->points[cp].y;
        de_meaned[2] = cloud->points[points_indices_[cp][i]].z - cloud_fit_->points[cp].z;
        u_coord = de_meaned.dot (u);
        v_coord = de_meaned.dot (v);
        f_vec_(i) = de_meaned.dot (plane_parameters.start<3>());
        //cerr << "f_vec[" << i << "] = " << f_vec[i] << endl;

        // compute the polynomial's terms at the current point
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

      // --[ Computing coefficients ]--

      // storing partial result for reuse
      #ifndef GLOBAL
      P_weight_ = P_ * weight_vec_.asDiagonal();
      #else
      P_weight_ = P_.corner(Eigen::TopLeft, nr_coeff_,k) * weight_vec_.start(k).asDiagonal();
      #endif

      /// @NOTE: result is symmetrical... hopefully part<SelfAdjoint>() is the only thing that Eigen needs to fully optimize it
      #ifndef GLOBAL
      P_weight_Pt_.part<Eigen::SelfAdjoint>() = P_weight_ * P_.transpose ();
      #else
      P_weight_Pt_.part<Eigen::SelfAdjoint>() = P_weight_ * P_.corner(Eigen::TopLeft, nr_coeff_,k).transpose ();
      #endif

      // Solve linear equation system - TODO: maybe experiment with ldlt () - supposedly faster and more stable Cholesky decomposition but doesn't work...
      #ifndef INVERSE
      c_vec_ = P_weight_ * f_vec_;
      P_weight_Pt_.llt().solveInPlace(c_vec_);
      #endif

      #ifdef INVERSE
      //if (cp < 10)
      //  cerr << P_weight_Pt_.determinant () << " ";

      // check if the determinant is zero - for inversion
      #ifndef PARALLEL
      double det = P_weight_Pt_.determinant ();
      if (det <= max_bad_det)
        zero_determinant++;
      else // det > max_bad_det
      #endif
      {
        // invert matrix
        P_weight_Pt_.computeInverse(&inv_P_weight_Pt_); /// @NOTE: according to documentation, this is the optimal way (no alloc!)

        //if (cp < 10)
        //{
        //  Eigen::MatrixXd identity_check = P_weight_Pt_ * inv_P_weight_Pt_;
        //  cerr << "identity: " << identity_check.isIdentity(1e-5) << endl;// << identity_check << endl;
        //  cerr << "diagonal has values between " << identity_check.diagonal().minCoeff() << " and " << identity_check.diagonal().maxCoeff() << endl;
        //  cerr << "off-diagonal max: " << (identity_check - identity_check.diagonal().asDiagonal()).maxCoeff() << endl;
        //}

        // check if the inversion was successful - Eigen doesn't return error code :(
        bool inverse_ok = false;
        #ifndef PARALLEL
        if (det < min_good_det)
        {
          if ((P_weight_Pt_ * inv_P_weight_Pt_).isIdentity(1e-5))
          {
            min_good_det = det;
            inverse_ok = true;
          }
          else
          {
            max_bad_det = det;
            not_inverse++;
          }
          //ROS_INFO ("Interval for the value of the determinant in order for correct inversion converged to: (%g,%g).", max_bad_det, min_good_det);
        }
        else
          inverse_ok = true;
        #else
        if ((P_weight_Pt_ * inv_P_weight_Pt_).isIdentity(1e-5))
          inverse_ok = true;
        #endif

        // check if results are correct
        if (inverse_ok)
        {
          // solve the equation system
          #ifndef GLOBAL
          c_vec_ = inv_P_weight_Pt_ * P_weight_ * f_vec_;
          #else
          c_vec_ = inv_P_weight_Pt_ * P_weight_ * f_vec_.start(k);
          #endif
        #endif

          //if (cp < 10)
          //  cerr << "coefficients: " << c_vec_.transpose () << endl;
          //cerr << c_vec_(0) << " ";

          // project point onto the surface
          double du = 0.0; // value of partial derivative at the current point, see below
          double dv = 0.0; // value of partial derivative at the current point, see below
          if (c_vec_[0] < 0.03) /// maybe make some different check here !!!
          {
            //print_info(stderr, "Projection onto MLS surface along Darboux normal to the height at (0,0) of: "); print_value(stderr, "%g\n", c_vec[0]);
            cloud_fit_->points[cp].x += c_vec_[0] * plane_parameters[0];
            cloud_fit_->points[cp].y += c_vec_[0] * plane_parameters[1];
            cloud_fit_->points[cp].z += c_vec_[0] * plane_parameters[2];
            //if (cp < 10)
            //  cerr << "moving point with " << c_vec_[0] << " along Darboux normal" << endl;

            // c_vec[order_+1] and c_vec[1] is the partial derivative of the polynomial w.r.t. u and v respectively evaluated at (0,0)
            du = c_vec_[order_+1];
            dv = c_vec_[1];
          }
          else /// @NOTE: simple approximation of orthogonal projection, in case adjusting the height at (0,0) seems inappropriate
          {
            // this one is slower obviously than the projection along the Darboux normal
            #ifndef PARALELL
            nr_orthogonal++;
            #endif

            // get the u and v coordinates of the closest point
            u_coord = P_(order_+1,0); // value of u^1*v^0 = u
            v_coord = P_(1,0);        // value of u^0*v^1 = v

            // compute the value of the polynomial at the nearest point
            // and the partial derivative of the polynomial w.r.t. u and v at (u_coord,v_coord)
            double height = 0.0;
            int j = 0;
            u_pow = 1;
            for (int ui=0; ui<=order_; ui++)
            {
              v_pow = 1;
              for (int vi=0; vi<=order_-ui; vi++)
              {
                double term = c_vec_[j] * u_pow * v_pow;
                height += term;
                du += ui * term / u_coord;
                dv += vi * term / v_coord;
                j++;
                v_pow *= v_coord;
              }
              u_pow *= u_coord;
            }

            // move the point to the corresponding surface point
            Eigen::Vector3d movement = u_coord * u + v_coord * v + height * plane_parameters.start<3>();
            cloud_fit_->points[cp].x += movement[0];
            cloud_fit_->points[cp].y += movement[1];
            cloud_fit_->points[cp].z += movement[2];
            //if (cp < 10)
            //  cerr << "moving point with " << movement.norm() << " along " << movement.transpose() << endl;
          }

          //if (cp < 10)
          //  cerr << "final point: " << cloud_fit_->points[cp].x << " " << cloud_fit_->points[cp].y << " " << cloud_fit_->points[cp].z << endl;

          // "stop" timer
          #ifdef PARTIAL_TIMES
          sum_ts_polynomial += (ros::Time::now () - ts_tmp).toSec ();
          #endif

          // STEP4: Compute local features
          #ifdef PARTIAL_TIMES
          ts_tmp = ros::Time::now ();
          #endif
          // compute tangent vectors using du and dv evaluated at the current point - which is the origin in case of projection along the normals, or (u_coord,v_coord)
          Eigen::Vector3d n_a = u + plane_parameters.start<3>() * du;
          Eigen::Vector3d n_b = v + plane_parameters.start<3>() * dv;
          //Eigen::Vector3d n = n_a.cross (n_b).normalize ();
          //plane_parameters[0] = n[0];
          //plane_parameters[1] = n[1];
          //plane_parameters[2] = n[2];
          plane_parameters.start<3>() = n_a.cross (n_b);
          plane_parameters.start<3>().normalize ();
          /*ANNpoint n = annAllocPt(3);
          n[0] = -c_vec[order+1][0];
          n[1] = -c_vec[1][0];
          n[2] = 1;*/
          //cerr << "recomputed normal:" << endl << plane_parameters.start<3>() << endl;

          // TODO estimate surface curvature analogously to the PCA way of \lambda_0 / \sum{\lambda}
          /// @NOTE: dependency on order? does it differ if plane is only PCA, and same radius is used?
          //#ifndef GLOBAL - you don't need sub-vectors !!!
          //double height_variation = f_vec_.start(k).maxCoeff() - f_vec_.start(k).minCoeff();
          //cerr << height_variation << " ";
          //curvature = height_variation / (height_variation + 4*radius_curvature_);
          //cerr << weight_vec_.start(k).minCoeff() << " ";

          /**
           *  5   4   3   2   1   0
           * 2,0 1,1 1,0 0,2 0,1 0,0
           *  a   b   d   c   e   f
          **/

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

          #ifdef PARTIAL_TIMES
          sum_ts_extra += (ros::Time::now () - ts_tmp).toSec ();
          #endif

      #ifdef INVERSE
        } // inverse was ok
      } // determinant was ok
      #endif
    } // polynomial fit

    // set surface normal and curvature
    #ifdef PARTIAL_TIMES
    ts_tmp = ros::Time::now ();
    #endif
    if (viewpoint_cloud_ != NULL) /// @NOTE: the point where the normal is placed is from cloud_fit
      cloud_geometry::angles::flipNormalTowardsViewpoint (plane_parameters, cloud_fit_->points[cp], *viewpoint_cloud_);
    cloud_fit_->channels[original_chan_size + 0].values[cp] = plane_parameters (0);
    cloud_fit_->channels[original_chan_size + 1].values[cp] = plane_parameters (1);
    cloud_fit_->channels[original_chan_size + 2].values[cp] = plane_parameters (2);
    cloud_fit_->channels[original_chan_size + 3].values[cp] = curvature;
    if (compute_moments_)
    {
      /// @NOTE: the moments have to be computed from the original cloud OR from cloud_fit *after* fitting (TODO?)
      cloud_geometry::nearest::computeMomentInvariants (*cloud, points_indices_[cp], j1, j2, j3);
      cloud_fit_->channels[original_chan_size + 4].values[cp] = j1;
      cloud_fit_->channels[original_chan_size + 5].values[cp] = j2;
      cloud_fit_->channels[original_chan_size + 6].values[cp] = j3;
    }
    // TODO: bool cloud_geometry::nearest::isBoundaryPoint (const sensor_msgs::PointCloud &points, int q_idx, const std::vector<int> &neighbors, const Eigen::Vector3d& u, const Eigen::Vector3d& v, double angle_threshold)
    #ifdef PARTIAL_TIMES
    sum_ts_extra += (ros::Time::now () - ts_tmp).toSec ();
    #endif
  }
  ROS_INFO ("Neighborhood sizes varied from %d to %d.", k_min, k_max);
  ROS_INFO ("Fitted points in %g seconds.", (ros::Time::now () - ts).toSec ());
  #ifdef PARTIAL_TIMES
  ROS_INFO ("- time spent filtering points: %g seconds.", sum_ts_filter);
  ROS_INFO ("- time spent approximating tangent: %g seconds.", sum_ts_tangent);
  ROS_INFO ("- time spent computing polynomial: %g seconds.", sum_ts_polynomial);
  ROS_INFO ("- time spent computing extra features: %g seconds.", sum_ts_extra);
  #endif
  if (polynomial_fit_)
  {
    if (not_enough_nn != 0) ROS_WARN ("%d (%g%%) points had too few neighbors for polynomial fit.", not_enough_nn, not_enough_nn*100.0/cloud_fit_->points.size ());
    #ifndef PARALELL
    #ifdef INVERSE
    ROS_INFO ("Interval for the value of the determinant in order to be correctly invertible converged to: (%g,%g).", max_bad_det, min_good_det);
    if ((zero_determinant != 0) || (not_inverse != 0)) ROS_WARN ("Inverting squared weighted term matrix failed %d times (%g%%), and a bad determinant was detected %d times (%g%%).", not_inverse, not_inverse*100.0/cloud_fit_->points.size (), zero_determinant, zero_determinant*100.0/cloud_fit_->points.size ());
    #endif
    if (nr_orthogonal != 0) ROS_WARN ("%d (%g%%) points would have had a probably inaccurate parallel projection - approximate orthogonal projection was used instead.", nr_orthogonal, nr_orthogonal*100.0/cloud_fit_->points.size ());
    #endif
  }
  //cerr << endl << endl << endl;

  // Finish
  ROS_INFO ("MLS fit done in %g seconds.\n", (ros::Time::now () - global_time).toSec ());
  //pub_.publish (*cloud_fit_);
  return std::string("ok");
}

boost::shared_ptr<const MovingLeastSquares::OutputType> MovingLeastSquares::output ()
  // TODO {return *(cloud_fit_.get ());}
  {return cloud_fit_;}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <MovingLeastSquares> (argc, argv);
}
#endif
