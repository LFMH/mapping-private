#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/pfh.h>

using namespace std;
using namespace cloud_algos;

void PointFeatureHistogram::init (ros::NodeHandle&)
{
  // node handler and publisher
  // TODO needed?
  //nh_ = nh;
  //pub_ = nh_.advertise <sensor_msgs::PointCloud> ("vis_pfh", 1);
}

void PointFeatureHistogram::pre ()
{

}

void PointFeatureHistogram::post ()
{

}

std::vector<std::string> PointFeatureHistogram::requires ()
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

std::vector<std::string> PointFeatureHistogram::provides ()
{
  // compute number of bins
  nr_features_ = use_dist_ ? 4:3;
  if (combine_)
  {
    nr_bins_ = (int)ceil (pow (quantum_, nr_features_));
  }
  else
  {
    nr_bins_ = quantum_ * nr_features_;
    //if (diferential_)
    //  nr_bins_ -= nr_features_;
  }

  std::vector<std::string> provides(3);
  // provides features (variable number)
  for (int i = 0; i < nr_bins_; i++)
  {
    char dim_name[16];
    sprintf (dim_name, "f%d", i+1);
    provides.push_back (dim_name);
  }
  return provides;
}

std::string PointFeatureHistogram::process (const boost::shared_ptr<const PointFeatureHistogram::InputType>& cloud)
{
  // TODO where to put this?
  clear ();

  // Check if normals exist TODO: solve issue with getIndex(cloud, "nx");
  int nxIdx = -1;
  for (unsigned int d = 0; d < cloud->get_channels_size (); d++)
    if (cloud->channels[d].name == "nx")
    {
      nxIdx = d;
      break;
    }
  if (nxIdx == -1)
  {
    ROS_ERROR ("Provided point cloud does not have normals. Use the normal_estimation or mls_fit first!");
    return std::string("missing normals");
  }
  /// @NOTE: we assume that if nx exists then ny and nz are the channels following it

  // compute number of bins
  nr_features_ = use_dist_ ? 4:3;
  if (combine_)
  {
    nr_bins_ = (int)ceil (pow (quantum_, nr_features_));
  }
  else
  {
    nr_bins_ = quantum_ * nr_features_;
    //if (diferential_)
    //  nr_bins_ -= nr_features_;
  }

  /// The order of the features in the histogram bining (if combine==true) - DO NOT CHANGE IT !!!
  //int a = 3, b = 1, c = 0, d = 2;
  int a_ = 3, b_ = 0, c_ = 2, d_ = 1;
  if (!use_dist_)
  {
    a_ = 2; b_ = 0; c_ = 1; d_ = 3;
  }
  /// If the features are saved sequentially the order doesn't really matter
  if (!combine_)
  {
    a_ = 0; b_ = 1; c_ = 2; d_ = 3;
  }
  /// @NOTE: Distance has to be last indexed if it is not used !!!

  // Timers
  ros::Time global_time = ros::Time::now ();
  ros::Time ts;

  // Copy the original PCD
  cloud_pfh_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
  cloud_pfh_->header   = cloud->header;
  cloud_pfh_->points   = cloud->points;
  cloud_pfh_->channels = cloud->channels;

  // Allocate the extra needed channels
  int fIdx = cloud_pfh_->channels.size ();
  cloud_pfh_->channels.resize (fIdx  + nr_bins_ + 1);

  // Name the extra channels
  for (int i = 0; i < nr_bins_; i++)
  {
    char dim_name[16];
    sprintf (dim_name, "f%d", i+1);
    cloud_pfh_->channels[fIdx + i].name = dim_name;
    if (i == 0)
      ROS_INFO ("Added channel: %s ... [cont]", dim_name);
    else if (i == nr_bins_-1)
      ROS_INFO ("Added channel: %s [done]", dim_name);
  }
  if (point_label_ != -1)
  {
    cloud_pfh_->channels[fIdx + nr_bins_].name = "point_label";
    ROS_INFO ("Added channel: %s", cloud_pfh_->channels[fIdx + nr_bins_].name.c_str ());
  }

  // Allocate space for histograms
  vector<vector<float> > histograms (nr_bins_);
  for (int b = 0; b < nr_bins_; b++)
    histograms[b].resize (cloud_pfh_->points.size (), 0.0);
  for (size_t d = fIdx; d < cloud_pfh_->channels.size (); d++)
  {
    // initialize histogram bins to 0.0
    cloud_pfh_->channels[d].values.resize (cloud_pfh_->points.size (), 0.0);
    //ROS_INFO ("Added channel: %s", cloud_pfh_->channels[d].name.c_str ());
  }

  // Create Kd-Tree
  if (kdtree_ == NULL)
  {
    ts = ros::Time::now ();
    /// @NOTE: kd-tree for the original PCD... doesn't matter since they are the same
    kdtree_ = new cloud_kdtree::KdTreeANN (*cloud);
    ROS_INFO ("Kd-tree created in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // Allocate enough space for point indices and distances
  points_indices_.resize (cloud_pfh_->points.size ());
  points_sqr_distances_.resize (cloud_pfh_->points.size ());

  // Get the nearest neighbors for all points to be fitted
  ts = ros::Time::now ();
  int k_min = max_nn_, k_max = 0;
  for (size_t cp = 0; cp < cloud_pfh_->points.size (); cp++)
  {
    kdtree_->radiusSearch (cp, radius_, points_indices_[cp], points_sqr_distances_[cp], max_nn_);
    int k = points_indices_[cp].size ();
    if (k > k_max) k_max = k;
    if (k < k_min) k_min = k;
  }
  ROS_INFO ("Nearest neighbors found in %g seconds.", (ros::Time::now () - ts).toSec ());
  ROS_INFO ("Neighborhood sizes varied from %d to %d.", k_min, k_max);

  // TODO make a function that takes a neighborhood and returns the histogram (mesh or star)

  // Go through all the points and compute histograms
  ts = ros::Time::now ();
  double alpha, beta, gamma, delta;
  double u[3], v[3], w[3], t[3], dP2P1[3];
  int index, power;
  int fi[4]; // feature index
  // TODO parallelize!
  for (size_t cp = 0; cp < cloud_pfh_->points.size (); cp++)
  {
    // specify class label - if known
    if (point_label_ != -1)
      cloud_pfh_->channels[fIdx + nr_bins_].values[cp] = point_label_;

    // increment for the bins
    double npsqr = 100.0 / points_indices_[cp].size ();

    // TODO add cached version!

    // for all neighbors (so skip the first in points_indices_[cp] because it is cp itself)
    for (size_t ni = 1; ni < points_indices_[cp].size (); ni++)
    {
      //cerr << " " << ni << "/" << points_indices_[cp][ni];

      delta = sqrt (points_sqr_distances_[cp][ni]);
      if (getPointPairFeatures (cloud_pfh_, nxIdx, cp, points_indices_[cp][ni], u, v, w, t, dP2P1, alpha, beta, gamma, delta, 2*radius_, check_flip_, abs_angles_))
      {
        fi[a_] = max (0, min (quantum_-1, (int) floor (quantum_ * alpha)));
        fi[b_] = max (0, min (quantum_-1, (int) floor (quantum_ * beta)));
        fi[c_] = max (0, min (quantum_-1, (int) floor (quantum_ * gamma)));
        if (use_dist_)
          fi[d_] = max (0, min (quantum_-1, (int) floor (quantum_ * delta)));
        #if DEBUG
          //cerr << "feature indices: " << fi[0] << " " << fi[1] << " " << fi[2] << " " << fi[3] << endl;
          for (int f = 0; f < nr_features_; f++)
            fnr[f][fi[f]]++;
        #endif

        /*
        f[a_] = getFeatureIndice (quantum_, alpha);
        f[b_] = getFeatureIndice (quantum_, beta);
        f[c_] = getFeatureIndice (quantum_, gamma);
        if (use_dist_)
          f[d_] = getFeatureIndice (quantum_, delta);
        */

        // Assemble a n-D histogram
        if (combine_)
        {
          index = 0;
          power = 1;
          // treat the feature indices as numbers represented in nr_features base
          for (int f = 0; f < nr_features_; f++)
          {
            index += power * fi[f];
            power *= quantum_;
          }
          #if DEBUG
            //cerr << "index in the histogram: " << index << endl;
            inr[index]++;
          #endif
          histograms[index][cp] += npsqr;
          //cloud_pfh_->channels[fIdx+index].values[cp] += npsqr;
        }
        // Assemble individual 1-D histograms for each feature
        else
        {
          histograms[a_*quantum_+fi[a_]][cp] += npsqr;
          histograms[b_*quantum_+fi[b_]][cp] += npsqr;
          histograms[c_*quantum_+fi[c_]][cp] += npsqr;
          if (use_dist_)
            histograms[d_*quantum_+fi[d_]][cp] += npsqr;
          /*
          cloud_pfh_->channels[fIdx+a*quantum_+fi[a_]].values[cp] += npsqr;
          cloud_pfh_->channels[fIdx+b*quantum_+fi[b_]].values[cp] += npsqr;
          cloud_pfh_->channels[fIdx+c*quantum_+fi[c_]].values[cp] += npsqr;
          if (use_dist_)
            cloud_pfh_->channels[fIdx+d*quantum_+fi[d_]].values[cp] += npsqr;
          */
        }
      }
      else
      {
        // assure normalized histogram in the case the point pair did not produce valid features (very rare)
        if (combine_)
          for (int i=0; i<nr_bins_; i++)
            histograms[i][cp] += npsqr/nr_bins_;
        else
          for (int i=0; i<nr_bins_; i++)
            histograms[i][cp] += npsqr/quantum_; // == nr_features_* npsqr / nr_bins_
      }
    }

    // Subtract for each feature's histogram bins the value of the previous histogram bin (from the same feature)
    /* TODO: where to put this
    if (!combine && diferential)
    {
      for (int f = 0; f < nr_features; f++)
      {
        int baseIdx = fIdx + f*quantum_;
        for (int b = 1; b < quantum_; b++)
          cloud_pfh_->channels[baseIdx+b].values[cp] -= cloud_pfh_->channels[baseIdx+b-1].values[cp];
      }
    }
    */
  }
  ROS_INFO ("Computed feature histograms in %g seconds.", (ros::Time::now () - ts).toSec ());

  // Do an average of histograms weighted with the distance (or leave them as they are)
  if (!average_)
  {
    // use the histogram vectors as channel values
    ts = ros::Time::now ();
    for (int b = 0; b < nr_bins_; b++)
      cloud_pfh_->channels[fIdx+b].values = histograms[b];
    ROS_INFO ("Copied histograms in %g seconds.", (ros::Time::now () - ts).toSec ());
    /// @NOTE: we loose a bit of time here, but no biggie
  }
  else
  {
    ts = ros::Time::now ();
    // TODO parallelize!
    for (size_t cp = 0; cp < cloud_pfh_->points.size (); cp++)
    {
      // TODO make this a function too

      // Skip points with no neighbors
      if (points_indices_[cp].size () == 0)
        continue;

      // Compute the weighted sum
      double sum_weight = 0.0;
      /// @NOTE: don't consider current point's histogram as the values from it are encompassed in the neighboring histograms
      for (size_t ni = 1; ni < points_indices_[cp].size (); ni++)
      {
        // TODO: do a better weighting
        double weight = 1.0 / points_sqr_distances_[cp][ni];
        sum_weight += weight;

        // Multiply all values with the weight
        for (int b = 0; b < nr_bins_; b++)
          cloud_pfh_->channels[fIdx+b].values[cp] += histograms[b][points_indices_[cp][ni]] * weight;
      }

      // Normalize histograms
      for (int b = 0; b < nr_bins_; b++)
        cloud_pfh_->channels[fIdx+b].values[cp] /= sum_weight;
    }
    ROS_INFO ("Computed weighted averages of histograms in %g seconds.", (ros::Time::now () - ts).toSec ());
  }

  // TODO where to put this?
  // Subtract for each feature's histogram bins the value of the previous histogram bin (from the same feature)
  if (!combine_ && diferential_)
  {
    // TODO parallelize!
    for (size_t cp = 0; cp < cloud_pfh_->points.size (); cp++)
    {
      for (int f = 0; f < nr_features_; f++)
      {
        int baseIdx = fIdx + f*quantum_;
        for (int b = quantum_-1; b > 0; b--)
          cloud_pfh_->channels[baseIdx+b].values[cp] -= cloud_pfh_->channels[baseIdx+b-1].values[cp];
      }
    }
  }

  // Finish
  ROS_INFO ("PFH done in %g seconds.\n", (ros::Time::now () - global_time).toSec ());
  return std::string("ok");
}

PointFeatureHistogram::OutputType PointFeatureHistogram::output ()
  {return *(cloud_pfh_.get());}
  //{return *cloud_pfh_;}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <PointFeatureHistogram> (argc, argv);
}
#endif
