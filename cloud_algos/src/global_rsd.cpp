#include <cloud_algos/global_rsd.h>

using namespace std;
using namespace cloud_algos;

void GlobalRSD::init (ros::NodeHandle& nh)
{
  nh_ = nh;
}

void GlobalRSD::pre ()
{
  nh_.param("label", label_, label_);
  nh_.param("width", width_, width_);
  nh_.param("step", step_, step_);
  nh_.param("min_voxel_pts", min_voxel_pts_, min_voxel_pts_);
}

void GlobalRSD::post ()
{

}

std::vector<std::string> GlobalRSD::requires ()
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

std::vector<std::string> GlobalRSD::provides ()
{
  std::vector<std::string> provides;
  // provides filtered points
  provides.push_back("x");
  provides.push_back("y");
  provides.push_back("z");
  return provides;
}

std::string GlobalRSD::process (const boost::shared_ptr<const GlobalRSD::InputType>& cloud)
{
  // Check if normals exist TODO: solve issue with getIndex(cloud, "nx");
  int nxIdx = -1;
  for (unsigned int d = 0; d < cloud->channels.size (); d++)
    if (cloud->channels[d].name == "nx")
    {
      nxIdx = d;
      break;
    }
  if (nxIdx == -1)
  {
    if (verbosity_level_ > -2) ROS_ERROR ("[GlobalRSD] Provided point cloud does not have normals. Use the normal_estimation or mls_fit first!");
    return std::string("missing normals");
  }
  /// @NOTE: we assume that if nx exists then ny and nz are the channels following it

  // Timers
  ros::Time global_time = ros::Time::now ();
  ros::Time ts;

  // Copy the original PCD
  cloud_vrsd_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
  cloud_vrsd_->header   = cloud->header;
  cloud_vrsd_->points   = cloud->points;
  cloud_vrsd_->channels = cloud->channels;

  // Allocate and name the extra needed channels (don't forget to allocate space for values too!)
  int rIdx = cloud_vrsd_->channels.size ();
  cloud_vrsd_->channels.resize (rIdx + 2 + 1);
  cloud_vrsd_->channels[rIdx+0].name = "r_min";
  cloud_vrsd_->channels[rIdx+1].name = "r_max";
  cloud_vrsd_->channels[rIdx+2].name = "r_dif";
  int regIdx = cloud_vrsd_->channels.size ();
  cloud_vrsd_->channels.resize (regIdx  + 1);
  cloud_vrsd_->channels[regIdx].name = "point_label";

  // Allocate space for the extra needed channel values
  for (size_t d = rIdx; d < cloud_radius_->channels.size (); d++)
  {
    cloud_vrsd_->channels[d].values.resize (cloud_vrsd_->points.size ());
    if (verbosity_level_ > 0) ROS_INFO ("[GlobalRSD] (cloud_vrsd) Added channel: %s", cloud_vrsd_->channels[d].name.c_str ());
  }

  // Create PCD for centroids
  cloud_centroids_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
  cloud_centroids_->header   = cloud_vrsd_->header;
  cloud_centroids_->points   = cloud_vrsd_->points;   /// @NOTE: will be overwritten and need to be resized afterwards
  cloud_centroids_->channels = cloud_vrsd_->channels; /// @NOTE: will be overwritten and need to be resized afterwards

  // Create the final cloud with 1 point (cluster center) to hold the Global Radius-based Surface Descriptor
  cloud_grsd_ = boost::shared_ptr<sensor_msgs::PointCloud> (new sensor_msgs::PointCloud());
  cloud_grsd_->header   = cloud->header;
  cloud_grsd_->points.resize (1);

  // Allocate the extra needed channels
  cloud_grsd_->channels.resize (nr_bins_ + 1);

  // Name the extra channels
  for (int i = 0; i < nr_bins_; i++)
  {
    char dim_name[16];
    sprintf (dim_name, "f%d", i+1);
    cloud_grsd_->channels[i].name = dim_name;
    cloud_grsd_->channels[i].values.resize (1);
    if (i == 0)
      if (verbosity_level_ > 0) ROS_INFO ("[GlobalRSD] (cloud_grsd) Added channel: %s ... [cont]", dim_name);
    else if (i == nr_bins_-1)
      if (verbosity_level_ > 0) ROS_INFO ("[GlobalRSD] (cloud_grsd) Added channel: %s [done]", dim_name);
  }
  cloud_grsd_->channels[nr_bins_].name = "point_label";
  cloud_grsd_->channels[nr_bins_].values.resize (1);
  if (verbosity_level_ > 0) ROS_INFO ("[GlobalRSD] (cloud_grsd) Added channel: %s", cloud_grsd_->channels[nr_bins_].name.c_str ());

  // Create a fixed-size octree decomposition
  setOctree (cloud_vrsd_, width_, -1);

  // Select only the occupied leaves
  std::list<octomap::OcTreeVolume> cells;
  octree_->getOccupied(cells, 0);

  // Set surface type for each cell in advance
  ts = ros::Time::now ();
  int cnt_centroids = 0;
  double x_min, y_min, z_min, x_max, y_max, z_max;
  octree_->getMetricMin(x_min, y_min, z_min);
  octree_->getMetricMax(x_max, y_max, z_max);
  for (std::list<octomap::OcTreeVolume>::iterator it_i = cells.begin (); it_i != cells.end (); ++it_i)
  {
    // Get a cell
    octomap::point3d centroid_i;
    centroid_i(0) = it_i->first.x();
    centroid_i(1) = it_i->first.y();
    centroid_i(2) = it_i->first.z();
    octomap::OcTreeNodePCL *node_i = octree_->search(centroid_i);

    // Get its contents
    vector<int> indices = node_i->get3DPointInliers ();
    if (indices.size () == 0)
      continue;

    // Iterating through neighbors
    vector<int> neighbors = indices;
    for (int i = step; i <= step; i++)
    {
      for (int j = -step; j <= step; j++)
      {
        for (int k = -step; k <= step; k++)
        {
          // skip current point
          if (i==0 && j==0 && k==0)
            continue;
          // skip inexistent cells
          octomap::point3d centroid_neighbor;
          centroid_neighbor(0) = centroid_i(0) + i*width_;
          centroid_neighbor(1) = centroid_i(1) + j*width_;
          centroid_neighbor(2) = centroid_i(2) + k*width_;
          if (centroid_neighbor(0)<x_min || centroid_neighbor(1)<y_min || centroid_neighbor(2)<z_min || centroid_neighbor(0)>x_max || centroid_neighbor(1)>y_max || centroid_neighbor(2)>z_max)
            continue;
          // accumulate indices
          octomap::OcTreeNodePCL *node_neighbor = octree_->search(centroid_neighbor);
          vector<int> ni = node_neighbor->get3DPointInliers ();
          neighbors.insert (neighbors.end (), ni.begin (), ni.end ());
        } // i
      } // j
    } // k

    // Mark all points with result
    int type = setSurfaceType (cloud_vrsd_, &indices, &neighbors, nxIdx, (2*step_+1)*width_*sqrt(3), regIdx, rIdx);

    // Set up PCD with centroids
    cloud_centroids_->points[cnt_centroids].x = centroid_i(0);
    cloud_centroids_->points[cnt_centroids].y = centroid_i(1);
    cloud_centroids_->points[cnt_centroids].z = centroid_i(2);
    for (size_t d = 0; d < cloud->channels.size (); d++)
      cloud_centroids_->channels[d].values[cnt_centroids] = 0.0;
    for (size_t i = 0; i < transitions.size (); i++)
    {
      for (size_t d = 0; d < cloud->channels.size (); d++)
        cloud_centroids_->channels[d].values[cnt_centroids] = 0.0;
    }
    cloud_centroids_->channels[regIdx].values[cnt_centroids] = type;
    cnt_centroids++;
  }
  cloud_centroids_->points.resize (cnt_centroids);
  for (size_t d = 0; d < cloud_centroids_->channels.size (); d++)
    cloud_centroids_->channels[d].values.resize (cnt_centroids);
  if (verbosity_level_ > 0) ROS_INFO ("[GlobalRSD] Cells annotated in %g seconds.", (ros::Time::now () - ts).toSec ());

  // Initialize transition matrix for counting
  vector<vector<int> > transitions (NR_CLASS+1);
  for (size_t i = 0; i < transitions.size (); i++)
    transitions[i].resize (NR_CLASS+1);

  /// Iterate over all cells
  ts = ros::Time::now ();
  float line_p1[3], line_p2[3], box_bounds[6];
  for (std::list<octomap::OcTreeVolume>::iterator it_i = cells.begin (); it_i != cells.end (); ++it_i)
  {
    // Get a cell
    octomap::point3d centroid_i;
    centroid_i(0) = it_i->first.x();
    centroid_i(1) = it_i->first.y();
    centroid_i(2) = it_i->first.z();
    octomap::OcTreeNodePCL *node_i = octree_->search(centroid_i);

    // Get its contents
    vector<int> indices = node_i->get3DPointInliers ();
    if (indices.size () == 0)
      continue;

    // Connect current cell to all the remaining ones
    for (std::list<octomap::OcTreeVolume>::iterator it_j = it_i; it_j != cells.end (); ++it_j)
    {
      /// TODO: just start for from it_i+1
      if (it_i == it_j)
       continue;

      // Get a cell
      octomap::point3d centroid_j;
      centroid_j(0) = it_j->first.x();
      centroid_j(1) = it_j->first.y();
      centroid_j(2) = it_j->first.z();
      octomap::OcTreeNodePCL *node_j = octree_->search(centroid_j);

      // Get its contents
      vector<int> indices_j = node_j->get3DPointInliers ();
      if (indices_j.size () == 0)
        continue;

      // Create a paired histogram vector which holds: a) the actual centroid value of the intersected voxel, b) the distance from start_voxel to voxel_i
      vector<pair<int, IntersectedLeaf> > histogram_values;

      // Get the leaves along the ray
      vector<point3d> ray;
      octree_->computeRay(centroid_i, centroid_j, ray);

      // Iterate over leaves
      for (vector<point3d>::iterator centroid_ij = ray.begin (); centroid_ij != ray.end (); centroid_ij++)
      {
        // Get a cell
        octomap::OcTreeNodePCL *node_neighbor = octree_->search(*centroid_ij)
        // Get its contents
        vector<int> indices_ij = ->get3DPointInliers ();

        // Compute the distance to the start leaf
        pair<int, IntersectedLeaf> histogram_pair;
        histogram_pair.second.centroid = *centroid_ij;
        histogram_pair.second.sqr_distance = _sqr_dist (*centroid_ij, centroid_i);
        histogram_pair.second.nr_points = ;

        // Get its contents, if empty set to EMPTY_VALUE
        if (histogram_pair.second.nr_points == 0)
        {
          histogram_pair.first = -1;
        }
        else
        {
          //histogram_pair.first = getSurfaceType (points, &((*tree) (x, y, z)), nxIdx, width*sqrt(3));
          histogram_pair.first = (int)(new_points[(*tree) (x, y, z).at (0)][regIdx]);
        }

        histogram_values.push_back (histogram_pair);
      }

      // Add the first voxel
      pair<int, IntersectedLeaf> histogram_pair1;
      histogram_pair1.second.sqr_distance = _sqr_dist (centroid_i, centroid_i); // 0.0
      histogram_pair1.second.centroid = centroid_i;
      histogram_pair1.second.nr_points = ((*tree) (X_i[0], X_i[1], X_i[2])).size ();
      histogram_pair1.first = (int)(new_points[(*tree) (X_i[0], X_i[1], X_i[2]).at (0)][regIdx]);
      histogram_values.push_back (histogram_pair1);

      // Add the last voxel
      pair<int, IntersectedLeaf> histogram_pair2;
      histogram_pair2.second.sqr_distance = _sqr_dist (centroid_j, centroid_i); // line length
      histogram_pair2.second.centroid = centroid_j;
      histogram_pair2.second.nr_points = ((*tree) (X_j[0], X_j[1], X_j[2])).size ();
      histogram_pair2.first = (int)(new_points[(*tree) (X_j[0], X_j[1], X_j[2]).at (0)][regIdx]);
      histogram_values.push_back (histogram_pair2);

      // Sort the histogram according to the distance of the leaves to the start leaf
      sort (histogram_values.begin (), histogram_values.end (), histogramElementCompare);

      // Count transitions between the first and last voxel
      for (unsigned int hi = 1; hi < histogram_values.size (); hi++)
      {
        // transition matrix has to be symmetrical
        transitions[histogram_values[hi].first+1][histogram_values[hi-1].first+1]++;
        transitions[histogram_values[hi-1].first+1][histogram_values[hi].first+1]++;
        //cerr << "transition: " << histogram_values[hi].first-EMPTY_VALUE << " => " << histogram_values[hi-1].first-EMPTY_VALUE << endl;
      }
    }
  }
  if (verbosity_level_ > 0) ROS_INFO ("[GlobalRSD] Transitions counted in %g seconds.", (ros::Time::now () - ts).toSec ());

  /// Outputting result in libSVM format
  if (verbosity_level_ > 1)
  {
    ROS_INFO ("[GlobalRSD] Transition matrix is:");
    for (int i=0; i<NR_CLASS+1; i++)
    {
      stringstream line;
      for (int j=0; j<NR_CLASS+1; j++)
        line << " " << transitions[i][j];
      ROS_INFO ("%s", line.str ().c_str ());
    }
  }
  int nrf = 0;
  for (int i=0; i<NR_CLASS+1; i++)
    for (int j=i; j<NR_CLASS+1; j++)
      cloud_grsd_->channels[nrf++].values[0] = transitions[i][j];
  cloud_grsd_->channels[nr_bins_].values = label_;

  // Finish
  if (verbosity_level_ > 0) ROS_INFO ("[GlobalRSD] Computed features in %g seconds.", (ros::Time::now () - global_time).toSec ());
  return std::string("ok");
}

boost::shared_ptr<const GlobalRSD::OutputType> GlobalRSD::output ()
  {return cloud_grsd_;}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <GlobalRSD> (argc, argv);
}
#endif
