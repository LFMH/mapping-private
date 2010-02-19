#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/registration.h>

using namespace cloud_algos;


//instead of maintaining a patch to ROS, we provide a copy of this function
int
  getIndex (const boost::shared_ptr<const sensor_msgs::PointCloud> points, std::string value)
{
  // Get the index we need
  for (unsigned int d = 0; d < points->get_channels_size (); d++)
    if (points->channels[d].name == value)
      return (d);

  return (-1);
}

void Registration::init (ros::NodeHandle &nh) 
{
  nh_ = nh;
}

std::vector<std::string> Registration::pre () 
  {return std::vector<std::string>();}

std::vector<std::string> Registration::post ()
  {return std::vector<std::string>();}


double Registration::RigidTransformSVD (const boost::shared_ptr<const sensor_msgs::PointCloud>& source, std::vector<int> &src, std::vector<int> &tgt, Eigen::Matrix4d &transform)
{
  // Compute centroid of both point clouds
  geometry_msgs::Point32 source_centroid;
  geometry_msgs::Point32 target_centroid;
  cloud_geometry::nearest::computeCentroid (source, src, source_centroid);
  cloud_geometry::nearest::computeCentroid (target_, tgt, target_centroid);

  Eigen::VectorXd b(src.size());
  Eigen::MatrixXd A(6, tgt.size());

  for (int i = 0; i < (int)src.size(); i++)
  {
    double normal_x = target_->channels[nxIdx_].values[tgt[i]];
    double normal_y = target_->channels[nyIdx_].values[tgt[i]];
    double normal_z = target_->channels[nzIdx_].values[tgt[i]];
    b[i] = normal_x*target_->points[tgt[i]].x + normal_y*target_->points[tgt[i]].y + normal_z*target_->points[tgt[i]].z
         - normal_x*source->points[src[i]].x - normal_y*source->points[src[i]].y - normal_z*source->points[src[i]].z;
    A(0, i) = normal_z*source->points[src[i]].y - normal_y*source->points[src[i]].z;
    A(1, i) = normal_x*source->points[src[i]].z - normal_z*source->points[src[i]].x;
    A(2, i) = normal_y*source->points[src[i]].x - normal_x*source->points[src[i]].y;
    A(3, i) = normal_x;
    A(4, i) = normal_y;
    A(5, i) = normal_z;
  }
  Eigen::SVD<Eigen::MatrixXd::PlainMatrixType> svd = A.svd();

  const Eigen::MatrixXd U = svd.matrixU();
  const Eigen::MatrixXd V = svd.matrixV();
  const Eigen::VectorXd S = svd.singularValues();
  
  Eigen::MatrixXd S_plus(S);
  for (unsigned int i = 0; i < S_plus.size(); i++)
    if (S_plus[i] != 0.0)
      S_plus[i] = 1.0f / S_plus[i];
      
  Eigen::MatrixXd A_plus = V*S_plus*U.transpose();
  //Eigen::Matrix
  //x = A_plus

  return 0.0;
}

double Registration::oneIteration (const boost::shared_ptr<const sensor_msgs::PointCloud>& source, Eigen::Matrix4d &transform)
{
  // TODO: this method to get random samples is low on code, but high on mem..
  std::vector<int> rand_sample (indices_sorted_);
  std::random_shuffle (rand_sample.begin (), rand_sample.end());
  
  std::vector<int> source_corr(source->points.size());
  std::vector<int> target_corr(source->points.size());
  for (unsigned int i = 0; i < 0.1*source->points.size(); i++)
  {
    geometry_msgs::Point32 query_point;

    std::vector<int> k_indices;
    std::vector<float> k_distances;
    kdtree_->nearestKSearch (source->points[rand_sample[i]], 1, k_indices, k_distances);
    target_corr[i] = k_indices[0];
    source_corr[i] = rand_sample[i];
  }

  double error = RigidTransformSVD (source, target_corr, source_corr, transform);
  return error;
}


std::string Registration::process (const boost::shared_ptr<const Registration::InputType>& input)
{ 
  // does that make any sense?
  boost::shared_ptr<const sensor_msgs::PointCloud> source = boost::dynamic_pointer_cast<const sensor_msgs::PointCloud> (input);

  if (target_.get() != 0) // if we have a registration target set
  { 
    indices_sorted_ = std::vector<int>(source->points.size());
    for (unsigned int i = 0; i < source->points.size(); i++)
      indices_sorted_[i] = i;

    //TODO: make these parameters
    int max_iterations_ = 100;
    double error_bound = 0.0001;

    Eigen::Matrix4d transform;
    transform.setIdentity ();
    
    for (int iterations = 0; iterations < max_iterations_; iterations++)
    {
      double error = oneIteration (source, transform);
      if (error < error_bound)
        break;
    }
  }

  //transform_ = transform;
  // for the next registration, we set target to be the source from current iteration
  setTarget (source);
  return std::string("");
}

void Registration::setTarget (const boost::shared_ptr<const sensor_msgs::PointCloud> &target)
{ 
  target_ = target; 
  kdtree_ = new cloud_kdtree::KdTreeANN (*target);
  // TODO: get rid of this nasty const cast!

  nxIdx_ = getIndex (target, "nx");
  nyIdx_ = getIndex (target, "ny");
  nzIdx_ = getIndex (target, "nz");
}

Registration::OutputType Registration::output ()
{
  return transform_;
}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <Registration> (argc, argv);
}
#endif

