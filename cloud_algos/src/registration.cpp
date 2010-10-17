#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/registration.h>

using namespace cloud_algos;


//instead of maintaining a patch to ROS, we provide a copy of this function
int
  getIndex (const boost::shared_ptr<const sensor_msgs::PointCloud> points, std::string value)
{
  // Get the index we need
  for (unsigned int d = 0; d < points->channels.size (); d++)
    if (points->channels[d].name == value)
      return (d);

  return (-1);
}

void Registration::init (ros::NodeHandle &nh) 
{
  nh_ = nh;
}

void Registration::pre ()
{

}

void Registration::post ()
{

}
std::vector<std::string> Registration::requires () 
  {return std::vector<std::string>();}

std::vector<std::string> Registration::provides ()
  {return std::vector<std::string>();}


double Registration::RigidTransformSVD (const boost::shared_ptr<const sensor_msgs::PointCloud>& source, std::vector<int> &src, std::vector<int> &tgt, Eigen3::Matrix4d &transform)
{
  // Compute centroid of both point clouds
  geometry_msgs::Point32 source_centroid;
  geometry_msgs::Point32 target_centroid;
  cloud_geometry::nearest::computeCentroid (source, src, source_centroid);
  cloud_geometry::nearest::computeCentroid (target_, tgt, target_centroid);

  Eigen3::VectorXd b(src.size());
  Eigen3::MatrixXd A(tgt.size(), 6);

  for (int i = 0; i < (int)src.size(); i++)
  {
    double normal_x = target_->channels[nxIdx_].values[tgt[i]];
    double normal_y = target_->channels[nyIdx_].values[tgt[i]];
    double normal_z = target_->channels[nzIdx_].values[tgt[i]];
    b[i] = normal_x*target_->points[tgt[i]].x + normal_y*target_->points[tgt[i]].y + normal_z*target_->points[tgt[i]].z
         - normal_x*source->points[src[i]].x - normal_y*source->points[src[i]].y - normal_z*source->points[src[i]].z;
    A(i, 0) = normal_z*source->points[src[i]].y - normal_y*source->points[src[i]].z;
    A(i, 1) = normal_x*source->points[src[i]].z - normal_z*source->points[src[i]].x;
    A(i, 2) = normal_y*source->points[src[i]].x - normal_x*source->points[src[i]].y;
    A(i, 3) = normal_x;
    A(i, 4) = normal_y;
    A(i, 5) = normal_z;
  }
  Eigen3::SVD<Eigen3::MatrixXd> svd = A.svd();

  const Eigen3::MatrixXd U = svd.matrixU();
  const Eigen3::MatrixXd V = svd.matrixV();
  const Eigen3::VectorXd S = svd.singularValues();
  
  Eigen3::MatrixXd S_plus(S);
  for (int i = 0; i < S_plus.size(); i++)
    if (S_plus(i) != 0.0)
      S_plus(i) = 1.0f / S_plus(i);
      
  Eigen3::MatrixXd A_plus = V*S_plus.col(0).asDiagonal()*U.transpose();
  Eigen3::VectorXd x_opt = A_plus * b;
  std::cout << "Solution: " << x_opt << std::endl;
  
  Eigen3::MatrixXd T (4,4);
  T.setIdentity ();
  T(0, 3) = x_opt(3);
  T(1, 3) = x_opt(4);
  T(2, 3) = x_opt(5);

  double alpha = x_opt(0);
  double beta  = x_opt(1);
  double gamma = x_opt(2);

  Eigen3::MatrixXd R (4,4);
  R.setIdentity ();
  R(1, 1) =   cos(gamma) * cos(beta);
  R(1, 2) = - sin(gamma) * cos(alpha) + cos(gamma) * sin(beta) * sin(alpha);
  R(1, 3) =   sin(gamma) * sin(alpha) + cos(gamma) * sin(beta) * cos(alpha);
  R(2, 1) =   sin(gamma) * cos(beta);
  R(2, 2) =   cos(gamma) * cos(alpha) + sin(gamma) * sin(beta) * sin(alpha);
  R(2, 3) = - cos(gamma) * sin(alpha) + sin(gamma) * sin(beta) * cos(alpha);
  R(3, 1) = - sin(beta);
  R(3, 2) =   cos(beta) * sin(alpha);
  R(3, 3) =   cos(beta) * cos(alpha);

  transform = T * R * transform;
  //Eigen3::Matrix
  //x = A_plus

  return 0.0;
}

double Registration::oneIteration (const boost::shared_ptr<const sensor_msgs::PointCloud>& source, Eigen3::Matrix4d &transform)
{
  std::vector<bool> drawn (source->points.size(), false);
 
  std::vector<int> source_corr((unsigned int)(0.01*source->points.size()));
  std::vector<int> target_corr((unsigned int)(0.01*source->points.size()));
  for (unsigned int i = 0; i < (unsigned int)(0.01*source->points.size()); i++)
  {
    geometry_msgs::Point32 query_point;

    std::vector<int> k_indices;
    std::vector<float> k_distances;
    
    int random_idx;
    do {
     random_idx = (int) (rand () * (source->points.size() / (RAND_MAX + 1.0)));
    } while (drawn [random_idx]);
    drawn[random_idx] = true;

    kdtree_->nearestKSearch (source->points[random_idx], 1, k_indices, k_distances);
    target_corr[i] = k_indices[0];
    source_corr[i] = random_idx;
  }
  ROS_INFO ("selected %u sample points", (unsigned)source_corr.size());
  double error = RigidTransformSVD (source, target_corr, source_corr, transform);
  return error;
}


std::string Registration::process (const boost::shared_ptr<const Registration::InputType>& input)
{ 
  // does that make any sense?
  boost::shared_ptr<const sensor_msgs::PointCloud> source = boost::dynamic_pointer_cast<const sensor_msgs::PointCloud> (input);

  if (target_.get() != 0) // if we have a registration target set
  { 
    //indices_sorted_ = std::vector<int>(source->points.size());
    //for (unsigned int i = 0; i < source->points.size(); i++)
      //indices_sorted_[i] = i;

    //TODO: make these parameters
    int max_iterations_ = 100;
    double error_bound = 0.0001;

    Eigen3::Matrix4d transform;
    transform.setIdentity ();
    
    for (int iterations = 0; iterations < max_iterations_; iterations++)
    {
      double error = oneIteration (source, transform);
      ROS_INFO ("iteration %i: %f ", iterations, error);
//      if (error < error_bound)
//        break;
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

