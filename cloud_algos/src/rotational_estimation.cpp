#include <ros/ros.h>
#include <ros/node_handle.h>
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/rotational_estimation.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <ias_sample_consensus/sac_model_rotational.h>

void
  findRotationalObjects (sensor_msgs::PointCloud cloud, double threshold_, double probability_, 
                         int max_iterations_, sensor_msgs::PointCloud &cloud_synth, mapping_msgs::PolygonalMap &pmap)
{
  int debug = 2;
  ias_sample_consensus::SACModelRotational *sac_model_ = new ias_sample_consensus::SACModelRotational (pmap);
  std::cerr << "[findRotationalObjects] got called with PCD with " << cloud.points.size () << " points" << std::endl;
  sac_model_->setDataSet (&cloud);

  int iterations_ = 0;
  int n_best_inliers_count = -1;
  double k = INT_MAX;

  std::vector<int> best_model;
  std::vector<int> best_inliers, inliers;
  std::vector<int> selection;

  int n_inliers_count = 0;
  std::vector<double> best_coeffs;
  // Iterate
  while (iterations_ < k)
  {
    iterations_ += 1;
    if (iterations_ > max_iterations_)
    {
      if (debug > 0)
        std::cerr << "[findRotationalObjects] RANSAC reached the maximum number of trials." << std::endl;
      break;
    }
    if (debug > 1)
      std::cerr << "[findRotationalObjects] Trial " << iterations_ << " out of " << ceil (k) << ": " << n_inliers_count << " inliers (best is: " << n_best_inliers_count << " so far)." << std::endl;
    
    // Get X samples which satisfy the model criteria
    sac_model_->getSamples (iterations_, selection);

    if (selection.size () == 0) break;

    bool success = sac_model_->computeModelCoefficients (selection);
    sac_model_->selectWithinDistance (sac_model_->getModelCoefficients (), threshold_, inliers);

    if (success)
    {
      if (inliers.size() < 4)
      {
        ROS_ERROR ("Samples are not Inliers. This is possimpible!");
        continue; 
      }

      std::vector<double> coeffs = sac_model_->getModelCoefficients ();
      int after, before = inliers.size ();
      do
      {
        break;
        std::vector<double> backup_coeff (coeffs);
        std::vector<int> backup_inliers (inliers);
        before = inliers.size ();
        ROS_WARN ("before refit: %i inliers", (int)inliers.size());
        if (sac_model_->RefitAxis  (inliers, coeffs))
          sac_model_->selectWithinDistance (coeffs, threshold_, inliers);
        ROS_WARN ("after refit: %i inliers", (int)inliers.size());
        after = inliers.size ();
        if (after > before)
        {
          backup_inliers = inliers;
          backup_coeff = coeffs;
        }
        else
        {
          coeffs = backup_coeff;
          inliers = backup_inliers;
          break;
        }
      } while (true);

      sensor_msgs::PointCloud temp;
      temp.channels.resize (1);
      n_inliers_count = ((double)inliers.size ());// * (1.0+score);

      // Better match ?
      if (n_inliers_count > n_best_inliers_count)
      {
        n_best_inliers_count = n_inliers_count;
        best_inliers = inliers;
        //inliers.clear ();
        best_model = selection;
        best_coeffs = coeffs;
        std::cerr << "BEST COEFFS: ";
        for (unsigned int i = 0; i < best_coeffs.size(); i++)
          std::cerr << best_coeffs[i] << " "; 
        std::cerr << std::endl;

        // Compute the k parameter (k=log(z)/log(1-w^n))
        double w = (double)((double)n_inliers_count / (double)sac_model_->getIndices ()->size ());
        double p_no_outliers = 1 - pow (w, (double)selection.size ());
        p_no_outliers = std::max (std::numeric_limits<double>::epsilon (), p_no_outliers);       // Avoid division by -Inf
        p_no_outliers = std::min (1 - std::numeric_limits<double>::epsilon (), p_no_outliers);   // Avoid division by 0.
        k = log (1 - probability_) / log (p_no_outliers);
      }
    }
  }

  if (best_model.size () != 0)
  {
    std::cerr << "before" <<std::endl;
    //double score = sac_model_->computeScore (best_coeffs, sample_consensus::getMinMaxK (cloud, best_coeffs, best_inliers) , best_inliers, cloud_synth, threshold_);
    std::cerr << "after" <<std::endl;
    //if (debug > 0)
    //  std::cerr << "[RANSAC::computeModel] Model found: " << n_best_inliers_count << " inliers, score is: " << score << std::endl;
    sac_model_->setBestModel (best_model);
    sac_model_->setBestInliers (best_inliers);
    //      pmap.polygons.resize (1);
    //      pmap.polygons[0].points.push_back ();
    return;
  }
  else
    if (debug > 0)
      std::cerr << "[RANSAC::computeModel] Unable to find a solution!" << std::endl;
  return;
}

void RotationalEstimation::init (ros::NodeHandle &nh)
{
  nh_ = nh;
  pub_ = nh_.advertise <sensor_msgs::PointCloud> ("vis_rotational_objects", 1);
  pmap_pub_ = nh_.advertise <mapping_msgs::PolygonalMap> ("vis_rotational_objects_axis", 1);
}

std::vector<std::string> RotationalEstimation::pre () 
  {return std::vector<std::string> ();}

std::vector<std::string> RotationalEstimation::post ()
  {return std::vector<std::string>();}

std::string RotationalEstimation::process (sensor_msgs::PointCloudConstPtr &cloud)
{
  sensor_msgs::PointCloud pc;
  double threshold_ = 0.005;
  double probability_ = 0.99;
  int max_iterations_ = 500;
  mapping_msgs::PolygonalMap pmap;
  pmap.header.frame_id = cloud->header.frame_id;
  
  findRotationalObjects (*cloud, threshold_, probability_, max_iterations_, pc, pmap);
  
  pub_.publish (pc);
  pmap_pub_.publish (pmap);
  return std::string("ok");
}

RotationalEstimation::OutputType RotationalEstimation::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <RotationalEstimation> (argc, argv);
}
#endif

