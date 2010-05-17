#include <ros/ros.h>
#include <ros/node_handle.h>
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/rotational_estimation.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <ias_sample_consensus/sac_model_rotational.h>
#include <point_cloud_mapping/cloud_io.h>

using namespace cloud_algos;

void
  findRotationalObjects (sensor_msgs::PointCloud cloud, double threshold_, double probability_, 
                         int max_iterations_, boost::shared_ptr<triangle_mesh::TriangleMesh> mesh_synth, boost::shared_ptr<mapping_msgs::PolygonalMap> &pmap, boost::shared_ptr<position_string_rviz_plugin::PositionStringList> vis_text, std::vector<int> &final_inliers, std::vector<int> &final_outliers)
{
  int debug = 0;
  ias_sample_consensus::SACModelRotational *sac_model_ = new ias_sample_consensus::SACModelRotational (pmap);
  if (debug > 0)
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
        ROS_ERROR ("Samples are not Inliers. This is impossible!");
        continue; 
      }

      std::vector<double> coeffs = sac_model_->getModelCoefficients ();
      int after, before = inliers.size ();
      do
      {
        std::vector<double> backup_coeff (coeffs);
        std::vector<int> backup_inliers (inliers);
        before = inliers.size ();
        ROS_WARN ("before refit: %i inliers", (int)inliers.size());
        sac_model_->refitModel  (inliers, coeffs);
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
        if (debug > 0)
          std::cerr << "BEST COEFFS: ";
        for (unsigned int i = 0; i < best_coeffs.size(); i++)
          if (debug > 0)
            std::cerr << best_coeffs[i] << " "; 
        if (debug > 0)
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
  if (debug > 0)
    std::cerr << "[findRotationalObjects] finished after " << iterations_ << " iteration" << std::endl;

  if (best_model.size () != 0)
  {
    if (debug > 0)
      std::cerr << "before" <<std::endl;
    //cloud_geometry::getPointCloud (cloud, best_inliers, *cloud_synth);
    sac_model_->samplePointsOnRotational (best_coeffs, best_inliers, mesh_synth);
    
    final_inliers = best_inliers;    
    int last_index = 0;
    for (std::vector<int>::iterator it = best_inliers.begin(); it != best_inliers.end(); it++)
    {
      if ((*it) - last_index > 1) // we skipped something
        for (int skipped = last_index + 1; skipped < (*it); skipped++)
          final_outliers.push_back (skipped);
      last_index = *it;
    }
 
    std::stringstream ss;
    ss << best_inliers.size() << " inliers";
    vis_text->texts.push_back (ss.str());
    geometry_msgs::Point32 p;
    p.x = best_coeffs[0];
    p.y = best_coeffs[1];
    p.z = best_coeffs[2];
    vis_text->poses.push_back (p);
    //double score = sac_model_->computeScore (best_coeffs, sample_consensus::getMinMaxK (cloud, best_coeffs, best_inliers) , best_inliers, cloud_synth, threshold_);
    if (debug > 0)
      std::cerr << "after" <<std::endl;
    //if (debug > 0)
    //  std::cerr << "[RANSAC::computeModel] Model found: " << n_best_inliers_count << " inliers, score is: " << score << std::endl;
    sac_model_->setBestModel (best_model);
    sac_model_->setBestInliers (best_inliers);
//          pmap->polygons.resize (1);
//          for (unsigned int i = 0 ; i < best_model.size() ; i++)
//            pmap->polygons[0].points.push_back (cloud.points[best_model[i]]);
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
  nh_.param("threshold", threshold_, 0.004);
  nh_.param("probability", probability_, 0.99);
  nh_.param("max_iterations", max_iterations_, 100);
  ROS_INFO("threshold_: %lf, probability_: %lf, max_iterations_: %d", threshold_, 
           probability_, max_iterations_);
  vis_cloud_pub_ = nh_.advertise <sensor_msgs::PointCloud> ("vis_rotational_objects", 1);
  vis_cloud_outliers_pub_ = nh_.advertise <sensor_msgs::PointCloud> (output_cloud_outliers_, 1);
  vis_pmap_pub_ = nh_.advertise <mapping_msgs::PolygonalMap> ("vis_rotational_objects_axis", 1);
  vis_text_pub_ = nh_.advertise <position_string_rviz_plugin::PositionStringList> ("vis_rotational_objects_text", 1);
}

void RotationalEstimation::pre ()
{
  vis_pmap_ = boost::shared_ptr<mapping_msgs::PolygonalMap> 
                           (new mapping_msgs::PolygonalMap ());
  vis_cloud_ = boost::shared_ptr<sensor_msgs::PointCloud> 
                           (new sensor_msgs::PointCloud ());
  vis_cloud_outliers_ = boost::shared_ptr<sensor_msgs::PointCloud> 
                           (new sensor_msgs::PointCloud ());
  vis_text_ = boost::shared_ptr<position_string_rviz_plugin::PositionStringList> 
                           (new position_string_rviz_plugin::PositionStringList ());
}

void RotationalEstimation::post ()
{
  vis_pmap_pub_.publish (vis_pmap_);
  vis_cloud_pub_.publish (vis_cloud_);
  vis_text_pub_.publish (vis_text_);
  //publish outliers
  vis_cloud_outliers_ = getOutliers();
  vis_cloud_outliers_pub_.publish(vis_cloud_outliers_);
}

std::vector<std::string> RotationalEstimation::requires () 
{
  return std::vector<std::string> ();
}

std::vector<std::string> RotationalEstimation::provides ()
{
  return std::vector<std::string>();
}

std::string RotationalEstimation::process (const boost::shared_ptr<const RotationalEstimation::InputType> cloud)
{
  cloud_ = cloud;
  mesh_ = boost::shared_ptr <triangle_mesh::TriangleMesh>(new triangle_mesh::TriangleMesh ());
  mesh_->header = cloud_->header;
  mesh_->sending_node = ros::this_node::getName();
  if (debug_ > 0)
    std::cerr<<"[RotationalEstimation::process] Line" << __LINE__ << std::endl;
  if (debug_ > 0)
    std::cerr<<"[RotationalEstimation::process] Line" << __LINE__ << std::endl;
  
  findRotationalObjects (*cloud, threshold_, probability_, max_iterations_, mesh_, vis_pmap_, vis_text_, inliers_, outliers_);
  if (debug_ > 0)
    std::cerr<<"[RotationalEstimation::process] Line" << __LINE__ << std::endl;
  vis_pmap_->header.frame_id = cloud->header.frame_id;
  vis_cloud_->header.frame_id = cloud->header.frame_id;
  vis_text_->header.frame_id = cloud->header.frame_id;
  if (debug_ > 0)
    std::cerr<<"[RotationalEstimation::process] Line" << __LINE__ << std::endl;
  return std::string("ok");
}

boost::shared_ptr<sensor_msgs::PointCloud> RotationalEstimation::getOutliers ()
{
  boost::shared_ptr<sensor_msgs::PointCloud> ret (new sensor_msgs::PointCloud);
  //ROS_INFO("created PointCloud object: 0x%x", (void*) ret.get()); - ZOLI COMMENTED THIS TO GET RID OF WARNING, SUPPOSING WAS ONLY DEBUG :)
  
  ret->header = cloud_->header;
  int channel_index = getChannelIndex (cloud_, "index");
  int channel_line = getChannelIndex (cloud_, "line");
  if (channel_line != -1)
  {
    ret->channels.resize(2);
    ret->channels[1].name = cloud_->channels[channel_line].name;
    ret->channels[1].values.resize(outliers_.size());
    ret->channels[0].name = cloud_->channels[channel_index].name;
    ret->channels[0].values.resize(outliers_.size());
  }
  else if (channel_index != -1)
  {
    ret->channels.resize(1);
    ret->channels[0].name = cloud_->channels[channel_index].name;
    ret->channels[0].values.resize(outliers_.size());
  }
   
  
  ret->points.resize(outliers_.size());

  if (debug_ > 0)
    ROS_INFO("outliers_.size(): %ld inliers.size() %ld", outliers_.size(), inliers_.size());
  for (unsigned int i = 0; i < outliers_.size (); i++)
  {
    ret->points.at(i) = cloud_->points.at (outliers_.at (i));
    if (channel_index != -1)
      ret->channels[0].values.at(i) = cloud_->channels[channel_index].values.at (outliers_.at (i));
    if (channel_line != -1)
      ret->channels[1].values.at(i) = cloud_->channels[channel_line].values.at (outliers_.at (i));
  }
  return ret;
}

boost::shared_ptr<sensor_msgs::PointCloud> RotationalEstimation::getInliers ()
{
  boost::shared_ptr<sensor_msgs::PointCloud> ret (new sensor_msgs::PointCloud ());
  ret->header = cloud_->header;
  for (unsigned int i = 0; i < inliers_.size (); i++)
    ret->points.push_back (cloud_->points.at (inliers_.at (i)));
  return ret;
}

boost::shared_ptr<RotationalEstimation::OutputType> RotationalEstimation::output ()
{
  return mesh_;
}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <RotationalEstimation> (argc, argv);
}
#endif

