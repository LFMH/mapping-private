#include <ros/ros.h>
#include <ros/node_handle.h>
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/region_growing.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <ias_sample_consensus/sac_model_rotational.h>

using namespace cloud_algos;

void RegionGrowing::setBehaviour (RegionGrowing::Behaviour b)
{
  behaviour_ = b;
}

void RegionGrowing::init (ros::NodeHandle &nh)
{
  nh_ = nh;
  regInitial_ = -1;
  behaviour_ = RegionGrowing::Behaviour();
}

void RegionGrowing::pre ()
{

}

void RegionGrowing::post ()
{

}

std::vector<std::string> RegionGrowing::requires () 
  {return std::vector<std::string> ();}

std::vector<std::string> RegionGrowing::provides ()
  {return std::vector<std::string>();}

void RegionGrowing::setRadius (double radius)
  { radius_ = radius; }

void RegionGrowing::setExtraDimensionIgnore (int dimIdx, double dimVal)
  { dimIdx_ = dimIdx; dimVal_ = dimVal; }

void RegionGrowing::setMaxNrNN (int max_nr_nn)
  { max_nr_nn_ = max_nr_nn; }

void RegionGrowing::GrowFromPoint (const sensor_msgs::PointCloudConstPtr &cloud, int idx)
{
  std::vector<int> k_indices;
  std::vector<float> k_distances;

  if (!behaviour_.grow_from_point (cloud, idx, kdtree_))
    return;
  // ignore points that have a tag set
  if (regions_[idx] != regInitial_)
    return;
  
  // start new cluster
  sensor_msgs::PointCloud cluster;
  cluster.header.frame_id = cloud->header.frame_id;
  cluster.points.push_back (cloud->points[idx]); // first point in cluster is the seed point
  int qIdx = 0; // index in the queue

  while (qIdx < (int) cluster.points.size ())
  {
    if (kdtree_->radiusSearch (cluster.points.at(qIdx), radius_, k_indices, k_distances, max_nr_nn_))
      for (unsigned int j = 1; j < k_indices.size(); j++)
      {
        if ((regions_.at(k_indices.at(j)) == regInitial_) && 
            ((dimIdx_ == -1) || (cloud->channels.at(dimIdx_).values.at(k_indices.at(j)) == dimVal_)))
        {
          if (behaviour_.grow_into_point (cloud, k_indices.at(j), qIdx))
          {
            cluster.points.push_back (cloud->points.at(k_indices.at(j)));
            regions_.at(k_indices.at(j)) = regId_;
          }
        }
      }
    qIdx++;
  }
  regId_++;
  clusters_.push_back (cluster);
}

void RegionGrowing::OptimizedRegionGrowing (const sensor_msgs::PointCloudConstPtr &cloud)
{
  //FOR EACH POINT
  for (unsigned int i = 0; i < cloud->points.size (); i++)
  {
    GrowFromPoint (cloud, i);
  }
}

void RegionGrowing::OptimizedRegionGrowing (const sensor_msgs::PointCloudConstPtr &cloud, std::vector<int> indices)
{
  //FOR EACH POINT
  for (unsigned int i = 0; i < indices.size (); i++)
  {
    GrowFromPoint (cloud, indices[i]);
  }
}

std::string RegionGrowing::process (const boost::shared_ptr<const RegionGrowing::InputType> &input)
{
  const sensor_msgs::PointCloudConstPtr &cloud = input; // this is to make it clear that InputType must match PointCloud
  if (kdtree_ == NULL)
  {
    kdtree_ = new cloud_kdtree::KdTreeANN (*cloud);
  }
  
  clusters_.clear ();
  regId_ = 0;
  regions_ = std::vector<int> (cloud->points.size(), 0);
  OptimizedRegionGrowing (cloud);  

  return std::string("ok");
}

RegionGrowing::OutputType RegionGrowing::output ()
  {return OutputType();}

#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <RegionGrowing> (argc, argv);
}
#endif

