#include <ros/ros.h>
#include <ros/node_handle.h>
#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/rotational_estimation.h>
#include <sensor_msgs/PointCloud.h>
#include <mapping_msgs/PolygonalMap.h>
#include <ias_sample_consensus/sac_model_rotational.h>

void RegionGrowing::init (ros::NodeHandle &nh)
{
  nh_ = nh;
}

std::vector<std::string> RegionGrowing::pre () 
  {return std::vector<std::string> ();}

std::vector<std::string> RegionGrowing::post ()
  {return std::vector<std::string>();}


void RegionGrowing::OptimizedRegionSegmentation ()
{
  kd_tree* tree;
  std::vector<PointCloud> clusters;
  //FOR EACH POINT pt
  {
    // ignore points that have a tag set
    if (pt[regIdx] != regInitial)
      continue;
    
    // start new cluster
    sensor_msgs::PointCloud cluster;
    cluster.header.frame_id = cloud->header.frame_id;
    cloud->points.push_back (pt); // first point in cluster is the seed point
    int qIdx = 0; // index in the queue

    while (qIdx < (int) q.size ())
    {
      int k = kd_tree->annkFRSearch (points[q[qIdx]], sqr_thresh, 0, NULL, NULL, 0.0);
      if (k > max_nr_nn)
        k = max_nr_nn;
      kd_tree->annkFRSearch (points[q[qIdx]], sqr_thresh, k, nnIdx, sqrDists, 0.0);
      for (int j = 1; j < k; j++)
      {
        if ((points[nnIdx[j]][regIdx] == regInitial) && ((dimIdx == -1) || (points[nnIdx[j]][dimIdx] == dimVal)))
        {
          q.push_back (nnIdx[j]);
        }
      }
      qIdx++;
    }
  }
}

std::string RegionGrowing::process (sensor_msgs::PointCloudConstPtr cloud)
{
//  for (std::vector<int>::iterator it = indices.begin (); it != indices.end (); it++)
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

