#include <sys/time.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include "pcl/filters/voxel_grid.h"
#include "pcl/features/rsd.h"

typedef pcl::KdTree<pcl::PointNormal>::Ptr KdTreePtr;

//* time
double t1,t2;
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

int main( int argc, char** argv ){
  if( argc != 2 ){
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }

  double fixed_radius_search = 0.03;
  // read input cloud
  pcl::PointCloud<pcl::PointNormal> input_cloud;
  pcl::PCDReader reader;
  pcl::PCDWriter writer;
  reader.read ( argv[1], input_cloud);

  // Create the voxel grid
  pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud = boost::make_shared<const pcl::PointCloud<pcl::PointNormal> > (input_cloud);
  pcl::PointCloud<pcl::PointNormal> cloud_downsampled;
  pcl::VoxelGrid<pcl::PointNormal> grid;
  double downsample_leaf = 0.01;                          // 1cm voxel size by default
  grid.setLeafSize (downsample_leaf, downsample_leaf, downsample_leaf);
  grid.setInputCloud (cloud);
  grid.setSaveLeafLayout(true);
  grid.filter (cloud_downsampled);
  pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud_downsampled_ptr;
  cloud_downsampled_ptr.reset (new pcl::PointCloud<pcl::PointNormal> (cloud_downsampled));
  

  // Compute RSD
  pcl::RadiusSurfaceDescriptor <pcl::PointNormal, pcl::PointNormal, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud(cloud_downsampled_ptr);
  rsd.setSearchSurface(cloud);
  rsd.setInputNormals(cloud);
  ROS_INFO("radius search: %f", max(fixed_radius_search, downsample_leaf/2 * sqrt(3)));
  rsd.setRadiusSearch(max(fixed_radius_search, downsample_leaf/2 * sqrt(3)));
  pcl::KdTree<pcl::PointNormal>::Ptr tree2 = boost::make_shared<pcl::KdTreeANN<pcl::PointNormal> > ();
  tree2->setInputCloud (cloud);
  rsd.setSearchMethod(tree2);
  pcl::PointCloud<pcl::PrincipalRadiiRSD> radii;
  rsd.compute(radii);
  writer.write("radii.pcd", radii, false);
//   set input cloud - downsampled
//     set surface - original cloud
// set normal - set normals
// set k search (to get all points in voxel)
// computeRSD
  
  // Get rmin/rmax for adjacent 27 voxel
  // Get transition matrix
  return(0);
}
