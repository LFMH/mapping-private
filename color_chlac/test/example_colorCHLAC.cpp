#include <sys/time.h>
#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include "pcl/features/normal_3d.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"
#include "pcl/kdtree/kdtree.h"
//#include "pcl/kdtree/kdtree_ann.h"
#include "pcl/kdtree/organized_data.h"
#include "color_chlac/color_chlac.h"

using namespace pcl;

//* time
double t1,t2;
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

inline void getVoxelGrid ( const pcl::PointCloud<PointXYZRGB> &cloud, pcl::PointCloud<PointXYZRGB> &voxel_grid, const float voxel_size )
{
  const float voxel_size_ = 1 / voxel_size;
  voxel_grid.points.resize ( cloud.size() );
  voxel_grid.width = cloud.width;
  voxel_grid.height = cloud.height;

  for (size_t i = 0; i < cloud.size (); ++i){
    voxel_grid.points[ i ].x = (int)(floor (cloud.points[ i ].x * voxel_size_));
    voxel_grid.points[ i ].y = (int)(floor (cloud.points[ i ].y * voxel_size_));
    voxel_grid.points[ i ].z = (int)(floor (cloud.points[ i ].z * voxel_size_));
    voxel_grid.points[ i ].rgb = cloud.points[ i ].rgb;
  }
}

template <typename T>
bool readPoints( const char *name, T& cloud_object_cluster ){
  if (pcl::io::loadPCDFile (name, cloud_object_cluster) == -1){
    ROS_ERROR ("Couldn't read file %s",name);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud_object_cluster.width * cloud_object_cluster.height), name, pcl::getFieldsList (cloud_object_cluster).c_str ());
  return(1);
}

bool writeColorCHLAC(const char *name, pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster ){
  pcl::PointCloud<PointXYZRGB>::ConstPtr cloud_ = boost::make_shared<const pcl::PointCloud<PointXYZRGB> > (cloud_object_cluster);

  // ---[ Create the voxel grid
  pcl::PointCloud<PointXYZRGB> cloud_downsampled;
  pcl::VoxelGrid<PointXYZRGB> grid_;
  double downsample_leaf_ = 0.01;                          // 1cm voxel size by default
  grid_.setLeafSize (downsample_leaf_, downsample_leaf_, downsample_leaf_);

  grid_.setInputCloud (cloud_);
  grid_.setSaveLeafLayout(true);
  grid_.filter (cloud_downsampled);
  pcl::PointCloud<PointXYZRGB>::ConstPtr cloud_downsampled_;
  cloud_downsampled_.reset (new pcl::PointCloud<PointXYZRGB> (cloud_downsampled));
  
//   // Transform each point to the center of its nearest voxel
//   pcl::PointCloud<PointXYZRGB> voxel_grid;
//   getVoxelGrid( cloud_downsampled, voxel_grid, downsample_leaf_ );
//   pcl::PointCloud<PointXYZRGB>::ConstPtr voxel_grid_;
//   voxel_grid_.reset (new pcl::PointCloud<PointXYZRGB> (voxel_grid));

  // ---[ Compute ColorCHLAC
  pcl::PointCloud<ColorCHLACSignature981> colorCHLAC_signature;
  pcl::ColorCHLACEstimation<PointXYZRGB> colorCHLAC_;
  KdTree<PointXYZRGB>::Ptr normals_tree_ = boost::make_shared<pcl::KdTreeFLANN<PointXYZRGB> > ();

  colorCHLAC_.setRadiusSearch (1.8);
  colorCHLAC_.setSearchMethod (normals_tree_);
  colorCHLAC_.setColorThreshold( 127, 127, 127 );
  //colorCHLAC_.setInputCloud (voxel_grid_);
  colorCHLAC_.setVoxelFilter (grid_);
  colorCHLAC_.setInputCloud (cloud_downsampled_);
  t1 = my_clock();
  colorCHLAC_.compute( colorCHLAC_signature );
  t2 = my_clock();
  ROS_INFO (" %d colorCHLAC estimated. (%f sec)", (int)colorCHLAC_signature.points.size (), t2-t1);
  pcl::io::savePCDFile (name, colorCHLAC_signature);
  ROS_INFO("ColorCHLAC signatures written to %s", name);
  
  return 1;
}

int main( int argc, char** argv ){
  if( argc != 2 ){
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }
  char filename[ 300 ];
  int length;

  //* write - ColorCHLAC -
  pcl::PointCloud<PointXYZRGB> cloud_object_cluster2;
  readPoints( argv[1], cloud_object_cluster2 );
  length = strlen( argv[1] );
  argv[1][ length-4 ] = '\0';
  sprintf(filename,"%s_colorCHLAC.pcd",argv[1]);
  writeColorCHLAC( filename, cloud_object_cluster2 );

  return(0);
}
