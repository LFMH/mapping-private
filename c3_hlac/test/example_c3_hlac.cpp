#include "c3_hlac/c3_hlac_tools.h"

using namespace pcl;

//#define DIVID_TEST

int main( int argc, char** argv ){
  if( argc != 2 ){
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }

  //* voxel size (downsample_leaf)
  const double voxel_size = 0.01;

  //* read
  pcl::PointCloud<PointXYZRGB> input_cloud;
  if (pcl::io::loadPCDFile (argv[1], input_cloud) == -1){
    ROS_ERROR ("Couldn't read file %s",argv[1]);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(input_cloud.width * input_cloud.height), argv[1], pcl::getFieldsList (input_cloud).c_str ());

  //* voxelize
  pcl::VoxelGrid<PointXYZRGB> grid;
  pcl::PointCloud<PointXYZRGB> cloud_downsampled;
  getVoxelGrid( grid, input_cloud, cloud_downsampled, voxel_size );

#ifdef DIVID_TEST
  //* extract - C3_HLAC -
  std::vector< std::vector<float> > c3_hlac;
  extract_C3_HLAC_Signature981( grid, cloud_downsampled, c3_hlac, 127, 127, 127, voxel_size, 10 );

  //* write
  writeFeature( "c3hlac.pcd", c3_hlac );

#else
  //* extract - C3_HLAC -
  std::vector<float> c3_hlac;
  extract_C3_HLAC_Signature981( grid, cloud_downsampled, c3_hlac, 127, 127, 127, voxel_size );

  //* write
  writeFeature( "c3hlac.pcd", c3_hlac );
#endif

  return(0);
}
