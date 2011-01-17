#include "color_chlac/grsd_colorCHLAC_tools.h"

using namespace pcl;

//#define DIVID_TEST

//-------
//* main
int main( int argc, char** argv ){
  if( argc != 2 ){
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }
  //* voxel size (downsample_leaf)
  const double voxel_size = 0.01;

  //* read
  pcl::PointCloud<PointXYZRGB> input_cloud;
  readPoints( argv[1], input_cloud );

  //* voxelize
  pcl::VoxelGrid<PointXYZRGB> grid;
  pcl::PointCloud<PointXYZRGB> cloud_downsampled;
  getVoxelGrid( grid, input_cloud, cloud_downsampled, voxel_size );

#ifdef DIVID_TEST
  //* compute - ColorCHLAC -
  std::vector< std::vector<float> > colorCHLAC;
  computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size, 10 );
  //computeColorCHLAC_RI( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size, 10 );

  //* write
  int length = strlen( argv[1] );
  argv[1][ length-4 ] = '\0';
  char filename[ 300 ];
  sprintf(filename,"%s_colorCHLAC.pcd",argv[1]);
  writeFeature( filename, colorCHLAC );
  writeFeature( "debug.pcd", debug );

#else
  //* compute - ColorCHLAC -
  std::vector<float> colorCHLAC;
  computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size );
  //computeColorCHLAC_RI( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size );

  //* write
  int length = strlen( argv[1] );
  argv[1][ length-4 ] = '\0';
  char filename[ 300 ];
  sprintf(filename,"%s_colorCHLAC.pcd",argv[1]);
  writeFeature( filename, colorCHLAC );
#endif

  return(0);
}
