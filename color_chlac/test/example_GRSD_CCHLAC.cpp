#include "color_chlac/grsd_colorCHLAC_tools.h"

using namespace pcl;

//-------
//* main
int main( int argc, char** argv ){
  if( argc != 2 ){
    ROS_ERROR ("Need one parameter! Syntax is: %s {input_pointcloud_filename.pcd}\n", argv[0]);
    return(-1);
  }
  char filename[ 300 ];
  int length;

  //* read
  pcl::PointCloud<PointXYZRGB> input_cloud;
  readPoints( argv[1], input_cloud );

  //* compute normals
  pcl::PointCloud<PointXYZRGBNormal> cloud;
  computeNormal( input_cloud, cloud );

  //* voxelize
  pcl::VoxelGrid<PointXYZRGBNormal> grid;
  pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled;
  getVoxelGrid( grid, cloud, cloud_downsampled );

  //* compute - GRSD -
  std::vector<float> grsd;
  computeGRSD( grid, cloud, cloud_downsampled, grsd );

  //* compute - ColorCHLAC -
  std::vector<float> colorCHLAC;
  computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, 127, 127, 127 );

  //* write
  length = strlen( argv[1] );
  argv[1][ length-4 ] = '\0';
  sprintf(filename,"%s_GRSD_CCHLAC.pcd",argv[1]);
  writeFeature( filename, conc_vector( grsd, colorCHLAC ) );

  return(0);
}
