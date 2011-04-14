#include <iostream>
#include <c3_hlac/c3_hlac_tools.h>
#include <color_voxel_recognition/Param.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

/***********************************************************************************************************/
/* extract C^3-HLAC features from all the subdivisions of scene                                            */
/*   Note that this process is necessary only once when the system sees the environment for the first time */
/***********************************************************************************************************/

using namespace std;
using namespace pcl;

int main(int argc, char** argv){
  if( argc != 3 ){
    cerr << "usage: " << argv[0] << " [path] <registration_num>" << endl;
    exit( EXIT_FAILURE );
  }
  const int file_num = atoi( argv[2] );
  char tmpname[ 1000 ];

  //* read the number of voxels in each subdivision's side of scene
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const int subdivision_size = Param::readBoxSize_scene( tmpname );

  //* read the length of voxel side
  const float voxel_size = Param::readVoxelSize( tmpname );

  //* read the threshold for RGB binalize
  int color_threshold_r, color_threshold_g, color_threshold_b;
  sprintf( tmpname, "%s/param/color_threshold.txt", argv[1] );
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b, tmpname );

  //* Voxel Grid
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid.setSaveLeafLayout(true);

  std::vector< std::vector<float> > c3_hlac;
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled;
  for( int i=0; i<file_num; i++ ){
    sprintf( tmpname, "%s/scene/Points/%05d.pcd", argv[1], i );
    pcl::io::loadPCDFile (tmpname, input_cloud);

    //* voxelize
    grid.setInputCloud ( boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB> > (input_cloud) );
    grid.filter (cloud_downsampled);

    //* extract features
    extract_C3_HLAC_Signature981( grid, cloud_downsampled, c3_hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );
    sprintf( tmpname, "%s/scene/Features/%05d.pcd", argv[1], i );
    writeFeature( tmpname, c3_hlac );
  }
  
  return 0;
}
