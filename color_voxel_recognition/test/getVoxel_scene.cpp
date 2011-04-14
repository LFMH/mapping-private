#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include <color_voxel_recognition/Param.hpp>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

/***********************************************************************************************************/
/* voxelize point clouds of scene                                                                          */
/*   Note that this process is necessary only once when the system sees the environment for the first time */
/***********************************************************************************************************/

using namespace std;

int main(int argc, char **argv)
{
  if( argc != 3 ){
    cerr << "usage: " << argv[0] << " [path] <registration_num>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];

  //* read the length of voxel side
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const float voxel_size = Param::readVoxelSize( tmpname );

  //* Voxel Grid
  pcl::VoxelGrid<pcl::PointXYZRGB> grid;
  grid.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid.setSaveLeafLayout(true);

  const int obj_num = atoi(argv[2]);
  pcl::PointCloud<pcl::PointXYZRGB> input_cloud;
  pcl::PointCloud<pcl::PointXYZRGB> output_cloud;
  for(int i=0;i<obj_num;i++){
    sprintf( tmpname, "%s/scene/Points/%05d.pcd", argv[1], i );
    pcl::io::loadPCDFile (tmpname, input_cloud);

    //* voxelize
    grid.setInputCloud ( boost::make_shared<const pcl::PointCloud<pcl::PointXYZRGB> > (input_cloud) );
    grid.filter (output_cloud);

    sprintf( tmpname, "%s/scene/Voxel/%05d.dat", argv[1], i ); 
    pcl::io::savePCDFile (tmpname, output_cloud, true);
  }

  return(0);    
}
