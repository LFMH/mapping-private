#include "color_chlac/grsd_colorCHLAC_tools.h"

using namespace pcl;
#define VERBOSE 1
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
  //pcl::io::savePCDFile ("hogehoge.pcd",input_cloud,true);
  double t1 = my_clock();

  //* compute normals
  pcl::PointCloud<PointXYZRGBNormal> cloud;
  computeNormal( input_cloud, cloud );
  ROS_INFO("Normal compute done in %f seconds.", my_clock()-t1);

  //* voxelize
  pcl::VoxelGrid<PointXYZRGBNormal> grid;
  pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled;
  getVoxelGrid( grid, cloud, cloud_downsampled, voxel_size );

#ifdef DIVID_TEST
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, 10 );
  //extractPlusGRSDSignature110( grid, cloud, cloud_downsampled, grsd, voxel_size, 10 );

  //* extract - ColorCHLAC -
  std::vector< std::vector<float> > colorCHLAC;
  extractColorCHLACSignature981( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size, 10 );
  //extractColorCHLACSignature117( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size, 10 );

  //* concatenate
  std::vector< std::vector<float> > feature;
  std::vector<float> debug( 137 );
  for( int i=0; i<137; i++ ) debug[ i ] = 0;
  const int hist_num = grsd.size();
  for( int h=0; h<hist_num; h++ ){
    feature.push_back( conc_vector( grsd[ h ], colorCHLAC[ h ] ) );
    for( int i=0; i<137; i++ )
      debug[ i ] += (conc_vector( grsd[ h ], colorCHLAC[ h ] ))[ i ];
  }

  //* write
  int length = strlen( argv[1] );
  argv[1][ length-4 ] = '\0';
  char filename[ 300 ];
  sprintf(filename,"%s_GRSD_CCHLAC.pcd",argv[1]);
  writeFeature( filename, feature );
  writeFeature( "debug.pcd", debug );

#else
  //* extract - GRSD -
  std::vector<float> grsd;
  extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size);
  //extractPlusGRSDSignature110( grid, cloud, cloud_downsampled, grsd, voxel_size);

  //* extract - ColorCHLAC -
  std::vector<float> colorCHLAC;
  extractColorCHLACSignature981( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size );
  //extractColorCHLACSignature117( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size );
#ifdef VERBOSE
  //  ROS_INFO("VOSCH %10f", (my_clock()-t1)/input_cloud.points.size());
  ROS_INFO("VOSCH %ld", input_cloud.points.size());
#endif
  //* write
  int length = strlen( argv[1] );
  argv[1][ length-4 ] = '\0';
  char filename[ 300 ];
  sprintf(filename,"%s_GRSD_CCHLAC.pcd",argv[1]);
  writeFeature( filename, conc_vector( grsd, colorCHLAC ) );
#endif

  return(0);
}
