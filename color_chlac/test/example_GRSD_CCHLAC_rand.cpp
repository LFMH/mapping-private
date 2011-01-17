#include "color_chlac/grsd_colorCHLAC_tools.h"

using namespace pcl;

//#define DIVID_TEST

//-------
//* main
int main( int argc, char** argv ){
  srand( (unsigned) time(NULL) );

  //* voxel size (downsample_leaf)
  const double voxel_size = 0.01;

  //* creat
  pcl::PointCloud<PointXYZRGB> input_cloud;
  input_cloud.width    = 6000;
  input_cloud.height   = 1;
  input_cloud.is_dense = false;
  input_cloud.points.resize (input_cloud.width * input_cloud.height);

  int red, green, blue, rgb;
  for (size_t i = 0; i < input_cloud.points.size (); ++i)
  {
    input_cloud.points[i].x = 0.1 * rand () / (RAND_MAX + 1.0);
    input_cloud.points[i].y = 0.1 * rand () / (RAND_MAX + 1.0);
    input_cloud.points[i].z = 0.1 * rand () / (RAND_MAX + 1.0);
    red   = (int)( 255.0 * rand () / (RAND_MAX + 1.0) );
    green = (int)( 255.0 * rand () / (RAND_MAX + 1.0) );
    blue  = (int)( 255.0 * rand () / (RAND_MAX + 1.0) );
    //std::cout << input_cloud.points[i].x << " " << input_cloud.points[i].y << " " << input_cloud.points[i].z << std::endl;
    //std::cout << red << " " << green << " " << blue << std::endl;
    rgb = red << 16 | green << 8 | blue;
    input_cloud.points[i].rgb = *reinterpret_cast<float*>(&rgb);
  }
  pcl::io::savePCDFile ("test.pcd", input_cloud, true);

  //* compute normals
  pcl::PointCloud<PointXYZRGBNormal> cloud;
  computeNormal( input_cloud, cloud );

  //* voxelize
  pcl::VoxelGrid<PointXYZRGBNormal> grid;
  pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled;
  getVoxelGrid( grid, cloud, cloud_downsampled, voxel_size );

#ifdef DIVID_TEST
  //* compute - GRSD -
  std::vector< std::vector<float> > grsd;
  computeGRSD( grid, cloud, cloud_downsampled, grsd, voxel_size, 10 );

  //* compute - ColorCHLAC -
  std::vector< std::vector<float> > colorCHLAC;
  //computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size, 10 );
  computeColorCHLAC_RI( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size, 10 );

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
  writeFeature( "test_GRSD_CCHLAC_rand.pcd", feature );
  writeFeature( "debug.pcd", debug );

#else
  //* compute - GRSD -
  std::vector<float> grsd;
  computeGRSD( grid, cloud, cloud_downsampled, grsd, voxel_size);

  //* compute - ColorCHLAC -
  std::vector<float> colorCHLAC;
  //computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size );
  computeColorCHLAC_RI( grid, cloud_downsampled, colorCHLAC, 127, 127, 127, voxel_size );

  //* write
  writeFeature( "test_GRSD_CCHLAC_rand.pcd", conc_vector( grsd, colorCHLAC ) );
#endif

  return(0);
}
