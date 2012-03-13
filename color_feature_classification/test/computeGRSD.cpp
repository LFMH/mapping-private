#define QUIET

//* NOTICE
//  Rotation is possible also when feature_type is 'g' or 'r'.

#include <color_feature_classification/points_tools.h>
#include "c3_hlac/c3_hlac_tools.h"
#include "vosch/vosch_tools.h"
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>
#include "FILE_MODE"

using namespace pcl;
using namespace terminal_tools;

int rotate_step_num = 0;
int thR, thG, thB;
float voxel_size;
int subdivision_size = 0;
int offset_step = 1; // the step size of offset voxel number for subdivisions

void computeFeature( const PointCloud<PointNormal> input_cloud, std::vector< std::vector<float> > &feature )
{
  int repeat_num_offset = ceil(subdivision_size / offset_step);
  if( subdivision_size == 0 ) repeat_num_offset = 1;

  //* voxelize
  pcl::VoxelGrid<PointNormal> grid;
  pcl::PointCloud<PointNormal> cloud_downsampled;
  getVoxelGrid( grid, input_cloud, cloud_downsampled, voxel_size );
  
  // repeat with changing offset values for subdivisions
  for( int ox = 0; ox < repeat_num_offset; ox++ ){
    for( int oy = 0; oy < repeat_num_offset; oy++ ){
      for( int oz = 0; oz < repeat_num_offset; oz++ ){
	//* extract - GRSD -
	std::vector< std::vector<float> > grsd;
	extractGRSDSignature21( grid, input_cloud, cloud_downsampled, grsd, voxel_size, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step, THEORY_NORMALIZE );
	//extractPlusGRSDSignature110( grid, input_cloud, cloud_downsampled, grsd, voxel_size, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step, THEORY_NORMALIZE );
	const int hist_num = grsd.size(); // number of subdivisions
	
	for( int h=0; h<hist_num; h++ )
	  feature.push_back ( grsd[ h ] );
      }
    }
  }
}

void computeFeature_with_rotate( const PointCloud<PointNormal> input_cloud, std::vector< std::vector<float> > &feature )
{
  int repeat_num_offset = ceil(subdivision_size / offset_step);
  if( subdivision_size == 0 ) repeat_num_offset = 1;

  std::vector< std::vector<float> > grsd;
  std::vector< std::vector<float> > colorCHLAC;
    
  pcl::PointCloud<PointNormal> cloud_normal_r;
  pcl::VoxelGrid<PointNormal> grid_normal;
  pcl::PointCloud<PointNormal> cloud_downsampled_normal;
  
  for(int r3=0; r3 < rotate_step_num; r3++){
    for(int r2=0; r2 < rotate_step_num; r2++){
      for(int r1=0; r1 < rotate_step_num; r1++){
	const double roll  = r3 * M_PI / (2*rotate_step_num);
	const double pan   = r2 * M_PI / (2*rotate_step_num);
	const double roll2 = r1 * M_PI / (2*rotate_step_num);
	
	//* voxelize
	rotatePoints( input_cloud, cloud_normal_r, roll, pan, roll2 );
	getVoxelGrid( grid_normal, cloud_normal_r, cloud_downsampled_normal, voxel_size );	      
	
	// repeat with changing offset values for subdivisions
	for( int ox = 0; ox < repeat_num_offset; ox++ ){
	  for( int oy = 0; oy < repeat_num_offset; oy++ ){
	    for( int oz = 0; oz < repeat_num_offset; oz++ ){
	      //* extract - GRSD -
	      extractGRSDSignature21( grid_normal, cloud_normal_r, cloud_downsampled_normal, grsd, voxel_size, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step, THEORY_NORMALIZE );
	      //extractPlusGRSDSignature110( grid_normal, cloud_normal_r, cloud_downsampled_normal, grsd, voxel_size, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step, THEORY_NORMALIZE );
	      const int hist_num = grsd.size();
	      
	      for( int h=0; h<hist_num; h++ )
		feature.push_back ( grsd[ h ] );
	    }
	  }
	}
	
	if(r2==0) break;
      }
    }
  }
}

//-----------
//* read
bool readPoints_oldPCD( const char *name, pcl::PointCloud<PointNormal>& cloud ){
  pcl::PointCloud<PointXYZ> input_cloud;
  readPoints( name, input_cloud );
  //pcl::io::savePCDFile( "hoge.pcd", input_cloud, true );

  //* compute normals
  pcl::NormalEstimation<PointXYZ, PointNormal> n3d_;
  //n3d_.setKSearch (k_);
  n3d_.setRadiusSearch (0.02);
  n3d_.setSearchMethod ( boost::make_shared<pcl::KdTreeFLANN<PointXYZ> > () );
  n3d_.setInputCloud ( input_cloud.makeShared() );
  n3d_.compute (cloud);

  for ( int i=0; i< (int)input_cloud.points.size (); i++ ){
    cloud.points[ i ].x = input_cloud.points[ i ].x;
    cloud.points[ i ].y = input_cloud.points[ i ].y;
    cloud.points[ i ].z = input_cloud.points[ i ].z;
  }

#ifndef QUIET
 ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud.width * cloud.height), name, pcl::getFieldsList (cloud).c_str ());
#endif
 return(1);
}

// template <typename T>
// bool readPoints_oldPCD( const char *name, pcl::PointCloud<T>& cloud ){
// // #if 1 
// //  sensor_msgs::PointCloud2 cloud_blob;
// //   pcl::io::loadPCDFile (name, cloud_blob);
// //   fromROSMsg (cloud_blob, cloud);
// //   pcl::io::savePCDFile( "hoge.pcd", cloud, true );
// // #else
// //   pcl::PCDReader pcd;
// //   sensor_msgs::PointCloud2 cloud_blob;
// //   if (pcd.read (name, cloud_blob)<  0)
// //     return (-1);
// //   fromROSMsg (cloud_blob, cloud);
// // #endif

//   FILE *fp = fopen( name, "rb" );
//   char line[ 1024 ];

//   while( fgets( line, sizeof(line), fp ) != NULL )
//     if( line[ 0 ] != '#' ) break;
//   fgets( line, sizeof(line), fp );

//   int p_num;
//   fscanf( fp, "POINTS %d", &p_num );
//   fgets( line, sizeof(line), fp );

//   cloud.points.resize( p_num );
//   //float x, y, z, intensity, distance, sid, pid, vx, vy, vz, idx, nx, ny, nz, c, bp, k;
//   float x, y, z, intensity, distance, vx, vy, vz, nx, ny, nz, c;
//   int sid, pid, idx, bp, k;

//   for( int p=0; p<p_num; p++ ){
//     fread( &x, sizeof(float), 1, fp );
//     fread( &y, sizeof(float), 1, fp );
//     fread( &z, sizeof(float), 1, fp );
//     fread( &intensity, sizeof(float), 1, fp );
//     fread( &distance, sizeof(float), 1, fp );
//     fread( &sid, sizeof(int), 1, fp );
//     fread( &pid, sizeof(int), 1, fp );
//     fread( &vx, sizeof(float), 1, fp );
//     fread( &vy, sizeof(float), 1, fp );
//     fread( &vz, sizeof(float), 1, fp );
//     fread( &idx, sizeof(int), 1, fp );
//     fread( &nx, sizeof(float), 1, fp );
//     fread( &ny, sizeof(float), 1, fp );
//     fread( &nz, sizeof(float), 1, fp );
//     fread( &c, sizeof(float), 1, fp );
//     fread( &bp, sizeof(int), 1, fp );
//     fread( &k, sizeof(int), 1, fp );
//     printf("%f %f %f %f %f %d %d %f %f %f %d %f %f %f %f %d %d\n", x, y, z, intensity, distance, sid, pid, vx, vy, vz, idx, nx, ny, nz, c, bp );
//   }
//   fclose( fp );

// #ifndef QUIET
//  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud.width * cloud.height), name, pcl::getFieldsList (cloud).c_str ());
// #endif
//  return(1);
// }

//-------
//* main
int main( int argc, char** argv ){
  if( argc < 4 ){
    ROS_ERROR ("Need at least three parameters! Syntax is: %s {input_pointcloud_filename.pcd} [options] {config_txt_path} {output_histogram_filename.pcd}\n", argv[0]);
    ROS_INFO ("                          -rotate rotate_step_num = e.g. 3 for 30 degrees rotation\n");
    ROS_INFO ("                          -subdiv N = subdivision size (e.g. 10 voxels)\n");
    ROS_INFO ("                          -offset n = offset step for subdivisions (e.g. 5 voxels)\n");
    return(-1);
  }

  // check test mode
  int test_mode = 0;
  parse_argument (argc, argv, "-test", test_mode);

  // rotate step num
  if( parse_argument (argc, argv, "-rotate", rotate_step_num) > 0 ){
    if ( rotate_step_num < 1 ){
      print_error ("Invalid rotate_step_num (%d)!\n", rotate_step_num);
      return (-1);
    }
  }

  //* voxel size (downsample_leaf)
  char voxel_size_file[1024];
  sprintf(voxel_size_file, "%s/voxel_size.txt", argv[argc-2]);
  FILE *fp = fopen( voxel_size_file, "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  // subdivision size
  if( parse_argument (argc, argv, "-subdiv", subdivision_size) > 0 ){
    if ( subdivision_size < 0 ){
      print_error ("Invalid subdivision size (%d)! \n", subdivision_size);
      return (-1);
    }
  }

  // offset step
  if( parse_argument (argc, argv, "-offset", offset_step) > 0 ){
    if ( ( offset_step < 1 ) || ( offset_step >= subdivision_size ) ){
      print_error ("Invalid offset step (%d)! (while subdivision size is %d.)\n", offset_step, subdivision_size);
      return (-1);
    }
  }

  //* read
  pcl::PointCloud<PointNormal> input_cloud;
  if( test_mode ){ // rotate points to a random posture
    pcl::PointCloud<PointNormal> original_cloud;
    readPoints_oldPCD( argv[1], original_cloud );
    unsigned int time_val;
    fp = fopen( "time_for_srand.txt", "r" );
    fscanf( fp, "%d", &time_val );
    fclose( fp );
    srand( time_val );
    const double roll  = (rand()%360) * M_PI / 180.0;
    const double pan   = (rand()%360) * M_PI / 180.0;
    const double roll2 = (rand()%360) * M_PI / 180.0;
    ROS_INFO ("Rotate test point cloud to a random posture. (%f %f %f)",roll,pan,roll2);
    rotatePoints( original_cloud, input_cloud, roll, pan, roll2 );
  }
  else
    readPoints_oldPCD( argv[1], input_cloud );

  //* compute feature
  std::vector< std::vector<float> > feature;
  if( rotate_step_num == 0 )
    computeFeature( input_cloud, feature );
  else
    computeFeature_with_rotate( input_cloud, feature );
  
  //* write
  writeFeature( argv[ argc - 1 ], feature );
  
  return(0);
}
