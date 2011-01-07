#define QUIET

#include <color_feature_classification/points_tools.hpp>
#include "color_chlac/grsd_colorCHLAC_tools.h"
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>

using namespace pcl;
using namespace terminal_tools;

int rotate_step_num = 1;
int thR, thG, thB;
float voxel_size;
int subdivision_size = 0;
const int offset_step = 5;//2; // the step size of offset voxel number for subdivisions

void computeFeature( const PointCloud<PointXYZRGB> input_cloud, std::vector< std::vector<float> > &feature, const char feature_type )
{
  const int repeat_num_offset = ceil(subdivision_size / offset_step);
  //* compute normals
  pcl::PointCloud<PointXYZRGBNormal> cloud;
  computeNormal( input_cloud, cloud );
  
  //* voxelize
  pcl::VoxelGrid<PointXYZRGBNormal> grid;
  pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled;
  getVoxelGrid( grid, cloud, cloud_downsampled, voxel_size );
  
  // repeat with changing offset values for subdivisions
  for( int ox = 0; ox < repeat_num_offset; ox++ ){
    for( int oy = 0; oy < repeat_num_offset; oy++ ){
      for( int oz = 0; oz < repeat_num_offset; oz++ ){
	//* compute - GRSD -
	std::vector< std::vector<float> > grsd;
	computeGRSD( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step );
	const int hist_num = grsd.size(); // number of subdivisions
	
	if( feature_type == 'g' ){
	  for( int h=0; h<hist_num; h++ )
	    feature.push_back ( grsd[ h ] );
	}
	else{
	  //* compute - ColorCHLAC - rotation-invariant
	  std::vector< std::vector<float> > colorCHLAC;
	  computeColorCHLAC_RI( grid, cloud_downsampled, colorCHLAC, thR, thG, thB, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step );
	  for( int h=0; h<hist_num; h++ )
	    feature.push_back ( conc_vector( grsd[ h ], colorCHLAC[ h ] ) );
	}
      }
    }
  }
}

void computeFeature_with_rotate( const PointCloud<PointXYZRGB> input_cloud, std::vector< std::vector<float> > &feature, const char feature_type )
{
  const int repeat_num_offset = ceil(subdivision_size / offset_step);

  // necessary if feature_type == 'd'
  pcl::PointCloud<PointXYZRGBNormal> cloud_normal;
  pcl::PointCloud<PointXYZRGBNormal> cloud_normal_r;
  pcl::VoxelGrid<PointXYZRGBNormal> grid_normal;
  pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled_normal;

  // necessary if feature_type == 'c'
  pcl::PointCloud<PointXYZRGB> input_cloud_r; // rotate
  pcl::VoxelGrid<PointXYZRGB> grid;
  pcl::PointCloud<PointXYZRGB> cloud_downsampled;

  std::vector< std::vector<float> > grsd;
  std::vector< std::vector<float> > colorCHLAC;
  if( feature_type == 'd' ){
    //* compute normals
    computeNormal( input_cloud, cloud_normal );
  }	
  
  for(int r3=0; r3 < rotate_step_num; r3++){
    for(int r2=0; r2 < rotate_step_num; r2++){
      for(int r1=0; r1 < rotate_step_num; r1++){
	const double roll  = r3 * M_PI / (2*rotate_step_num);
	const double pan   = r2 * M_PI / (2*rotate_step_num);
	const double roll2 = r1 * M_PI / (2*rotate_step_num);
	
	//* voxelize
	if( feature_type == 'd' ){
	  rotatePoints( cloud_normal, cloud_normal_r, roll, pan, roll2 );
	  getVoxelGrid( grid_normal, cloud_normal_r, cloud_downsampled_normal, voxel_size );	      
	}
	else{
	  rotatePoints( input_cloud, input_cloud_r, roll, pan, roll2 );
	  //* voxelize
	  getVoxelGrid( grid, input_cloud_r, cloud_downsampled, voxel_size );
	}
	
	
	// repeat with changing offset values for subdivisions
	for( int ox = 0; ox < repeat_num_offset; ox++ ){
	  for( int oy = 0; oy < repeat_num_offset; oy++ ){
	    for( int oz = 0; oz < repeat_num_offset; oz++ ){
	      //* compute features
	      if( feature_type == 'd' ){
		computeGRSD( grid_normal, cloud_normal_r, cloud_downsampled_normal, grsd, voxel_size, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step );
		computeColorCHLAC( grid_normal, cloud_downsampled_normal, colorCHLAC, thR, thG, thB, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step );
	      }
	      else
		computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, thR, thG, thB, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step );
	      
	      const int hist_num = colorCHLAC.size();
	      
	      for( int h=0; h<hist_num; h++ ){
		if( feature_type == 'c' ) feature.push_back (colorCHLAC[ h ]);
		else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC[ h ] ) );
		
		std::vector<float> colorCHLAC_rotate;
		std::vector<float> colorCHLAC_rotate_pre = colorCHLAC[ h ];
		std::vector<float> colorCHLAC_rotate_pre2;
		
		for(int t=0;t<3;t++){
		  rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		  if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		  else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 1 - 3
		  colorCHLAC_rotate_pre = colorCHLAC_rotate;
		}
		
		rotateFeature90( colorCHLAC_rotate,colorCHLAC[ h ],R_MODE_3);
		if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 4
		colorCHLAC_rotate_pre  = colorCHLAC_rotate;
		colorCHLAC_rotate_pre2 = colorCHLAC_rotate;
		
		for(int t=0;t<3;t++){
		  rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		  if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		  else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 5 - 7
		  colorCHLAC_rotate_pre = colorCHLAC_rotate;
		}
		
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre2,R_MODE_3);
		if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 8
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
		colorCHLAC_rotate_pre2 = colorCHLAC_rotate;
		
		for(int t=0;t<3;t++){
		  rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		  if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		  else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 9 - 11
		  colorCHLAC_rotate_pre = colorCHLAC_rotate;
		}
		
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre2,R_MODE_3);
		if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 12
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
		//colorCHLAC_rotate_pre2 = colorCHLAC_rotate;
		
		for(int t=0;t<3;t++){
		  rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		  if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		  else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 13 - 15
		  colorCHLAC_rotate_pre = colorCHLAC_rotate;
		}
		
		rotateFeature90( colorCHLAC_rotate,colorCHLAC[ h ],R_MODE_1);
		if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 16
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
		
		for(int t=0;t<3;t++){
		  rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		  if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		  else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 17 - 19
		  colorCHLAC_rotate_pre = colorCHLAC_rotate;
		}
		
		rotateFeature90( colorCHLAC_rotate,colorCHLAC[ h ],R_MODE_4);
		if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 20
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
		
		for(int t=0;t<3;t++){
		  rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		  if( feature_type == 'c' ) feature.push_back (colorCHLAC_rotate);
		  else feature.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 21 - 23
		  colorCHLAC_rotate_pre = colorCHLAC_rotate;
		}
	      }
	    }
	  }
	}
	
	if(r2==0) break;
      }
    }
  }
}

//-------
//* main
int main( int argc, char** argv ){
  if( argc < 4 ){
    ROS_ERROR ("Need at least three parameters! Syntax is: %s {input_pointcloud_filename.pcd} {feature_initial(g, c, d, or r)} [options] {output_histogram_filename.pcd}\n", argv[0]);
    ROS_INFO ("                          -rotate rotate_step_num = e.g. 3 for 30 degrees rotation\n");
    ROS_INFO ("                          -subdiv N = subdivision size (e.g. 10 voxels)\n");
    return(-1);
  }

  // check feature_type
  const char feature_type = argv[2][0];
  if( (feature_type != 'c') && (feature_type != 'g') && (feature_type != 'd') && (feature_type != 'r') ){
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }

  // rotate step num
  if( parse_argument (argc, argv, "-rotate", rotate_step_num) > 0 ){
    if ( rotate_step_num < 1 ){
      print_error ("Invalid rotate_step_num (%d)!\n", rotate_step_num);
      return (-1);
    }
  }

  // color threshold
  FILE *fp = fopen( "color_threshold.txt", "r" );
  fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
  fclose(fp);

  //* voxel size (downsample_leaf)
  fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  // subdivision size
  if( parse_argument (argc, argv, "-subdiv", subdivision_size) > 0 ){
    if ( subdivision_size < 0 ){
      print_error ("Invalid subdivision size (%d)! \n", subdivision_size);
      return (-1);
    }
  }

  //* read
  pcl::PointCloud<PointXYZRGB> input_cloud;
  readPoints( argv[1], input_cloud );

  //* compute feature
  std::vector< std::vector<float> > feature;
  if( (feature_type == 'g') || (feature_type == 'r') )
    computeFeature( input_cloud, feature, feature_type );
  else
    computeFeature_with_rotate( input_cloud, feature, feature_type );

  //* write
  writeFeature( argv[ argc - 1 ], feature );

  return(0);
}
