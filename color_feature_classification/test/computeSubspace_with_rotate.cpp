#define QUIET

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>
#include <color_chlac/grsd_colorCHLAC_tools.h>
#include "color_feature_classification/libPCA.hpp"

using namespace pcl;
using namespace std;
using namespace terminal_tools;

// color threshold
int thR, thG, thB;
float voxel_size;

bool rotatePoints( const pcl::PointCloud<PointXYZRGB> &input_cloud, pcl::PointCloud<PointXYZRGB> &output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);  R1[1]=-sin(roll);  R1[2]=0;
  R1[3]=sin(roll);  R1[4]=cos(roll);   R1[5]=0;
  R1[6]=0;          R1[7]=0;           R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);   R2[1]=0;  R2[2]=sin(pan);
  R2[3]=0;          R2[4]=1;  R2[5]=0;
  R2[6]=-sin(pan);  R2[7]=0;  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2); R3[1]=-sin(roll2); R3[2]=0;
  R3[3]=sin(roll2); R3[4]=cos(roll2);  R3[5]=0;
  R3[6]=0;          R3[7]=0;           R3[8]=1;

  float x1, y1, z1, x2, y2, z2;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input_cloud.points[ i ].x + R1[1] * input_cloud.points[ i ].y + R1[2] * input_cloud.points[ i ].z;
    y1 = R1[3] * input_cloud.points[ i ].x + R1[4] * input_cloud.points[ i ].y + R1[5] * input_cloud.points[ i ].z;
    z1 = R1[6] * input_cloud.points[ i ].x + R1[7] * input_cloud.points[ i ].y + R1[8] * input_cloud.points[ i ].z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
	    
    output_cloud.points[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    output_cloud.points[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    output_cloud.points[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
  }

  return(1);
}

bool rotatePoints( const pcl::PointCloud<PointXYZRGBNormal> &input_cloud, pcl::PointCloud<PointXYZRGBNormal> &output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);  R1[1]=-sin(roll);  R1[2]=0;
  R1[3]=sin(roll);  R1[4]=cos(roll);   R1[5]=0;
  R1[6]=0;          R1[7]=0;           R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);   R2[1]=0;  R2[2]=sin(pan);
  R2[3]=0;          R2[4]=1;  R2[5]=0;
  R2[6]=-sin(pan);  R2[7]=0;  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2); R3[1]=-sin(roll2); R3[2]=0;
  R3[3]=sin(roll2); R3[4]=cos(roll2);  R3[5]=0;
  R3[6]=0;          R3[7]=0;           R3[8]=1;

  float x1, y1, z1, x2, y2, z2;
  float nx1, ny1, nz1, nx2, ny2, nz2;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input_cloud.points[ i ].x + R1[1] * input_cloud.points[ i ].y + R1[2] * input_cloud.points[ i ].z;
    y1 = R1[3] * input_cloud.points[ i ].x + R1[4] * input_cloud.points[ i ].y + R1[5] * input_cloud.points[ i ].z;
    z1 = R1[6] * input_cloud.points[ i ].x + R1[7] * input_cloud.points[ i ].y + R1[8] * input_cloud.points[ i ].z;
    nx1 = R1[0] * input_cloud.points[ i ].normal_x + R1[1] * input_cloud.points[ i ].normal_y + R1[2] * input_cloud.points[ i ].normal_z;
    ny1 = R1[3] * input_cloud.points[ i ].normal_x + R1[4] * input_cloud.points[ i ].normal_y + R1[5] * input_cloud.points[ i ].normal_z;
    nz1 = R1[6] * input_cloud.points[ i ].normal_x + R1[7] * input_cloud.points[ i ].normal_y + R1[8] * input_cloud.points[ i ].normal_z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
    nx2 = R2[0]*nx1 + R2[1]*ny1 + R2[2]*nz1;
    ny2 = R2[3]*nx1 + R2[4]*ny1 + R2[5]*nz1;
    nz2 = R2[6]*nx1 + R2[7]*ny1 + R2[8]*nz1;
	    
    output_cloud.points[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    output_cloud.points[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    output_cloud.points[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
    output_cloud.points[ i ].normal_x = R3[0]*nx2 + R3[1]*ny2 + R3[2]*nz2;
    output_cloud.points[ i ].normal_y = R3[3]*nx2 + R3[4]*ny2 + R3[5]*nz2;
    output_cloud.points[ i ].normal_z = R3[6]*nx2 + R3[7]*ny2 + R3[8]*nz2;
  }

  return(1);
}

void
computeFeatureModels ( const char feature_type, const int rotate_step_num, int argc, char **argv, const std::string &extension, 
		       std::vector< std::vector<float> > &models, const int subdivision_size)
{  
 // necessary if feature_type == 'd'
  pcl::PointCloud<PointXYZRGBNormal> cloud_normal;
  pcl::PointCloud<PointXYZRGBNormal> cloud_normal_r;

 // necessary if feature_type == 'c'
  pcl::PointCloud<PointXYZRGB> cloud_object_cluster;
  pcl::PointCloud<PointXYZRGB> cloud_object_cluster_r; // rotate

  for (int i = 1; i < argc; i++){
    string fname = string (argv[i]);
    // Needs to have the right size
    if (fname.size () <= extension.size ())
      continue;
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0){

      readPoints( argv[i], cloud_object_cluster );

      std::vector< std::vector<float> > grsd;
      std::vector< std::vector<float> > colorCHLAC;
      if( feature_type == 'd' ){
	//* compute normals
	computeNormal( cloud_object_cluster, cloud_normal );
      }	
      
      for(int r3=0; r3 < rotate_step_num; r3++){
	for(int r2=0; r2 < rotate_step_num; r2++){
	  for(int r1=0; r1 < rotate_step_num; r1++){
	    const double roll  = r3 * M_PI / (2*rotate_step_num);
	    const double pan   = r2 * M_PI / (2*rotate_step_num);
	    const double roll2 = r1 * M_PI / (2*rotate_step_num);
	    
	    if( feature_type == 'd' ){
	      rotatePoints( cloud_normal, cloud_normal_r, roll, pan, roll2 );

	      //* voxelize
	      pcl::VoxelGrid<PointXYZRGBNormal> grid;
	      pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled;
	      getVoxelGrid( grid, cloud_normal_r, cloud_downsampled, voxel_size );
	      
	      //* compute - GRSD -
	      computeGRSD( grid, cloud_normal_r, cloud_downsampled, grsd, voxel_size, subdivision_size );

	      //* compute - ColorCHLAC -
	      computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, thR, thG, thB, subdivision_size );
	    }
	    else{
	      rotatePoints( cloud_object_cluster, cloud_object_cluster_r, roll, pan, roll2 );

	      //* voxelize
	      pcl::VoxelGrid<PointXYZRGB> grid;
	      pcl::PointCloud<PointXYZRGB> cloud_downsampled;
	      getVoxelGrid( grid, cloud_object_cluster_r, cloud_downsampled, voxel_size );

	      //* compute - ColorCHLAC -
	      computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, thR, thG, thB, subdivision_size );
	    }
	    const int hist_num = colorCHLAC.size();

	    for( int h=0; h<hist_num; h++ ){
	      if( feature_type == 'c' ) models.push_back (colorCHLAC[ h ]);
	      else models.push_back ( conc_vector( grsd[ h ], colorCHLAC[ h ] ) );
	      
	      std::vector<float> colorCHLAC_rotate;
	      std::vector<float> colorCHLAC_rotate_pre = colorCHLAC[ h ];
	      std::vector<float> colorCHLAC_rotate_pre2;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
		else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 1 - 3
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      }
	      
	      rotateFeature90( colorCHLAC_rotate,colorCHLAC[ h ],R_MODE_3);
	      if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
	      else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 4
	      colorCHLAC_rotate_pre  = colorCHLAC_rotate;
	      colorCHLAC_rotate_pre2 = colorCHLAC_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
		else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 5 - 7
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      }
	      
	      rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre2,R_MODE_3);
	      if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
	      else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 8
	      colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      colorCHLAC_rotate_pre2 = colorCHLAC_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
		else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 9 - 11
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      }
	      
	      rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre2,R_MODE_3);
	      if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
	      else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 12
	      colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      //colorCHLAC_rotate_pre2 = colorCHLAC_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
		else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 13 - 15
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      }
	      
	      rotateFeature90( colorCHLAC_rotate,colorCHLAC[ h ],R_MODE_1);
	      if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
	      else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 16
	      colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
		else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 17 - 19
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      }
	      
	      rotateFeature90( colorCHLAC_rotate,colorCHLAC[ h ],R_MODE_4);
	      if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
	      else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 20
	      colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( colorCHLAC_rotate,colorCHLAC_rotate_pre,R_MODE_2);
		if( feature_type == 'c' ) models.push_back (colorCHLAC_rotate);
		else models.push_back ( conc_vector( grsd[ h ], colorCHLAC_rotate ) ); // 21 - 23
		colorCHLAC_rotate_pre = colorCHLAC_rotate;
	      }
	    }

	    if(r2==0) break;
	  }
	}
      }
    }
  }
}

void compressFeature( string filename, std::vector< std::vector<float> > &models, const int dim, bool ascii ){
  PCA pca;
  pca.read( filename.c_str(), ascii );
  MatrixXf tmpMat = pca.Axis();
  MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim);
  const int num = (int)models.size();
  for( int i=0; i<num; i++ ){
    Map<VectorXf> vec( &(models[i][0]), models[i].size() );
    //vec = tmpMat2.transpose() * vec;
    VectorXf tmpvec = tmpMat2.transpose() * vec;
    models[i].resize( dim );
    for( int t=0; t<dim; t++ )
      models[i][t] = tmpvec[t];
  }
}

void computeSubspace( std::vector< std::vector<float> > models, const char* filename, bool ascii ){
  cout << models[0].size() << endl;
  PCA pca( false );
  const int num = (int)models.size();
  for( int i=0; i<num; i++ )
    pca.addData( models[ i ] );
  pca.solve();
  pca.write( filename, ascii );
}

int main( int argc, char** argv ){
  if( argc < 4 ){
    ROS_ERROR ("Need at least three parameters! Syntax is: %s {feature_initial(c or d)} [model_directory] [options] [output_pca_name]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = size of compressed feature vectors\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    ROS_INFO ("                          -rotate rotate_step_num = e.g. 3 for 30 degrees rotation\n");
    ROS_INFO ("                          -subdiv N = subdivision size (e.g. 10 voxels)\n");
    return(-1);
  }

  // check feature_type
  const char feature_type = argv[1][0];
  if( (feature_type != 'c') && (feature_type != 'd') ){
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }

  // rotate step num
  int rotate_step_num = 1;
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

  // voxel size
  fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  // subdivision size
  int subdivision_size = 0;
  if( parse_argument (argc, argv, "-subdiv", subdivision_size) > 0 ){
    if ( subdivision_size < 0 ){
      print_error ("Invalid subdivision size (%d)! \n", subdivision_size);
      return (-1);
    }
  }

  // compute features
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  std::vector< std::vector<float> > models;
  computeFeatureModels (feature_type, rotate_step_num, argc, argv, extension, models, subdivision_size);

  // compress the dimension of the vector (if needed)
  int dim;
  if( parse_argument (argc, argv, "-dim", dim) > 0 ){
    if ((dim < 0)||(dim >= (int)models[0].size())){
      print_error ("Invalid dimension (%d)!\n", dim);
      return (-1);
    }
    string filename;
    parse_argument (argc, argv, "-comp", filename);
    compressFeature( filename, models, dim, false );
  }

  // compute subspace
  computeSubspace( models, argv[ argc - 1 ], false );

  return(0);
}
