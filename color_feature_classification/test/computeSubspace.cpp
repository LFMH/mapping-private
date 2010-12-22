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

template <typename T>
bool rotatePoints( const T& input_cloud, T& output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);
  R1[1]=-sin(roll);
  R1[2]=0;
  R1[3]=sin(roll);
  R1[4]=cos(roll);
  R1[5]=0;
  R1[6]=0;
  R1[7]=0;
  R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);
  R2[1]=0;
  R2[2]=sin(pan);
  R2[3]=0;
  R2[4]=1;
  R2[5]=0;
  R2[6]=-sin(pan);
  R2[7]=0;
  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2);
  R3[1]=-sin(roll2);
  R3[2]=0;
  R3[3]=sin(roll2);
  R3[4]=cos(roll2);
  R3[5]=0;
  R3[6]=0;
  R3[7]=0;
  R3[8]=1;

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

bool computeColorCHLAC( pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster, std::vector<float> &m ){
  // ---[ Create the voxel grid
  pcl::PointCloud<PointXYZRGB> cloud_downsampled;
  pcl::VoxelGrid<PointXYZRGB> grid_;
  float voxel_size;
  FILE *fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);
  grid_.setLeafSize (voxel_size, voxel_size, voxel_size);

  grid_.setInputCloud (boost::make_shared<const pcl::PointCloud<PointXYZRGB> > (cloud_object_cluster));
  grid_.setSaveLeafLayout(true);
  grid_.filter (cloud_downsampled);

  // color threshold
  int thR, thG, thB;
  fp = fopen( "color_threshold.txt", "r" );
  fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
  fclose(fp);

  // ---[ Compute ColorCHLAC
  computeColorCHLAC( grid_, cloud_downsampled, m, thR, thG, thB );

  return 1;
}

void
computeFeatureModels ( const char classifier_type, const int rotate_step_num, int argc, char **argv, const std::string &extension, 
		       std::vector< std::vector<float> > &models)
{  
  for (int i = 1; i < argc; i++){
    string fname = string (argv[i]);
    // Needs to have the right size
    if (fname.size () <= extension.size ())
      continue;
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0){

      std::vector< float > m;
      if( classifier_type == 'c' ){
	pcl::PointCloud<PointXYZRGB> cloud_object_cluster;
	pcl::PointCloud<PointXYZRGB> cloud_object_cluster_r; // rotate
	readPoints( argv[i], cloud_object_cluster );
	for(int r3=0; r3 < rotate_step_num; r3++){
	  for(int r2=0; r2 < rotate_step_num; r2++){
	    for(int r1=0; r1 < rotate_step_num; r1++){
	      const double roll  = r3 * M_PI / (2*rotate_step_num);
	      const double pan   = r2 * M_PI / (2*rotate_step_num);
	      const double roll2 = r1 * M_PI / (2*rotate_step_num);
	      rotatePoints( cloud_object_cluster, cloud_object_cluster_r, roll, pan, roll2 );
	      computeColorCHLAC( cloud_object_cluster_r, m );
	      models.push_back (m);

	      std::vector<float> m_rotate;
	      std::vector<float> m_rotate_pre = m;
	      std::vector<float> m_rotate_pre2;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( m_rotate,m_rotate_pre,R_MODE_2);
		models.push_back ( m_rotate ); // 1 - 3
		m_rotate_pre = m_rotate;
	      }
	      
	      rotateFeature90( m_rotate,m,R_MODE_3);
	      models.push_back ( m_rotate ); // 4
	      m_rotate_pre  = m_rotate;
	      m_rotate_pre2 = m_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( m_rotate,m_rotate_pre,R_MODE_2);
		models.push_back ( m_rotate ); // 5 - 7
		m_rotate_pre = m_rotate;
	      }
	      
	      rotateFeature90( m_rotate,m_rotate_pre2,R_MODE_3);
	      models.push_back ( m_rotate ); // 8
	      m_rotate_pre = m_rotate;
	      m_rotate_pre2 = m_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( m_rotate,m_rotate_pre,R_MODE_2);
		models.push_back ( m_rotate ); // 9 - 11
		m_rotate_pre = m_rotate;
	      }
	      
	      rotateFeature90( m_rotate,m_rotate_pre2,R_MODE_3);
	      models.push_back ( m_rotate ); // 12
	      m_rotate_pre = m_rotate;
	      //m_rotate_pre2 = m_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( m_rotate,m_rotate_pre,R_MODE_2);
		models.push_back ( m_rotate ); // 13 - 15
		m_rotate_pre = m_rotate;
	      }
	      
	      rotateFeature90( m_rotate,m,R_MODE_1);
	      models.push_back ( m_rotate ); // 16
	      m_rotate_pre = m_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( m_rotate,m_rotate_pre,R_MODE_2);
		models.push_back ( m_rotate ); // 17 - 19
		m_rotate_pre = m_rotate;
	      }
	      
	      rotateFeature90( m_rotate,m,R_MODE_4);
	      models.push_back ( m_rotate ); // 20
	      m_rotate_pre = m_rotate;
	      
	      for(int t=0;t<3;t++){
		rotateFeature90( m_rotate,m_rotate_pre,R_MODE_2);
		models.push_back ( m_rotate ); // 21 - 23
		m_rotate_pre = m_rotate;
	      }

	      if(r2==0) break;
	    }
	  }
	}
      }
      else{
	ROS_ERROR ("Unknown feature type.\n");
	return;
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
    ROS_ERROR ("Need at least three parameters! Syntax is: %s {feature_initial(c or n)} [model_directory] [options] [output_pca_name]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = size of compressed feature vectors\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    ROS_INFO ("                          -rotate rotate_step_num = e.g. 3 for 30 degrees rotation\n");
    return(-1);
  }
  const char classifier_type = argv[1][0];
  int rotate_step_num = 1;
  if( parse_argument (argc, argv, "-rotate", rotate_step_num) > 0 ){
    if ( rotate_step_num < 1 ){
      print_error ("Invalid rotate_step_num (%d)!\n", rotate_step_num);
      return (-1);
    }
  }

  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  std::vector< std::vector<float> > models;
  computeFeatureModels (classifier_type, rotate_step_num, argc, argv, extension, models);

  // Compress the dimension of the vector (if needed)
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

  computeSubspace( models, argv[ argc - 1 ], false );

  return(0);
}
