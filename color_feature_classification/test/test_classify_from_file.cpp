#define QUIET
//#define CALC_TIME
//#define SHOW_SIMILARITY

#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/features/feature.h>
//#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>
#include "color_voxel_recognition/pca.h"
#include <sys/stat.h>
#include <dirent.h>
#include "FILE_MODE"

using namespace pcl;
using namespace std;
using namespace terminal_tools;

//const float lower = 0;//-1;
const float upper = 1;

//-----------
//* time
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

int MyCmp(const void* a, const void* b){
  if(*(float *)a < *(float *)b) return 1;
  else if(*(float *)a == *(float *)b) return 0;
  else                            return -1;
}

// void scaling( const int index, std::vector<float> &feature, const std::vector<float> feature_min, const std::vector<float> feature_max ) {
//   if( feature_min[ index ] == feature_max[ index ] ) feature[ index ] = 0;
//   else if( feature[ index ] == feature_min[ index ] ) feature[ index ] = lower;
//   else if( feature[ index ] == feature_max[ index ] ) feature[ index ] = upper;
//   else feature[ index ] = lower + (upper-lower) * ( feature[ index ] - feature_min[ index ] ) / ( feature_max[ index ] - feature_min[ index ] );
// }
void scaling( const int index, std::vector<float> &feature, const std::vector<float> feature_max ) {
  if( feature_max[ index ] == 0 ) feature[ index ] = 0;
  else if( feature[ index ] == feature_max[ index ] ) feature[ index ] = upper;
  else feature[ index ] = upper * feature[ index ] / feature_max[ index ];
}

int classify_by_kNN( std::vector<float> feature, const char feature_type, int argc, char** argv, int k = 8, int metric = 7, double thresh = DBL_MAX ){
  ROS_ERROR ("classify_by_kNN() cannot be used now.\n");
  return 1;
}

int classify_by_subspace( std::vector<float> feature, const char feature_type, const int dim_subspace, const char* config_txt_path ){
  char dirname[ 20 ];
  char filename[ 300 ];
  Eigen::Map<Eigen::VectorXf> vec( &(feature[0]), feature.size() );

  sprintf( dirname, "%s/pca_result_", config_txt_path );
  dirname[ 11 ] = feature_type;
  dirname[ 12 ] = '\0';

  // count obj class num
  int obj_class_num = 0;
  struct dirent *entry;
  DIR *dp = opendir( "data" );
  while( ( entry = readdir( dp ) ) != NULL ){
    string tmpname = string (entry->d_name);
    if( tmpname.compare (0, 3, "obj") == 0 )  obj_class_num ++;
  }
  if( obj_class_num == 0 ){
    ROS_ERROR ("Please name \"objXXX\". ( obj_class_num == 0 )");
    exit(1);
  }
  //cout << obj_class_num << endl;

  // calc similarity
  PCA pca;
  int class_num = -1;
  float dot_max = 0;
  float dot;
  float *dots = new float[ obj_class_num ];
  float sum = vec.dot( vec );
  double t1, t2, t_all = 0;
  for( int i=0; i<obj_class_num;i++ ){
    sprintf( filename, "%s/%03d", dirname, i );
    pca.read( filename, false );
//     cout << pca.getAxis() << endl;
//     continue;
//     for( int t=0; t<981;t++ )
//       printf("%f ",vec[t]);
    // Eigen::VectorXf var = pca.getVariance();
    // cout << var << endl;
    // exit(1);
    Eigen::MatrixXf tmpMat = pca.getAxis();
    Eigen::VectorXf variance = pca.getVariance();
    Eigen::MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim_subspace);
    Eigen::VectorXf tmpVec;
    t1 = my_clock();
    if( pca.mean_flg ){
      //cout << pca.getMean() << endl;
      Eigen::VectorXf tmpVec2 = vec-pca.getMean();
      tmpVec = tmpMat2.transpose() * tmpVec2;
      sum = tmpVec2.dot( tmpVec2 );
    }
    else
      tmpVec = tmpMat2.transpose() * vec;

    if( MULTIPLE_SIMILARITY )
      for( int j=1; j<dim_subspace; j++ )
	tmpVec( j ) *= sqrt( variance( j ) ) / sqrt( variance( 0 ) );

    dot = tmpVec.dot( tmpVec ) / sum;

#ifdef SHOW_SIMILARITY
    dots[ i ] = dot;
#endif
    if( dot > dot_max ){
      dot_max = dot;
      class_num = i;
    }
    t2 = my_clock();
    t_all += t2 - t1;
  }
#ifdef SHOW_SIMILARITY
  float *dots_copy = new float[ obj_class_num ];
  for(int i=0;i<obj_class_num;i++)
    dots_copy[i] = dots[i];
  qsort(dots_copy, obj_class_num, sizeof(float), MyCmp);
  for( int i=0; i<obj_class_num;i++ )
    for( int j=0; j<obj_class_num;j++ )
      if(dots[j]==dots_copy[i])
	cout << j << " " << dots[ j ] << endl;
  
  
#else
#ifdef CALC_TIME
  cout << t_all * 1000 << endl;
#else
  cout << class_num << " " << dot_max << endl;
#endif
#endif
  return class_num;
}

void compressFeature( string filename, std::vector<float> &feature, const int dim, bool ascii ){
  PCA pca;
  pca.read( filename.c_str(), ascii );
  Eigen::VectorXf variance = pca.getVariance();
  Eigen::MatrixXf tmpMat = pca.getAxis();
  Eigen::MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim);
  Eigen::Map<Eigen::VectorXf> vec( &(feature[0]), feature.size() );
  //vec = tmpMat2.transpose() * vec;
  Eigen::VectorXf tmpvec = tmpMat2.transpose() * vec;
  feature.resize( dim );
  if( WHITENING ){
    for( int t=0; t<dim; t++ )
      feature[t] = tmpvec[t] / sqrt( variance( t ) );
  }
  else{
    for( int t=0; t<dim; t++ )
	feature[t] = tmpvec[t];
  }
}

//--------------------------------------------------------------------------------------------
int main( int argc, char** argv ){
  //ros::init( argc, argv, "test" );
  if( argc < 5 ){
    ROS_ERROR ("Need three parameters! Syntax is: %s {input_pointcloud_filename.pcd} {feature_initial(g, c, d, or r)} {classifier_initial(k or s)} [options] {config_txt_path}\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = dimension of compressed feature vectors\n");
    ROS_INFO ("    where [options] are:  -sub R = dimension of subspace\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    ROS_INFO ("                          -norm filename = name of bin_normalize file\n");
    return(-1);
  }
  const char feature_type = argv[2][0];
  const char classifier_type = argv[3][0];
  std::vector<float> feature;
  std::vector<float> feature_max;
  bool is_normalize = false;

  // color threshold
  int thR, thG, thB;
  FILE *fp;
  if( feature_type != 'g' ){
    char color_threshold_file[1024];
    sprintf(color_threshold_file, "%s/color_threshold.txt", argv[argc-1]);
    fp = fopen( color_threshold_file, "r" );
    fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
    fclose(fp);
  }

  // voxel size
  float voxel_size;
  char voxel_size_file[1024];
  sprintf(voxel_size_file, "%s/voxel_size.txt", argv[argc-1]);
  fp = fopen( voxel_size_file, "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  // bin normalization parameters
  string filename;
  if( parse_argument (argc, argv, "-norm", filename) > 0 ){
    is_normalize = true;
    fp = fopen( filename.c_str(), "r" );
    float val;
    while( fscanf( fp, "%f\n", &val ) != EOF )
      feature_max.push_back( val );
    fclose( fp );
  }

  //------------
  //* features
  char line[ 100 ];
  string tmpname;
  int dim, sample_num;
  fp = fopen( argv[1], "r" );
  while( 1 ){
    fgets(line,sizeof(line),fp);
    tmpname = string (line);
    if( tmpname.compare (0, 5, "COUNT") == 0 )
      sscanf( line, "COUNT %d", &dim );
    else if( tmpname.compare (0, 6, "POINTS") == 0 )
      sscanf( line, "POINTS %d", &sample_num );
    else if( tmpname.compare (0, 4, "DATA") == 0 )
      break;
  }
  if( sample_num != 1 ){
    ROS_ERROR ("Input feature file is not global.\n");
    return(-1);
  }
  feature.resize( dim );
  if( is_normalize ){
    for(int t=0;t<dim;t++){
      fscanf(fp,"%f ",&(feature[ t ]) );
      scaling( t, feature, feature_max );
    }
  }
  else
    for(int t=0;t<dim;t++)
      fscanf(fp,"%f ",&(feature[ t ]) ); 
  fclose(fp);
  
  //--------------------------------------------------
  //* Compress the dimension of the vector (if needed)
  dim = -1;
  if( parse_argument (argc, argv, "-dim", dim) > 0 ){
    if ((dim < 0)||(dim >= (int)feature.size())){
      print_error ("Invalid dimension (%d)!\n", dim);
      return (-1);
    }
    string filename;
    parse_argument (argc, argv, "-comp", filename);
    compressFeature( filename, feature, dim, false );
  }

  //-------------------------
  //* dimension of subspace
  int dim_subspace = -1;
  if( parse_argument (argc, argv, "-sub", dim_subspace) > 0 ){
    if ( (dim_subspace < 0) || ((dim_subspace >= (int)dim) && (dim>0)) ){
      print_error ("Invalid subspace dimension (%d)! : larger than dim %d\n", dim_subspace, dim);
      return (-1);
    }
  }
  else{
    print_error ("Please input subspace-dimension. \"-sub R\"\n");
    return (-1);
  }
  
  //--------------
  //* classifier
  if( classifier_type == 'k' ){
    //* classifier - kNN -
    classify_by_kNN( feature, feature_type, argc, argv );
  }
  else if( classifier_type == 's' ){
    //* classifier - SubspaceMethod -
    classify_by_subspace( feature, feature_type, dim_subspace, argv[ argc-1 ] );
  }
  else{
    ROS_ERROR ("Unknown classifier type.\n");
    return(-1);
  }

  return(0);
}
