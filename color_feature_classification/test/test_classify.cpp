#define QUIET

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
#include <color_chlac/grsd_colorCHLAC_tools.h>
#include "color_feature_classification/libPCA.hpp"
#include <sys/stat.h>
#include <dirent.h>

using namespace pcl;
using namespace std;
using namespace terminal_tools;

int classify_by_kNN( std::vector<float> feature, const char feature_type, int argc, char** argv, int k = 8, int metric = 7, double thresh = DBL_MAX ){
  ROS_ERROR ("classify_by_kNN() cannot be used now.\n");
  return 1;
}

int classify_by_subspace( std::vector<float> feature, const char feature_type, const int dim_subspace ){
  char dirname[ 20 ];
  char filename[ 300 ];
  Map<VectorXf> vec( &(feature[0]), feature.size() );

  sprintf( dirname, "pca_result_" );
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
  float sum, dot;
  for( int i=0; i<obj_class_num;i++ ){
    sprintf( filename, "%s/%03d", dirname, i );
    pca.read( filename, false );
//     cout << pca.Axis() << endl;
//     continue;
//     for( int t=0; t<981;t++ )
//       printf("%f ",vec[t]);
    // VectorXf var = pca.Variance();
    // cout << var << endl;
    // exit(1);
    MatrixXf tmpMat = pca.Axis();
    MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim_subspace);
    VectorXf tmpVec;
    if( pca.mean_flg ){
      //cout << pca.Mean() << endl;
      VectorXf tmpVec2 = vec-pca.Mean();
      tmpVec = tmpMat2.transpose() * tmpVec2;
      sum = tmpVec2.dot( tmpVec2 );
      dot = tmpVec.dot( tmpVec ) / sum;
    }
    else{
      tmpVec = tmpMat2.transpose() * vec;
      dot = tmpVec.dot( tmpVec );
    }
    if( dot > dot_max ){
      dot_max = dot;
      class_num = i;
    }
  }
  cout << class_num << " " << dot_max << endl;
  return class_num;
}

void compressFeature( string filename, std::vector<float> &feature, const int dim, bool ascii ){
  PCA pca;
  pca.read( filename.c_str(), ascii );
  MatrixXf tmpMat = pca.Axis();
  MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim);
  Map<VectorXf> vec( &(feature[0]), feature.size() );
  //vec = tmpMat2.transpose() * vec;
  VectorXf tmpvec = tmpMat2.transpose() * vec;
  feature.resize( dim );
  for( int t=0; t<dim; t++ )
    feature[t] = tmpvec[t];
}

//--------------------------------------------------------------------------------------------
int main( int argc, char** argv ){
  //ros::init( argc, argv, "test" );
  if( argc < 4 ){
    ROS_ERROR ("Need three parameters! Syntax is: %s {input_pointcloud_filename.pcd} {feature_initial(g, c, d, or r)} {classifier_initial(k or s)} [options]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = dimension of compressed feature vectors\n");
    ROS_INFO ("    where [options] are:  -sub R = dimension of subspace\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    return(-1);
  }
  const char feature_type = argv[2][0];
  const char classifier_type = argv[3][0];
  std::vector<float> feature;

  // color threshold
  int thR, thG, thB;
  FILE *fp = fopen( "color_threshold.txt", "r" );
  fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
  fclose(fp);

  // voxel size
  float voxel_size;
  fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  // read points
  pcl::PointCloud<PointXYZRGB> cloud_object_cluster;
  readPoints( argv[1], cloud_object_cluster );

  //------------
  //* features
  if( feature_type == 'c' ){
    //* voxelize
    pcl::VoxelGrid<PointXYZRGB> grid;
    pcl::PointCloud<PointXYZRGB> cloud_downsampled;
    getVoxelGrid( grid, cloud_object_cluster, cloud_downsampled, voxel_size );
    
    //* compute - ColorCHLAC -
    computeColorCHLAC( grid, cloud_downsampled, feature, thR, thG, thB );
  }
  else if( ( feature_type == 'g' ) || ( feature_type == 'd' ) || ( feature_type == 'r' ) ){
    //* compute normals
    pcl::PointCloud<PointXYZRGBNormal> cloud;
    computeNormal( cloud_object_cluster, cloud );
    
    //* voxelize
    pcl::VoxelGrid<PointXYZRGBNormal> grid;
    pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled;
    getVoxelGrid( grid, cloud, cloud_downsampled, voxel_size );

    if( feature_type == 'g' )
      computeGRSD( grid, cloud, cloud_downsampled, feature, voxel_size );
    else{    // TODO : implement 2 versions - d and r -
      //* compute - GRSD -
      std::vector<float> grsd;
      computeGRSD( grid, cloud, cloud_downsampled, grsd, voxel_size );

      //* compute - ColorCHLAC -
      std::vector< float > colorCHLAC;
      if( feature_type == 'd' )
	computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, thR, thG, thB );
      else
	computeColorCHLAC_RI( grid, cloud_downsampled, colorCHLAC, thR, thG, thB );
      feature = conc_vector( grsd, colorCHLAC );
    }
  }
  else{
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }
  //ROS_INFO ("Input feature vector extracted.");

  //--------------------------------------------------
  //* Compress the dimension of the vector (if needed)
  int dim = -1;
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
    classify_by_subspace( feature, feature_type, dim_subspace );
  }
  else{
    ROS_ERROR ("Unknown classifier type.\n");
    return(-1);
  }

  return(0);
}

///////////////////////////////////////////////////////////////////////////////////////////////
//* This is old version (not used now.)
//* To activate this function, this package needs the dependency of "vfh_cluster_classifier"
//*   and also some proper changes.

// int classify_by_kNN( vfh_model feature, const char feature_type, int argc, char** argv, int k = 8, int metric = 7, double thresh = DBL_MAX ){
//   string kdtree_idx_file_name, training_data_h5_file_name, training_data_list_file_name;

//   if( feature_type == 'v' ){
//     kdtree_idx_file_name = "kdtree_vfh.idx";
//     training_data_h5_file_name = "training_data_vfh.h5";
//     training_data_list_file_name = "training_data_vfh.list";
//   }
//   else if( feature_type == 'c' ){
//     kdtree_idx_file_name = "kdtree_colorCHLAC.idx";
//     training_data_h5_file_name = "training_data_colorCHLAC.h5";
//     training_data_list_file_name = "training_data_colorCHLAC.list";
//   }
//   else{
//     ROS_ERROR ("Unknown feature type.\n");
//     return(-1);
//   }

//   vector<vfh_model> models;
//   flann::Matrix<int> k_indices;
//   flann::Matrix<float> k_distances;
//   flann::Matrix<float> data;

//   // Check if the data has already been saved to disk
//   if (!boost::filesystem::exists (training_data_h5_file_name) || !boost::filesystem::exists (training_data_list_file_name))
//   {
//     print_error ("Could not find training data models files %s and %s!\n", training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
//     return (-1);
//   }
//   else
//   {
//     loadFileList (models, training_data_list_file_name);
//     flann::load_from_file (data, training_data_h5_file_name, "training_data");
//     print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
//   }

//   // Check if the tree index has already been saved to disk
//   if (!boost::filesystem::exists (kdtree_idx_file_name))
//   {
//     print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
//     return (-1);
//   }
//   else
//   {
//     flann::Index<flann::ChiSquareDistance<float> > index (data, flann::SavedIndexParams (kdtree_idx_file_name));
//     index.buildIndex ();
//     nearestKSearch (index, feature, k, k_indices, k_distances);
//   }

//   // Output the results on screen
//   print_highlight ("The closest %d neighbors for %s are:\n", k, feature.first.c_str() );
//   for (int i = 0; i < k; ++i)
//     print_info ("    %d - %s (%d) with a distance of: %f\n", i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
  
//   // // Visualize the results
//   // if( feature_type == 'c' ){
//   //   for( int m = 0; m < (int)models.size(); m++ ){
//   //     int len = models[ m ].first.length();
//   //     models[ m ].first[ len - 14 ] = 'v';
//   //     models[ m ].first[ len - 13 ] = 'f';
//   //     models[ m ].first[ len - 12 ] = 'h';
//   //     models[ m ].first[ len - 11 ] = '.';
//   //     models[ m ].first[ len - 10 ] = 'p';
//   //     models[ m ].first[ len - 9 ] = 'c';
//   //     models[ m ].first[ len - 8 ] = 'd';
//   //     models[ m ].first[ len - 7 ] = '\0';
//   //   }
//   // }
//   // visualizeNearestModels( argc, argv, k, thresh, models, k_indices, k_distances );

//   return(1);
// }
