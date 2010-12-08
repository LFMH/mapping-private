#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/common/eigen.h>
#include <pcl/features/feature.h>
#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <vfh_cluster_classifier/vfh_nearest_neighbors.h>
#include <color_chlac/ColorCHLAC.hpp>
#include <color_chlac/ColorVoxel.hpp>
#include "color_feature_classification/libPCA.hpp"

using namespace pcl;

// bool vfhExtraction(color_feature_classification::GetHoge::Request &req, color_feature_classification::GetHoge::Response &res){
//   return(true);
// }

template <typename T>
bool readPoints( const char *name, T& cloud_object_cluster ){
  if (pcl::io::loadPCDFile (name, cloud_object_cluster) == -1){
    ROS_ERROR ("Couldn't read file %s",name);
    return (-1);
  }
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud_object_cluster.width * cloud_object_cluster.height), name, pcl::getFieldsList (cloud_object_cluster).c_str ());
  return(1);
}

//--------------------------------------------------------------------------------------------
// function1
bool getVFHsignature(pcl::PointCloud<pcl::PointXYZ> cloud_object_cluster, std::vector<float> &vfh_vector){
  // compute normals
  pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal> >();
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n3d;
  n3d.setKSearch (30); // have to use the same parameters as during training
  pcl::KdTree<pcl::PointXYZ>::Ptr tree = boost::make_shared<pcl::KdTreeANN<pcl::PointXYZ> > ();
  n3d.setSearchMethod (tree);
  n3d.setInputCloud(cloud_object_cluster.makeShared ());
  n3d.compute(*normals);
  
  // compute vfh
  pcl::VFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::VFHSignature308> vfh;
  vfh.setSearchMethod (tree);
  vfh.setInputCloud(cloud_object_cluster.makeShared ());
  vfh.setInputNormals(normals);
  pcl::PointCloud<pcl::VFHSignature308> vfh_signature;
  vfh.compute(vfh_signature);
  //vfh_vector = vfh_signature.points[0].histogram;
  if( vfh_vector.size() != 308 ) vfh_vector.resize( 308 );
  for( int i=0; i<308; i++ )
    vfh_vector[i] = vfh_signature.points[0].histogram[i];

  return(1);
}

//--------------------------------------------------------------------------------------------
// function2
bool getColorCHLAC(pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster, std::vector<float> &colorCHLAC ){
  // compute voxel
  ColorVoxel voxel;
  float voxel_size;
  FILE *fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);
  voxel.setVoxelSize( voxel_size );
  voxel.points2voxel( cloud_object_cluster, SIMPLE_REVERSE );
  ColorVoxel voxel_bin;
  voxel_bin = voxel;
  int thR, thG, thB;
  fp = fopen( "color_threshold.txt", "r" );
  fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
  fclose(fp);
  voxel_bin.binarize( thR, thG, thB );

  // compute colorCHLAC
  ColorCHLAC::extractColorCHLAC( colorCHLAC, voxel );
  colorCHLAC.resize( DIM_COLOR_1_3+DIM_COLOR_BIN_1_3 );
  std::vector<float> tmp;
  ColorCHLAC::extractColorCHLAC_bin( tmp, voxel_bin );
  for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
    colorCHLAC[ t+DIM_COLOR_1_3 ] = tmp[ t ];

  return 1;
}

//--------------------------------------------------------------------------------------------
// function3

int classify_by_kNN( vfh_model feature, const char feature_type, int argc, char** argv, int k = 8, int metric = 7, double thresh = DBL_MAX ){
  string kdtree_idx_file_name, training_data_h5_file_name, training_data_list_file_name;

  if( feature_type == 'v' ){
    kdtree_idx_file_name = "kdtree_vfh.idx";
    training_data_h5_file_name = "training_data_vfh.h5";
    training_data_list_file_name = "training_data_vfh.list";
  }
  else if( feature_type == 'c' ){
    kdtree_idx_file_name = "kdtree_colorCHLAC.idx";
    training_data_h5_file_name = "training_data_colorCHLAC.h5";
    training_data_list_file_name = "training_data_colorCHLAC.list";
  }
  else{
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }

  vector<vfh_model> models;
  flann::Matrix<int> k_indices;
  flann::Matrix<float> k_distances;
  flann::Matrix<float> data;

  // Check if the data has already been saved to disk
  if (!boost::filesystem::exists (training_data_h5_file_name) || !boost::filesystem::exists (training_data_list_file_name))
  {
    print_error ("Could not find training data models files %s and %s!\n", training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    return (-1);
  }
  else
  {
    loadFileList (models, training_data_list_file_name);
    flann::load_from_file (data, training_data_h5_file_name, "training_data");
    print_highlight ("Training data found. Loaded %d VFH models from %s/%s.\n", (int)data.rows, training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
  }

  // Check if the tree index has already been saved to disk
  if (!boost::filesystem::exists (kdtree_idx_file_name))
  {
    print_error ("Could not find kd-tree index in file %s!", kdtree_idx_file_name.c_str ());
    return (-1);
  }
  else
  {
    flann::Index< flann::L2<float> > index (data, flann::SavedIndexParams (kdtree_idx_file_name));
    index.buildIndex ();
    nearestKSearch (index, feature, k, k_indices, k_distances);
  }

  // Output the results on screen
  print_highlight ("The closest %d neighbors for %s are:\n", k, feature.first.c_str() );
  for (int i = 0; i < k; ++i)
    print_info ("    %d - %s (%d) with a distance of: %f\n", i, models.at (k_indices[0][i]).first.c_str (), k_indices[0][i], k_distances[0][i]);
  
  // Visualize the results
  if( feature_type == 'c' ){
    for( int m = 0; m < (int)models.size(); m++ ){
      int len = models[ m ].first.length();
      models[ m ].first[ len - 14 ] = 'v';
      models[ m ].first[ len - 13 ] = 'f';
      models[ m ].first[ len - 12 ] = 'h';
      models[ m ].first[ len - 11 ] = '.';
      models[ m ].first[ len - 10 ] = 'p';
      models[ m ].first[ len - 9 ] = 'c';
      models[ m ].first[ len - 8 ] = 'd';
      models[ m ].first[ len - 7 ] = '\0';
    }
  }
  visualizeNearestModels( argc, argv, k, thresh, models, k_indices, k_distances );

  return(1);
}

//--------------------------------------------------------------------------------------------
// function4

int classify_by_subspace( vfh_model feature, const char feature_type, int argc, char** argv, int dim_feature ){
  char dirname[ 300 ];
  char filename[ 300 ];
  Map<VectorXf> vec( &(feature.second[0]), feature.second.size() );

  if( feature_type == 'v' ){
    if( dim_feature == -1 ) dim_feature = 308;
    sprintf( dirname, "pca_result_vfh" );
  }
  else if( feature_type == 'c' ){
    if( dim_feature == -1 ) dim_feature = 981;
    sprintf( dirname, "pca_result_colorCHLAC" );
  }
  else{
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }

  // read parameters
  int obj_class_num, dim_subspace;
  FILE *fp = fopen( "pca_parameters.txt", "r" );
  if( fscanf( fp, "class_num: %d\n", &obj_class_num ) != 1 ) return -1;
  if( fscanf( fp, "dim: %d\n", &dim_subspace ) != 1 ) return -1;
  fclose( fp );
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

void compressFeature( string filename, vfh_model &feature, const int dim, bool ascii ){
  PCA pca;
  pca.read( filename.c_str(), ascii );
  MatrixXf tmpMat = pca.Axis();
  MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim);
  Map<VectorXf> vec( &(feature.second[0]), feature.second.size() );
  //vec = tmpMat2.transpose() * vec;
  VectorXf tmpvec = tmpMat2.transpose() * vec;
  feature.second.resize( dim );
  for( int t=0; t<dim; t++ )
    feature.second[t] = tmpvec[t];
}

//--------------------------------------------------------------------------------------------
int main( int argc, char** argv ){
  //ros::init( argc, argv, "test" );
  if( argc < 4 ){
    ROS_ERROR ("Need three parameters! Syntax is: %s {input_pointcloud_filename.pcd} {feature_initial(v or c)} {classifier_initial(k or s)} [options]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = size of compressed feature vectors\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    return(-1);
  }
  const char feature_type = argv[2][0];
  const char classifier_type = argv[3][0];
  vfh_model feature;
  char filename[ 300 ];
  int length;

  //------------
  //* features
  if( feature_type == 'v' ){
    //* compute - VFH -
    pcl::PointCloud<PointXYZ> cloud_object_cluster;
    readPoints( argv[1], cloud_object_cluster );
    std::vector<float> vfh_vector;
    getVFHsignature( cloud_object_cluster, vfh_vector );
    length = strlen( argv[1] );
    argv[1][ length-4 ] = '\0';
    sprintf(filename,"%s_vfh.pcd",argv[1]);
    feature.first = filename;
    feature.second = vfh_vector;
  }
  else if( feature_type == 'c' ){
    //* compute - colorCHLAC -
    pcl::PointCloud<PointXYZRGB> cloud_object_cluster2;
    readPoints( argv[1], cloud_object_cluster2 );
    std::vector<float> colorCHLAC;
    getColorCHLAC( cloud_object_cluster2, colorCHLAC );
    length = strlen( argv[1] );
    argv[1][ length-4 ] = '\0';
    sprintf(filename,"%s_colorCHLAC.pcd",argv[1]);
    feature.first = filename;
    feature.second = colorCHLAC;
  }
  else{
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }
  ROS_INFO ("Input feature vector extracted.\n");

  //--------------------------------------------------
  //* Compress the dimension of the vector (if needed)
  int dim = -1;
  if( parse_argument (argc, argv, "-dim", dim) > 0 ){
    if ((dim < 0)||(dim >= (int)feature.second.size())){
      print_error ("Invalid dimension (%d)!\n", dim);
      return (-1);
    }
    string filename;
    parse_argument (argc, argv, "-comp", filename);
    compressFeature( filename, feature, dim, false );
  }

  //--------------
  //* classifier
  if( classifier_type == 'k' ){
    //* classifier - kNN -
    classify_by_kNN( feature, feature_type, argc, argv );
  }
  else if( classifier_type == 's' ){
    //* classifier - SubspaceMethod -
    classify_by_subspace( feature, feature_type, argc, argv, dim );
  }
  else{
    ROS_ERROR ("Unknown classifier type.\n");
    return(-1);
  }

  return(0);
}
