#include <sys/time.h>
#include "color_chlac/color_chlac.h"
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>
//#include "pcl/filters/voxel_grid.h"
//#include "pcl/kdtree/kdtree.h"
//#include "pcl/kdtree/kdtree_ann.h"
//#include "pcl/kdtree/organized_data.h"
//#include <algorithm>

//* GRSD type
#define NR_CLASS 5 
#define EMPTY 0 
#define PLANE 1 
#define CYLINDER 2
#define CIRCLE 3  
#define EDGE 4 
#define NOISE 5 

//* const variables
const double min_radius_plane_ = 0.066;
const double min_radius_noise_ = 0.030, max_radius_noise_ = 0.050;
const double max_min_radius_diff_ = 0.01;
const double min_radius_edge_ = 0.030;
//const float NORMALIZE_GRSD = NR_CLASS / 52.0; // 52 = 2 * 26
const float NORMALIZE_GRSD = NR_CLASS / 104.0; // 104 = 2 * 2 * 26

//-----------
//* time
double t1,t2;
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

//-----------
//* read
template <typename T>
bool readPoints( const char *name, pcl::PointCloud<T>& cloud ){
  if (pcl::io::loadPCDFile (name, cloud) == -1){
    ROS_ERROR ("Couldn't read file %s",name);
    return (-1);
  }
#ifndef QUIET
  ROS_INFO ("Loaded %d data points from %s with the following fields: %s", (int)(cloud.width * cloud.height), name, pcl::getFieldsList (cloud).c_str ());
#endif
  return(1);
}

//-----------
//* write
void writeFeature(const char *name, const std::vector< std::vector<float> > feature){
  const int hist_num = feature.size();
  const int feature_size = feature[ 0 ].size();
  FILE *fp = fopen( name, "w" );
  fprintf(fp,"# .PCD v.7 - Point Cloud Data file format\n");
  fprintf(fp,"FIELDS vfh\n");
  fprintf(fp,"SIZE 4\n");
  fprintf(fp,"TYPE F\n");
  fprintf(fp,"COUNT %d\n",feature_size);
  fprintf(fp,"WIDTH %d\n", hist_num);
  fprintf(fp,"HEIGHT 1\n");
  fprintf(fp,"POINTS %d\n", hist_num);
  fprintf(fp,"DATA ascii\n");
  for( int h=0; h<hist_num; h++ ){
    for(int t=0;t<feature_size;t++)
      fprintf(fp,"%f ",feature[ h ][ t ]);
    fprintf(fp,"\n");
  }
  fclose(fp);
}
void writeFeature(const char *name, const std::vector<float> feature){
  std::vector< std::vector<float> > tmp( 1 );
  tmp[ 0 ] = feature;
  writeFeature( name, tmp );
}

//------------------
//* compute normals
template <typename T1, typename T2>
void computeNormal( pcl::PointCloud<T1> input_cloud, pcl::PointCloud<T2>& output_cloud ){
  int k_ = 10;
  if ((int)input_cloud.points.size () < k_){
    ROS_WARN ("Filtering returned %d points! Continuing.", (int)input_cloud.points.size ());
    //pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_temp;
    //pcl::concatenateFields <Point, pcl::Normal, pcl::PointXYZRGBNormal> (cloud_filtered, cloud_normals, cloud_temp);
    //pcl::io::savePCDFile ("test.pcd", cloud, false);
  }
  pcl::NormalEstimation<T1, T2> n3d_;
  n3d_.setKSearch (k_);
  n3d_.setSearchMethod ( boost::make_shared<pcl::KdTreeFLANN<T1> > () );
  n3d_.setInputCloud ( boost::make_shared<const pcl::PointCloud<T1> > (input_cloud) );
  n3d_.compute (output_cloud);

  //* TODO: move the following lines to NormalEstimation class ?
  for ( int i=0; i< (int)input_cloud.points.size (); i++ ){
    output_cloud.points[ i ].x = input_cloud.points[ i ].x;
    output_cloud.points[ i ].y = input_cloud.points[ i ].y;
    output_cloud.points[ i ].z = input_cloud.points[ i ].z;
    output_cloud.points[ i ].rgb = input_cloud.points[ i ].rgb;
  }
}

//-----------
//* voxelize
template <typename T>
void getVoxelGrid( pcl::VoxelGrid<T> &grid, pcl::PointCloud<T> input_cloud, pcl::PointCloud<T>& output_cloud, const double voxel_size ){
  grid.setLeafSize (voxel_size, voxel_size, voxel_size);
  grid.setInputCloud ( boost::make_shared<const pcl::PointCloud<T> > (input_cloud) );
  grid.setSaveLeafLayout(true);
  grid.filter (output_cloud);
}

//--------------------
//* function for GRSD 
int get_type (float min_radius, float max_radius)
{
  if (min_radius > min_radius_plane_) // 0.066
    return PLANE; // plane
  else if ((min_radius < min_radius_noise_) && (max_radius < max_radius_noise_))
    return NOISE; // noise/corner
  else if (max_radius - min_radius < max_min_radius_diff_) // 0.0075
    return CIRCLE; // circle (corner?)
  else if (min_radius < min_radius_edge_) /// considering small cylinders to be edges
    return EDGE; // edge
  else
    return CYLINDER; // cylinder (rim)
}

//--------------------
//* compute - GRSD -
template <typename T>
void computeGRSD(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector< std::vector<float> > &feature, const double voxel_size, int subdivision_size = 0 ){
  double fixed_radius_search = 0.03;

  //* for computing multiple GRSD with subdivisions
  int hist_num = 1;
  float inverse_subdivision_size;
  Eigen3::Vector3i div_b_;
  Eigen3::Vector3i subdiv_b_;
  Eigen3::Vector3i subdivb_mul_;
  if( subdivision_size > 0 ){
    inverse_subdivision_size = 1.0 / subdivision_size;
    div_b_ = grid.getNrDivisions();
    subdiv_b_ = Eigen3::Vector3i ( ceil( div_b_[0]*inverse_subdivision_size ), ceil( div_b_[1]*inverse_subdivision_size ), ceil( div_b_[2]*inverse_subdivision_size ) );
    subdivb_mul_ = Eigen3::Vector3i ( 1, subdiv_b_[0], subdiv_b_[0] * subdiv_b_[1] );
    hist_num = subdiv_b_[0] * subdiv_b_[1] * subdiv_b_[2];
  }
  else if( subdivision_size < 0 ){
    std::cerr << "(In computeGRSD) Invalid subdivision size: " << subdivision_size << std::endl;
    return;
  }

  // Compute RSD
  pcl::RSDEstimation <T, T, pcl::PrincipalRadiiRSD> rsd;
  rsd.setInputCloud( boost::make_shared<const pcl::PointCloud<T> > (cloud_downsampled) );
  rsd.setSearchSurface( boost::make_shared<const pcl::PointCloud<T> > (cloud) );
  rsd.setInputNormals( boost::make_shared<const pcl::PointCloud<T> > (cloud) );
#ifndef QUIET
  ROS_INFO("radius search: %f", std::max(fixed_radius_search, voxel_size/2 * sqrt(3)));
#endif
  rsd.setRadiusSearch(std::max(fixed_radius_search, voxel_size/2 * sqrt(3)));
  ( boost::make_shared<pcl::KdTreeFLANN<T> > () )->setInputCloud ( boost::make_shared<const pcl::PointCloud<T> > (cloud) );
  rsd.setSearchMethod( boost::make_shared<pcl::KdTreeFLANN<T> > () );
  pcl::PointCloud<pcl::PrincipalRadiiRSD> radii;
  rsd.compute(radii);
  
  // Get rmin/rmax for adjacent 27 voxel
  Eigen3::MatrixXi relative_coordinates (3, 13);

  //Eigen3::MatrixXi transition_matrix =  Eigen3::MatrixXi::Zero(6, 6);
  std::vector< Eigen3::MatrixXi > transition_matrix( hist_num );
  for( int i=0; i<hist_num; i++ )
    transition_matrix[ i ] =  Eigen3::MatrixXi::Zero(6, 6);

  int idx = 0;
  
  // 0 - 8
  for( int i=-1; i<2; i++ )
  {
    for( int j=-1; j<2; j++ )
    {
      relative_coordinates( 0, idx ) = i;
      relative_coordinates( 1, idx ) = j;
      relative_coordinates( 2, idx ) = -1;
      idx++;
    }
  }
  // 9 - 11
  for( int i=-1; i<2; i++ )
  {
    relative_coordinates( 0, idx ) = i;
    relative_coordinates( 1, idx ) = -1;
    relative_coordinates( 2, idx ) = 0;
    idx++;
  }
  // 12
  relative_coordinates( 0, idx ) = -1;
  relative_coordinates( 1, idx ) = 0;
  relative_coordinates( 2, idx ) = 0;

  Eigen3::MatrixXi relative_coordinates_all (3, 26);
  relative_coordinates_all.block<3, 13>(0, 0) = relative_coordinates;
  relative_coordinates_all.block<3, 13>(0, 13) = -relative_coordinates;
  
  // Get transition matrix
  std::vector<int> types (radii.points.size());
 
  for (size_t idx = 0; idx < radii.points.size (); ++idx)
    types[idx] = get_type(radii.points[idx].r_min, radii.points[idx].r_max);
  
  for (size_t idx = 0; idx < (boost::make_shared<const pcl::PointCloud<T> > (cloud_downsampled))->points.size (); ++idx)
  {
    // calc hist_idx
    int hist_idx;
    if( hist_num == 1 ) hist_idx = 0;
    else{
      const int x_mul_y = div_b_[0] * div_b_[1];
      const int tmp_z = idx / x_mul_y;
      const int tmp_y = ( idx % x_mul_y ) / div_b_[0];
      const int tmp_x = idx % div_b_[0];
      Eigen3::Vector3i ijk = Eigen3::Vector3i ( floor ( tmp_x * inverse_subdivision_size), floor ( tmp_y * inverse_subdivision_size), floor ( tmp_z * inverse_subdivision_size) );
      hist_idx = ijk.dot (subdivb_mul_);
    }

    int source_type = types[idx];
    std::vector<int> neighbors = grid.getNeighborCentroidIndices ((boost::make_shared<const pcl::PointCloud<T> > (cloud_downsampled))->points[idx], relative_coordinates);
    for (unsigned id_n = 0; id_n < neighbors.size(); id_n++)
    {
      int neighbor_type;
      if (neighbors[id_n] == -1)
        neighbor_type = EMPTY;
      else
        neighbor_type = types[neighbors[id_n]];

      transition_matrix[ hist_idx ](source_type, neighbor_type)++;
    }
  }

  pcl::PointCloud<pcl::GRSDSignature21> cloud_grsd;
  cloud_grsd.points.resize(hist_num);
  
  for( int h=0; h<hist_num; h++ ){
    int nrf = 0;
    for (int i=1; i<NR_CLASS+1; i++)
      for (int j=0; j<=i; j++)
	cloud_grsd.points[ h ].histogram[nrf++] = transition_matrix[ h ](i, j); //@TODO: resize point cloud
  }

  feature.resize( hist_num );
  for( int h=0; h<hist_num; h++ ){
    feature[ h ].resize( 20 );
    for( int i=0; i<20; i++)
      feature[ h ][ i ] = cloud_grsd.points[ h ].histogram[ i ] * NORMALIZE_GRSD;
  }
}

template <typename T>
void computeGRSD(pcl::VoxelGrid<T> grid, pcl::PointCloud<T> cloud, pcl::PointCloud<T> cloud_downsampled, std::vector<float> &feature, const double voxel_size ){
  std::vector< std::vector<float> > tmp( 1 );
  computeGRSD( grid, cloud, cloud_downsampled, tmp, voxel_size, 0 ); // for one signature
  feature = tmp[ 0 ];
}

//------------------------
//* compute - ColorCHLAC -
template <typename PointT>
void computeColorCHLAC(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int thR, int thG, int thB, int subdivision_size = 0 ){
    pcl::PointCloud<pcl::ColorCHLACSignature981> colorCHLAC_signature;
  pcl::ColorCHLACEstimation<PointT, pcl::ColorCHLACSignature981> colorCHLAC_;

  colorCHLAC_.setRadiusSearch (1.8);
  colorCHLAC_.setSearchMethod ( boost::make_shared<pcl::KdTreeFLANN<PointT> > () );
  colorCHLAC_.setColorThreshold( thR, thG, thB );
  colorCHLAC_.setVoxelFilter (grid, subdivision_size);
  colorCHLAC_.setInputCloud ( boost::make_shared<const pcl::PointCloud<PointT> > (cloud) );
  t1 = my_clock();
  colorCHLAC_.compute( colorCHLAC_signature );
  t2 = my_clock();
#ifndef QUIET
  ROS_INFO (" %d colorCHLAC estimated. (%f sec)", (int)colorCHLAC_signature.points.size (), t2-t1);
#endif
  const int hist_num = colorCHLAC_signature.points.size();
  feature.resize( hist_num );
  for( int h=0; h<hist_num; h++ ){
    feature[ h ].resize( DIM_COLOR_1_3_ALL );
    for( int i=0; i<DIM_COLOR_1_3_ALL; i++)
      feature[ h ][ i ] = colorCHLAC_signature.points[ h ].histogram[ i ];
  }
}

template <typename PointT>
void computeColorCHLAC(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int thR, int thG, int thB ){
  std::vector< std::vector<float> > tmp( 1 );
  computeColorCHLAC( grid, cloud, tmp, thR, thG, thB ); // for one signature
  feature = tmp[ 0 ];
}

template <typename PointT>
void computeColorCHLAC_RI(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int thR, int thG, int thB, int subdivision_size = 0 ){
  pcl::PointCloud<pcl::ColorCHLACSignature117> colorCHLAC_signature;
  pcl::ColorCHLAC_RI_Estimation<PointT, pcl::ColorCHLACSignature117> colorCHLAC_;

  colorCHLAC_.setRadiusSearch (1.8);
  colorCHLAC_.setSearchMethod ( boost::make_shared<pcl::KdTreeFLANN<PointT> > () );
  colorCHLAC_.setColorThreshold( thR, thG, thB );
  colorCHLAC_.setVoxelFilter (grid, subdivision_size);
  colorCHLAC_.setInputCloud ( boost::make_shared<const pcl::PointCloud<PointT> > (cloud) );
  t1 = my_clock();
  colorCHLAC_.compute( colorCHLAC_signature );
  t2 = my_clock();
#ifndef QUIET
  ROS_INFO (" %d colorCHLAC estimated. (%f sec)", (int)colorCHLAC_signature.points.size (), t2-t1);
#endif
  const int hist_num = colorCHLAC_signature.points.size();
  feature.resize( hist_num );
  for( int h=0; h<hist_num; h++ ){
    feature[ h ].resize( DIM_COLOR_RI_1_3_ALL );
    for( int i=0; i<DIM_COLOR_RI_1_3_ALL; i++)
      feature[ h ][ i ] = colorCHLAC_signature.points[ h ].histogram[ i ];
  }
}

template <typename PointT>
void computeColorCHLAC_RI(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int thR, int thG, int thB ){
  std::vector< std::vector<float> > tmp( 1 );
  computeColorCHLAC_RI( grid, cloud, tmp, thR, thG, thB ); // for one signature
  feature = tmp[ 0 ];
}

//--------------
//* concatenate
const std::vector<float> conc_vector( const std::vector<float> v1, const std::vector<float> v2 ){
  std::vector<float> vec = v1;
  vec.insert(vec.end(), v2.begin(), v2.end());
  return vec;

}
