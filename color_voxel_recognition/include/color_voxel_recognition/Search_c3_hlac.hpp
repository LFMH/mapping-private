#include <color_voxel_recognition/Search.hpp>
#include "c3_hlac/c3_hlac_tools.h"

////////////
// single //
////////////

class Search_c3_hlac : public SearchObj {
public:
  Search_c3_hlac(){}
  ~Search_c3_hlac(){}
  void set_C3_HLAC( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void Search_c3_hlac::set_C3_HLAC( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  std::vector< std::vector<float> > c3hlac;
  Eigen::Vector3i subdiv_b_ = extract_C3_HLAC_Signature981( grid, cloud_downsampled, c3hlac, thR, thG, thB, voxel_size, subdivision_size );

  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // the number of occupied voxels before adding up
  setData( subdiv_b_, c3hlac );
}

///////////
// multi //
///////////

class Search_c3_hlac_multi : public SearchObj_multi {
public:
  Search_c3_hlac_multi(){}
  ~Search_c3_hlac_multi(){}
  void set_C3_HLAC( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void Search_c3_hlac_multi::set_C3_HLAC( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGB> grid, pcl::PointCloud<pcl::PointXYZRGB> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  std::vector< std::vector<float> > c3hlac;
  Eigen::Vector3i subdiv_b_ = extract_C3_HLAC_Signature981( grid, cloud_downsampled, c3hlac, thR, thG, thB, voxel_size, subdivision_size );

  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // the number of occupied voxels before adding up
  setData( subdiv_b_, c3hlac );
}
