#ifndef COLOR_VOXEL_RECOGNITION_2_SEARCH_NEW_H_
#define COLOR_VOXEL_RECOGNITION_2_SEARCH_NEW_H_
#include <color_voxel_recognition/search.h>
#include "c3_hlac/c3_hlac_tools.h"
#include "vosch/vosch_tools.h"

////////////
// single //
////////////

class SearchVOSCH : public SearchObj {
public:
  SearchVOSCH(){}
  ~SearchVOSCH(){}
  void setVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class SearchConVOSCH : public SearchObj {
public:
  SearchConVOSCH(){}
  ~SearchConVOSCH(){}
  void setConVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class SearchGRSD : public SearchObj {
public:
  SearchGRSD(){}
  ~SearchGRSD(){}
  void setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void SearchVOSCH::setVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - VOSCH -
  std::vector< std::vector<float> > vosch;
  Eigen::Vector3i subdiv_b_ = extractVOSCH( grid, cloud, cloud_downsampled, vosch, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  const int h_num = vosch.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( vosch[h][20] + vosch[h][21] ) * 2 + 0.001; // the number of occupied voxels before adding up

  setData( subdiv_b_, vosch );
}

void SearchConVOSCH::setConVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - ConVOSCH -
  std::vector< std::vector<float> > convosch;
  Eigen::Vector3i subdiv_b_ = extractConVOSCH( grid, cloud, cloud_downsampled, convosch, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  const int h_num = convosch.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( convosch[h][20] + convosch[h][21] ) * 2 + 0.001; // the number of occupied voxels before adding up

  setData( subdiv_b_, convosch );
}

void SearchGRSD::setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );

  const int h_num = grsd.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    exist_voxel_num[ h ] = 0;
    for( int i=0; i<20; i++ )
      exist_voxel_num[ h ] += grsd[ h ][ i ];
    exist_voxel_num[ h ] /= 26;  // the number of occupied voxels before adding up
  }
  setData( subdiv_b_, grsd );
}

///////////
// multi //
///////////

class SearchVOSCHMulti : public SearchObjMulti {
public:
  SearchVOSCHMulti(){}
  ~SearchVOSCHMulti(){}
  void setVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class SearchConVOSCHMulti : public SearchObjMulti {
public:
  SearchConVOSCHMulti(){}
  ~SearchConVOSCHMulti(){}
  void setConVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class SearchGRSDMulti : public SearchObjMulti {
public:
  SearchGRSDMulti(){}
  ~SearchGRSDMulti(){}
  void setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void SearchVOSCHMulti::setVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - VOSCH -
  std::vector< std::vector<float> > vosch;
  Eigen::Vector3i subdiv_b_ = extractVOSCH( grid, cloud, cloud_downsampled, vosch, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  const int h_num = vosch.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( vosch[h][20] + vosch[h][21] ) * 2 + 0.001; // the number of occupied voxels before adding up

  setData( subdiv_b_, vosch );
}

void SearchConVOSCHMulti::setConVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - ConVOSCH -
  std::vector< std::vector<float> > convosch;
  Eigen::Vector3i subdiv_b_ = extractConVOSCH( grid, cloud, cloud_downsampled, convosch, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  const int h_num = convosch.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ )
    exist_voxel_num[ h ] = ( convosch[h][20] + convosch[h][21] ) * 2 + 0.001; // the number of occupied voxels before adding up

  setData( subdiv_b_, convosch );
}

void SearchGRSDMulti::setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );

  const int h_num = grsd.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    exist_voxel_num[ h ] = 0;
    for( int i=0; i<20; i++ )
      exist_voxel_num[ h ] += grsd[ h ][ i ];
    exist_voxel_num[ h ] /= 26;  // the number of occupied voxels before adding up
  }
  setData( subdiv_b_, grsd );
}

#endif
