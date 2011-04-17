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
  std::cout<< "hogehoge 1" << std::endl;
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extractC3HLACSignature117( grid, cloud_downsampled, c3hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );
  std::cout<< "hogehoge 2" << std::endl;

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  std::cout<< "hogehoge 3" << std::endl;
  for( int h=0; h<h_num; h++ ){
    feature.push_back( concVector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  std::cout<< "hogehoge" << std::endl;

  setData( subdiv_b_, feature );
}

void SearchConVOSCH::setConVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extractC3HLACSignature981( grid, cloud_downsampled, c3hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    feature.push_back( concVector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  setData( subdiv_b_, feature );
}

void SearchGRSD::setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = grsd.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    feature.push_back( grsd[ h ] );
    exist_voxel_num[ h ] = 0;
    for( int i=0; i<20; i++ )
      exist_voxel_num[ h ] += grsd[ h ][ i ];
    exist_voxel_num[ h ] /= 26; // ボクセルの数 before adding up
  }
  setData( subdiv_b_, feature );
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
  std::cout<< "hogehoge 1" << std::endl;
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extractC3HLACSignature117( grid, cloud_downsampled, c3hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );
  std::cout<< "hogehoge 2" << std::endl;

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  std::cout<< "hogehoge 3" << std::endl;
  for( int h=0; h<h_num; h++ ){
    feature.push_back( concVector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  std::cout<< "hogehoge" << std::endl;

  setData( subdiv_b_, feature );
}

void SearchConVOSCHMulti::setConVOSCH( int dim, int color_threshold_r, int color_threshold_g, int color_threshold_b, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extractC3HLACSignature981( grid, cloud_downsampled, c3hlac, color_threshold_r, color_threshold_g, color_threshold_b, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    feature.push_back( concVector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  setData( subdiv_b_, feature );
}

void SearchGRSDMulti::setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extractGRSDSignature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = grsd.size();
  exist_voxel_num = new int[ h_num ];
  integral_features = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    feature.push_back( grsd[ h ] );
    exist_voxel_num[ h ] = 0;
    for( int i=0; i<20; i++ )
      exist_voxel_num[ h ] += grsd[ h ][ i ];
    exist_voxel_num[ h ] /= 26; // ボクセルの数 before adding up
  }
  setData( subdiv_b_, feature );
}

#endif
