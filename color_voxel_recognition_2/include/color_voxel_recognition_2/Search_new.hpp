#include <color_voxel_recognition/Search.hpp>
#include "c3_hlac/c3_hlac_tools.h"
#include "vosch/vosch_tools.h"

////////////
// single //
////////////

class Search_VOSCH : public SearchObj {
public:
  Search_VOSCH(){}
  ~Search_VOSCH(){}
  void setVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class Search_ConVOSCH : public SearchObj {
public:
  Search_ConVOSCH(){}
  ~Search_ConVOSCH(){}
  void setConVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class Search_GRSD : public SearchObj {
public:
  Search_GRSD(){}
  ~Search_GRSD(){}
  void setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void Search_VOSCH::setVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  std::cout<< "hogehoge 1" << std::endl;
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extract_GRSD_Signature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extract_C3_HLAC_Signature117( grid, cloud_downsampled, c3hlac, thR, thG, thB, voxel_size, subdivision_size );
  std::cout<< "hogehoge 2" << std::endl;

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
  std::cout<< "hogehoge 3" << std::endl;
  for( int h=0; h<h_num; h++ ){
    feature.push_back( conc_vector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  std::cout<< "hogehoge" << std::endl;

  setData( subdiv_b_, feature );
}

void Search_ConVOSCH::setConVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extract_GRSD_Signature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extract_C3_HLAC_Signature981( grid, cloud_downsampled, c3hlac, thR, thG, thB, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    feature.push_back( conc_vector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  setData( subdiv_b_, feature );
}

void Search_GRSD::setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extract_GRSD_Signature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = grsd.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
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

class Search_VOSCH_multi : public SearchObj_multi {
public:
  Search_VOSCH_multi(){}
  ~Search_VOSCH_multi(){}
  void setVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class Search_ConVOSCH_multi : public SearchObj_multi {
public:
  Search_ConVOSCH_multi(){}
  ~Search_ConVOSCH_multi(){}
  void setConVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

class Search_GRSD_multi : public SearchObj_multi {
public:
  Search_GRSD_multi(){}
  ~Search_GRSD_multi(){}
  void setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size );
};

//////////

void Search_VOSCH_multi::setVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  std::cout<< "hogehoge 1" << std::endl;
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extract_GRSD_Signature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extract_C3_HLAC_Signature117( grid, cloud_downsampled, c3hlac, thR, thG, thB, voxel_size, subdivision_size );
  std::cout<< "hogehoge 2" << std::endl;

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
  std::cout<< "hogehoge 3" << std::endl;
  for( int h=0; h<h_num; h++ ){
    feature.push_back( conc_vector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  std::cout<< "hogehoge" << std::endl;

  setData( subdiv_b_, feature );
}

void Search_ConVOSCH_multi::setConVOSCH( int dim, int thR, int thG, int thB, pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud, pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extract_GRSD_Signature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );
  //* extract - C3HLAC -
  std::vector< std::vector<float> > c3hlac;
  extract_C3_HLAC_Signature981( grid, cloud_downsampled, c3hlac, thR, thG, thB, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = c3hlac.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    feature.push_back( conc_vector( grsd[ h ], c3hlac[ h ] ) );
    exist_voxel_num[ h ] = ( c3hlac[h][0] + c3hlac[h][1] ) * 2 + 0.001; // ボクセルの数 before adding up
  }
  setData( subdiv_b_, feature );
}

void Search_GRSD_multi::setGRSD( int dim, pcl::VoxelGrid<pcl::PointNormal> grid, pcl::PointCloud<pcl::PointNormal> cloud, pcl::PointCloud<pcl::PointNormal> cloud_downsampled, const double voxel_size, const int subdivision_size ){
  //* extract - GRSD -
  std::vector< std::vector<float> > grsd;
  Eigen::Vector3i subdiv_b_ = extract_GRSD_Signature21( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size );

  std::vector< std::vector<float> > feature;
  const int h_num = grsd.size();
  exist_voxel_num = new int[ h_num ];
  nFeatures = new Eigen::VectorXf [ h_num ];
  for( int h=0; h<h_num; h++ ){
    feature.push_back( grsd[ h ] );
    exist_voxel_num[ h ] = 0;
    for( int i=0; i<20; i++ )
      exist_voxel_num[ h ] += grsd[ h ][ i ];
    exist_voxel_num[ h ] /= 26; // ボクセルの数 before adding up
  }
  setData( subdiv_b_, feature );
}
