#ifndef C3_HLAC_TOOLS_H_
#define C3_HLAC_TOOLS_H_

#include <sys/time.h>
#include "c3_hlac/c3_hlac.h"
#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/rsd.h>

//-----------
//* time
double t1,t2;
double my_clock();

//-----------
//* read
void readFeature(const char *name, std::vector< std::vector<float> > &feature );
void readFeature(const char *name, std::vector<float> &feature );

//-----------
//* write
bool if_zero_vec( const std::vector<float> vec );
void writeFeature(const char *name, const std::vector< std::vector<float> > feature, bool remove_0_flg = true );
void writeFeature(const char *name, const std::vector<float> feature, bool remove_0_flg = true );

//-----------
//* voxelize
template <typename T>
void getVoxelGrid( pcl::VoxelGrid<T> &grid, pcl::PointCloud<T> input_cloud, pcl::PointCloud<T>& output_cloud, const float voxel_size );

//------------------------
//* extract - C3HLAC -
template <typename PointT>
Eigen::Vector3i extract_C3_HLAC_Signature981(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int thR, int thG, int thB, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0 );

template <typename PointT>
void extract_C3_HLAC_Signature981(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int thR, int thG, int thB, const float voxel_size );

template <typename PointT>
Eigen::Vector3i extract_C3_HLAC_Signature117(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector< std::vector<float> > &feature, int thR, int thG, int thB, const float voxel_size, const int subdivision_size = 0, const int offset_x = 0, const int offset_y = 0, const int offset_z = 0 );

template <typename PointT>
void extract_C3_HLAC_Signature117(pcl::VoxelGrid<PointT> grid, pcl::PointCloud<PointT> cloud, std::vector<float> &feature, int thR, int thG, int thB, const float voxel_size );

#include <c3_hlac/c3_hlac_tools.hpp>

#endif  //#ifndef C3_HLAC_TOOLS_H_
