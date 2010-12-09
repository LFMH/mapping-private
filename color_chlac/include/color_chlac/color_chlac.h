#ifndef PCL_COLOR_CHLAC_H_
#define PCL_COLOR_CHLAC_H_

#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>

const int DIM_COLOR_1_3 = 495;        // Dimension of feature vector (without RGB binalize)
const int DIM_COLOR_BIN_1_3 = 486;    // Dimension of feature vector (with RGB binalize)
const int DIM_COLOR_1_3_ALL = 981;    // = DIM_COLOR_1_3 + DIM_COLOR_BIN_1_3
const float NORMALIZE_0 = 1/765.0;    // value for normalizing 0th-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_1 = 1/585225.0; // value for normalizing 1st-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_0_BIN = 1/3.0;  // value for normalizing 0th-order Color-CHLAC (with RGB binalize)
const float NORMALIZE_1_BIN = 1/9.0;  // value for normalizing 1st-order Color-CHLAC (with RGB binalize)

namespace pcl
{
  class ColorCHLACEstimation: public Feature<PointXYZRGB, ColorCHLACSignature981>
    {
    public:
      using Feature<PointXYZRGB, ColorCHLACSignature981>::feature_name_;
      using Feature<PointXYZRGB, ColorCHLACSignature981>::getClassName;
      using Feature<PointXYZRGB, ColorCHLACSignature981>::indices_;
      using Feature<PointXYZRGB, ColorCHLACSignature981>::surface_;
      using Feature<PointXYZRGB, ColorCHLACSignature981>::k_;
      using Feature<PointXYZRGB, ColorCHLACSignature981>::search_parameter_;

      typedef Feature<PointXYZRGB, ColorCHLACSignature981>::PointCloudOut PointCloudOut;

      //inline void setVoxelSize( float val ){ voxel_size = val; setRadiusSearch (val*1.75); }
      
/*       inline void getVoxelGrid ( const pcl::PointCloud<PointXYZRGB> &cloud, pcl::PointCloud<PointXYZRGB> &voxel_grid ); */
      
      inline void setColorThreshold ( int thR, int thG, int thB ){ color_thR = thR; color_thG = thG; color_thB = thB; }

      inline void setVoxelFilter ( pcl::VoxelGrid<PointXYZRGB> grid_ ){ grid = grid_; }

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //ColorCHLACEstimation () : voxel_size (0)
      ColorCHLACEstimation () : color_thR (-1), color_thG (-1), color_thB (-1)
      {
        feature_name_ = "ColorCHLACEstimation";
#if 0
	// 0 - 8
	for( int i=-1; i<2; i++ )
	  for( int j=-1; j<2; j++ )
	    relative_coordinates.push_back (Eigen3::Vector3i (i, j, -1)); 
	// 9 - 11
	for( int i=-1; i<2; i++ )
	  relative_coordinates.push_back (Eigen3::Vector3i (i, -1, 0)); 
	// 12
	relative_coordinates.push_back (Eigen3::Vector3i (-1, 0, 0)); 
#else
	relative_coordinates.resize(3, 13);
	int idx = 0;
	// 0 - 8
	for( int i=-1; i<2; i++ ){
	  for( int j=-1; j<2; j++ ){
	    relative_coordinates( 0, idx ) = i;
	    relative_coordinates( 1, idx ) = j;
	    relative_coordinates( 2, idx ) = -1;
	    idx++;
	  }
	}
	// 9 - 11
	for( int i=-1; i<2; i++ ){
	  relative_coordinates( 0, idx ) = i;
	  relative_coordinates( 1, idx ) = -1;
	  relative_coordinates( 2, idx ) = 0;
	  idx++;
	}
	// 12
	relative_coordinates( 0, idx ) = -1;
	relative_coordinates( 1, idx ) = 0;
	relative_coordinates( 2, idx ) = 0;
#endif
      };

    protected:
      inline void computeColorCHLAC (const pcl::PointCloud<PointXYZRGB> &cloud, PointCloudOut &output, const int center_idx );
      inline int binarize_r ( int val );
      inline int binarize_g ( int val );
      inline int binarize_b ( int val );

      inline void addColorCHLAC_0 ( PointCloudOut &output );
      inline void addColorCHLAC_0_bin ( PointCloudOut &output );
      inline void addColorCHLAC_1 ( PointCloudOut &output, int neighbor_idx, int r, int g, int b );
      inline void addColorCHLAC_1_bin ( PointCloudOut &output, int neighbor_idx, int r, int g, int b );

      inline void normalizeColorCHLAC ( PointCloudOut &output );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      void computeFeature (PointCloudOut &output);

    private:
      pcl::VoxelGrid<PointXYZRGB> grid;
      Eigen3::MatrixXi relative_coordinates;
      int color;
      int center_r;
      int center_g;
      int center_b;
      int center_r_;
      int center_g_;
      int center_b_;
      int center_bin_r;
      int center_bin_g;
      int center_bin_b;
      int color_thR;
      int color_thG;
      int color_thB;
  };
}

#include "color_chlac.hpp"

#endif  //#ifndef PCL_COLOR_CHLAC_H_


