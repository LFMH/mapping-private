#ifndef PCL_COLOR_CHLAC_H_
#define PCL_COLOR_CHLAC_H_

#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>

const int DIM_COLOR_1_3 = 495;        // Dimension of feature vector (without RGB binalize)
const int DIM_COLOR_BIN_1_3 = 486;    // Dimension of feature vector (with RGB binalize)
const int DIM_COLOR_1_3_ALL = 981;    // = DIM_COLOR_1_3 + DIM_COLOR_BIN_1_3
const int DIM_COLOR_RI_1_3 = 63;      // Dimension of feature vector (without RGB binalize) - rotation-invariant -
const int DIM_COLOR_RI_BIN_1_3 = 54;  // Dimension of feature vector (with RGB binalize) - rotation-invariant -
const int DIM_COLOR_RI_1_3_ALL = 117; // = DIM_COLOR_RI_1_3 + DIM_COLOR_RI_BIN_1_3
//const float NORMALIZE_0 = 1/255.0;    // value for normalizing 0th-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_0 = 1/510.0;    // value for normalizing 0th-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_1 = 1/65025.0;  // value for normalizing 1st-order Color-CHLAC (without RGB binalize)
//const float NORMALIZE_0_BIN = 1;    // value for normalizing 0th-order Color-CHLAC (with RGB binalize)
//const float NORMALIZE_1_BIN = 1;    // value for normalizing 1st-order Color-CHLAC (with RGB binalize)
//const float NORMALIZE_RI_0 = 1/255.0;    // value for normalizing 0th-order Color-CHLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_0 = 1/510.0;    // value for normalizing 0th-order Color-CHLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1 = 1/845325.0; // value for normalizing 1st-order Color-CHLAC (without RGB binalize) - rotation-invariant -
//const float NORMALIZE_RI_0_BIN = 1;    // value for normalizing 0th-order Color-CHLAC (with RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_0_BIN = 1/2.0;    // value for normalizing 0th-order Color-CHLAC (with RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1_BIN = 1/13.0; // value for normalizing 1st-order Color-CHLAC (with RGB binalize) - rotation-invariant -

namespace pcl
{
  //* Mode for 90 degrees rotation
  enum RotateMode{ R_MODE_1, R_MODE_2, R_MODE_3, R_MODE_4 };

  //* calculate a feature vector when the target voxel data is rotated by 90 degrees.
  inline void rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode );

  template <typename PointT, typename PointOutT>
  class ColorCHLACEstimation: public Feature<PointT, PointOutT>
    {
    public:
      using Feature<PointT, PointOutT>::feature_name_;
      using Feature<PointT, PointOutT>::getClassName;
      using Feature<PointT, PointOutT>::indices_;
      using Feature<PointT, PointOutT>::surface_;
      using Feature<PointT, PointOutT>::k_;
      using Feature<PointT, PointOutT>::search_parameter_;

      typedef typename Feature<PointT, PointOutT>::PointCloudOut PointCloudOut;

      //inline void setVoxelSize( float val ){ voxel_size = val; setRadiusSearch (val*1.75); }
      
/*       inline void getVoxelGrid ( const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &voxel_grid ); */
      
      inline void setColorThreshold ( int thR, int thG, int thB ){ color_thR = thR; color_thG = thG; color_thB = thB; }

      inline void setVoxelFilter ( pcl::VoxelGrid<PointT> grid_, const int subdivision_size_ );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //ColorCHLACEstimation () : voxel_size (0)
      ColorCHLACEstimation () : hist_num (1), color_thR (-1), color_thG (-1), color_thB (-1)
      {
        feature_name_ = "ColorCHLACEstimation";
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
      };

    protected:
      inline int binarize_r ( int val );
      inline int binarize_g ( int val );
      inline int binarize_b ( int val );

      //* functions for ColorCHLACSignature981 (rotation-variant) *//
      virtual inline void addColorCHLAC_0 ( const int idx, PointCloudOut &output );
      virtual inline void addColorCHLAC_0_bin ( const int idx, PointCloudOut &output );
      virtual inline void addColorCHLAC_1 ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b );
      virtual inline void addColorCHLAC_1_bin ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b );
      virtual inline void computeColorCHLAC ( const pcl::PointCloud<PointT> &cloud, PointCloudOut &output, const int center_idx );
      virtual inline void normalizeColorCHLAC ( PointCloudOut &output );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      void computeFeature (PointCloudOut &output);

    protected:
      pcl::VoxelGrid<PointT> grid;
      Eigen3::MatrixXi relative_coordinates;
      int hist_num;
      float inverse_subdivision_size;
      Eigen3::Vector3i div_b_;
      Eigen3::Vector3i subdiv_b_;
      Eigen3::Vector3i subdivb_mul_;
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

  template <typename PointT, typename PointOutT>
  class ColorCHLAC_RI_Estimation: public ColorCHLACEstimation<PointT, PointOutT>
    {
    public:
      using ColorCHLACEstimation<PointT, PointOutT>::feature_name_;
      using ColorCHLACEstimation<PointT, PointOutT>::getClassName;
      using ColorCHLACEstimation<PointT, PointOutT>::indices_;
      using ColorCHLACEstimation<PointT, PointOutT>::surface_;
      using ColorCHLACEstimation<PointT, PointOutT>::k_;
      using ColorCHLACEstimation<PointT, PointOutT>::search_parameter_;

      typedef typename ColorCHLACEstimation<PointT, PointOutT>::PointCloudOut PointCloudOut;

      //inline void setColorThreshold ( int thR, int thG, int thB );
      //inline void setVoxelFilter ( pcl::VoxelGrid<PointT> grid_ );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //ColorCHLACEstimation () : voxel_size (0)
      ColorCHLAC_RI_Estimation ()
      {
        feature_name_ = "ColorCHLAC_RI_Estimation";
      };

    protected:
      inline int binarize_r ( int val );
      inline int binarize_g ( int val );
      inline int binarize_b ( int val );

      //* functions for ColorCHLACSignature117 (rotation-invariant) *//
      inline void addColorCHLAC_0 ( const int idx, PointCloudOut &output );
      inline void addColorCHLAC_0_bin ( const int idx, PointCloudOut &output );
      inline void addColorCHLAC_1 ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b );
      inline void addColorCHLAC_1_bin ( const int idx, PointCloudOut &output, int neighbor_idx, int r, int g, int b );
      //inline void computeColorCHLAC (const pcl::PointCloud<PointT> &cloud, PointCloudOut &output, const int center_idx );
      inline void normalizeColorCHLAC ( PointCloudOut &output );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      void computeFeature (PointCloudOut &output);

    protected:
      using ColorCHLACEstimation<PointT, PointOutT>::grid;
      using ColorCHLACEstimation<PointT, PointOutT>::relative_coordinates;
      using ColorCHLACEstimation<PointT, PointOutT>::hist_num;
      using ColorCHLACEstimation<PointT, PointOutT>::inverse_subdivision_size;
      using ColorCHLACEstimation<PointT, PointOutT>::div_b_;
      using ColorCHLACEstimation<PointT, PointOutT>::subdiv_b_;
      using ColorCHLACEstimation<PointT, PointOutT>::subdivb_mul_;
      using ColorCHLACEstimation<PointT, PointOutT>::color;
      using ColorCHLACEstimation<PointT, PointOutT>::center_r;
      using ColorCHLACEstimation<PointT, PointOutT>::center_g;
      using ColorCHLACEstimation<PointT, PointOutT>::center_b;
      using ColorCHLACEstimation<PointT, PointOutT>::center_r_;
      using ColorCHLACEstimation<PointT, PointOutT>::center_g_;
      using ColorCHLACEstimation<PointT, PointOutT>::center_b_;
      using ColorCHLACEstimation<PointT, PointOutT>::center_bin_r;
      using ColorCHLACEstimation<PointT, PointOutT>::center_bin_g;
      using ColorCHLACEstimation<PointT, PointOutT>::center_bin_b;
      using ColorCHLACEstimation<PointT, PointOutT>::color_thR;
      using ColorCHLACEstimation<PointT, PointOutT>::color_thG;
      using ColorCHLACEstimation<PointT, PointOutT>::color_thB;
  };
}

#include "color_chlac.hpp"

#endif  //#ifndef PCL_COLOR_CHLAC_H_


