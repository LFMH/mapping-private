#ifndef PCL_COLOR_CHLAC_H_
#define PCL_COLOR_CHLAC_H_

#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>

//#define ENABLE_NORMALIZATION
#define C3_HLAC
#ifdef C3_HLAC
const float angle_norm = M_PI / 510;
const float AVERAGE_COLOR_VAL = 0.7;
#else
const float AVERAGE_COLOR_VAL = 0.5;
#endif
const float AVERAGE_COLOR_VAL_BIN = 0.5;

const int DIM_COLOR_1_3 = 495;        // Dimension of feature vector (without RGB binalize)
const int DIM_COLOR_BIN_1_3 = 486;    // Dimension of feature vector (with RGB binalize)
const int DIM_COLOR_1_3_ALL = 981;    // = DIM_COLOR_1_3 + DIM_COLOR_BIN_1_3
const int DIM_COLOR_RI_1_3 = 63;      // Dimension of feature vector (without RGB binalize) - rotation-invariant -
const int DIM_COLOR_RI_BIN_1_3 = 54;  // Dimension of feature vector (with RGB binalize) - rotation-invariant -
const int DIM_COLOR_RI_1_3_ALL = 117; // = DIM_COLOR_RI_1_3 + DIM_COLOR_RI_BIN_1_3

#ifdef ENABLE_NORMALIZATION
const float NORMALIZE_0 = 1/(255.0*AVERAGE_COLOR_VAL);    // value for normalizing 0th-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_1 = 1/(65025.0*AVERAGE_COLOR_VAL*AVERAGE_COLOR_VAL);  // value for normalizing 1st-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_0_BIN = 1/AVERAGE_COLOR_VAL_BIN;    // value for normalizing 0th-order Color-CHLAC (with RGB binalize)
const float NORMALIZE_1_BIN = 1/(AVERAGE_COLOR_VAL_BIN*AVERAGE_COLOR_VAL_BIN);    // value for normalizing 1st-order Color-CHLAC (with RGB binalize)
const float NORMALIZE_RI_0 = 1/(255.0*AVERAGE_COLOR_VAL);    // value for normalizing 0th-order Color-CHLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1 = 1/(845325.0*AVERAGE_COLOR_VAL*AVERAGE_COLOR_VAL); // value for normalizing 1st-order Color-CHLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_0_BIN = 1/AVERAGE_COLOR_VAL_BIN;    // value for normalizing 0th-order Color-CHLAC (with RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1_BIN = 1/(13.0*AVERAGE_COLOR_VAL_BIN*AVERAGE_COLOR_VAL_BIN); // value for normalizing 1st-order Color-CHLAC (with RGB binalize) - rotation-invariant -
#else
const float NORMALIZE_0 = 1/255.0;    // value for normalizing 0th-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_1 = 1/65025.0;  // value for normalizing 1st-order Color-CHLAC (without RGB binalize)
const float NORMALIZE_0_BIN = 1;    // value for normalizing 0th-order Color-CHLAC (with RGB binalize)
const float NORMALIZE_1_BIN = 1;    // value for normalizing 1st-order Color-CHLAC (with RGB binalize)
const float NORMALIZE_RI_0 = 1/255.0;    // value for normalizing 0th-order Color-CHLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1 = 1/845325.0; // value for normalizing 1st-order Color-CHLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_0_BIN = 1;    // value for normalizing 0th-order Color-CHLAC (with RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1_BIN = 1/13.0; // value for normalizing 1st-order Color-CHLAC (with RGB binalize) - rotation-invariant -
#endif

namespace pcl
{
  //* Mode for 90 degrees rotation
  enum RotateMode{ R_MODE_1, R_MODE_2, R_MODE_3, R_MODE_4 };

  //* calculate a feature vector when the target voxel data is rotated by 90 degrees.
  inline void rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode );

  template <typename PointT, typename PointOutT>
  class ColorCHLAC_RI_Estimation: public Feature<PointT, PointOutT>
    {
    public:
      using Feature<PointT, PointOutT>::feature_name_;
      //using Feature<PointT, PointOutT>::getClassName;
      using Feature<PointT, PointOutT>::indices_;
      using Feature<PointT, PointOutT>::surface_;
      //using Feature<PointT, PointOutT>::k_;
      //using Feature<PointT, PointOutT>::search_parameter_;

      typedef typename Feature<PointT, PointOutT>::PointCloudOut PointCloudOut;

/*       inline void getVoxelGrid ( const pcl::PointCloud<PointT> &cloud, pcl::PointCloud<PointT> &voxel_grid ); */
      
      inline void setColorThreshold ( int thR, int thG, int thB ){ color_thR = thR; color_thG = thG; color_thB = thB; }

      inline bool setVoxelFilter ( pcl::VoxelGrid<PointT> grid_, const int subdivision_size_, const int offset_x_, const int offset_y_, const int offset_z_, const float voxel_size_ );

      Eigen3::Vector3i getSubdivNum(){ return subdiv_b_; };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //ColorCHLAC_RI_Estimation () : voxel_size (0)
      ColorCHLAC_RI_Estimation () : voxel_size(0), hist_num (1), color_thR (-1), color_thG (-1), color_thB (-1)
      {
        feature_name_ = "ColorCHLAC_RI_Estimation";
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
      inline void setColor( int &r, int &g, int &b, int &r_, int &g_, int &b_ );

      //* functions for ColorCHLACSignature981 (rotation-variant) *//
      virtual inline void addColorCHLAC_0 ( const int idx, PointCloudOut &output );
      virtual inline void addColorCHLAC_0_bin ( const int idx, PointCloudOut &output );
      virtual inline void addColorCHLAC_1 ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b, const int r_, const int g_, const int b_ );
      virtual inline void addColorCHLAC_1_bin ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b );
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
      float voxel_size;
      pcl::VoxelGrid<PointT> grid;
      Eigen3::MatrixXi relative_coordinates;
      int hist_num;
      float inverse_subdivision_size;
      int offset_x;
      int offset_y;
      int offset_z;
      Eigen3::Vector3i div_b_;
      Eigen3::Vector3i min_b_;
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
  class ColorCHLACEstimation: public ColorCHLAC_RI_Estimation<PointT, PointOutT>
    {
    public:
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::feature_name_;
      //using ColorCHLAC_RI_Estimation<PointT, PointOutT>::getClassName;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::indices_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::surface_;
      //using ColorCHLAC_RI_Estimation<PointT, PointOutT>::k_;
      //using ColorCHLAC_RI_Estimation<PointT, PointOutT>::search_parameter_;

      typedef typename ColorCHLAC_RI_Estimation<PointT, PointOutT>::PointCloudOut PointCloudOut;

      //inline void setColorThreshold ( int thR, int thG, int thB );
      //inline bool setVoxelFilter ( pcl::VoxelGrid<PointT> grid_ );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //ColorCHLACEstimation () : voxel_size (0)
      ColorCHLACEstimation ()
      {
        feature_name_ = "ColorCHLACEstimation";
      };

    protected:

      //* functions for ColorCHLACSignature117 (rotation-invariant) *//
      inline void addColorCHLAC_0 ( const int idx, PointCloudOut &output );
      inline void addColorCHLAC_0_bin ( const int idx, PointCloudOut &output );
      inline void addColorCHLAC_1 ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b, const int r_, const int g_, const int b_ );
      inline void addColorCHLAC_1_bin ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b );
      inline void normalizeColorCHLAC ( PointCloudOut &output );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      void computeFeature (PointCloudOut &output);

    protected:
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::voxel_size;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::grid;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::relative_coordinates;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::hist_num;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::inverse_subdivision_size;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::offset_x;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::offset_y;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::offset_z;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::div_b_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::min_b_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::subdiv_b_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::subdivb_mul_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::color;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_r;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_g;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_b;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_r_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_g_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_b_;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_bin_r;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_bin_g;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::center_bin_b;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::color_thR;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::color_thG;
      using ColorCHLAC_RI_Estimation<PointT, PointOutT>::color_thB;
  };
}

#include "color_chlac.hpp"

#endif  //#ifndef PCL_COLOR_CHLAC_H_


