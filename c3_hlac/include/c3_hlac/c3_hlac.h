#ifndef PCL_C3_HLAC_H_
#define PCL_C3_HLAC_H_

#include <pcl/point_types.h>
#include <pcl/features/feature.h>
#include <pcl/filters/voxel_grid.h>

const float angle_norm = M_PI / 510;

const int DIM_C3HLAC_1_3 = 495;        // Dimension of feature vector (without RGB binalize)
const int DIM_C3HLAC_BIN_1_3 = 486;    // Dimension of feature vector (with RGB binalize)
const int DIM_C3HLAC_1_3_ALL = 981;    // = DIM_C3HLAC_1_3 + DIM_C3HLAC_BIN_1_3
const int DIM_C3HLAC_RI_1_3 = 63;      // Dimension of feature vector (without RGB binalize) - rotation-invariant -
const int DIM_C3HLAC_RI_BIN_1_3 = 54;  // Dimension of feature vector (with RGB binalize) - rotation-invariant -
const int DIM_C3HLAC_RI_1_3_ALL = 117; // = DIM_C3HLAC_RI_1_3 + DIM_C3HLAC_RI_BIN_1_3

const float NORMALIZE_0 = 1/255.0;    // value for normalizing 0th-order C3_HLAC (without RGB binalize)
const float NORMALIZE_1 = 1/65025.0;  // value for normalizing 1st-order C3_HLAC (without RGB binalize)
const float NORMALIZE_0_BIN = 1;    // value for normalizing 0th-order C3_HLAC (with RGB binalize)
const float NORMALIZE_1_BIN = 1;    // value for normalizing 1st-order C3_HLAC (with RGB binalize)
const float NORMALIZE_RI_0 = 1/255.0;    // value for normalizing 0th-order C3_HLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1 = 1/845325.0; // value for normalizing 1st-order C3_HLAC (without RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_0_BIN = 1;    // value for normalizing 0th-order C3_HLAC (with RGB binalize) - rotation-invariant -
const float NORMALIZE_RI_1_BIN = 1/13.0; // value for normalizing 1st-order C3_HLAC (with RGB binalize) - rotation-invariant -

namespace pcl{
  struct C3_HLAC_Signature981
  {
    float histogram[981];
  };
  inline std::ostream& operator << (std::ostream& os, const C3_HLAC_Signature981& p)
  {
    for (int i = 0; i < 981; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 980 ? ", " : ")");
    return (os);
  }

  struct C3_HLAC_Signature117
  {
    float histogram[117];
  };
  inline std::ostream& operator << (std::ostream& os, const C3_HLAC_Signature117& p)
  {
    for (int i = 0; i < 117; ++i)
      os << (i == 0 ? "(" : "") << p.histogram[i] << (i < 116 ? ", " : ")");
    return (os);
  }

  //* Mode for 90 degrees rotation
  enum RotateMode{ R_MODE_1, R_MODE_2, R_MODE_3, R_MODE_4 };

  //* calculate a feature vector when the target voxel data is rotated by 90 degrees.
  void rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode );

  template <typename PointT, typename PointOutT>
  class C3_HLAC_RI_Estimation: public Feature<PointT, PointOutT>
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

      bool setVoxelFilter ( pcl::VoxelGrid<PointT> grid_, const int subdivision_size_, const int offset_x_, const int offset_y_, const int offset_z_, const float voxel_size_ );

      Eigen::Vector3i getSubdivNum(){ return subdiv_b_; };

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //C3_HLAC_RI_Estimation () : voxel_size (0)
      C3_HLAC_RI_Estimation () : voxel_size(0), hist_num (1), color_thR (-1), color_thG (-1), color_thB (-1)
      {
        feature_name_ = "C3_HLAC_RI_Estimation";
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
      inline int binarize_r ( int val ){
	if( val > color_thR ) return 1;
	return 0;
      }
      inline int binarize_g ( int val ){
	if( val > color_thG ) return 1;
	return 0;
      }
      inline int binarize_b ( int val ){
	if( val > color_thB ) return 1;
	return 0;
      }

      virtual inline void setColor( int &r, int &g, int &b, int &r_, int &g_, int &b_ ){
	const int val1 = r;
	const int val2 = g;
	const int val3 = b;
	r  = 255 * sin( val1 * angle_norm );
	g  = 255 * sin( val2 * angle_norm );
	b  = 255 * sin( val3 * angle_norm );
	r_ = 255 * cos( val1 * angle_norm );
	g_ = 255 * cos( val2 * angle_norm );
	b_ = 255 * cos( val3 * angle_norm );
      }

      virtual void addC3_HLAC_0 ( const int idx, PointCloudOut &output );
      virtual void addC3_HLAC_0_bin ( const int idx, PointCloudOut &output );
      virtual void addC3_HLAC_1 ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b, const int r_, const int g_, const int b_ );
      virtual void addC3_HLAC_1_bin ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b );
      virtual void computeC3_HLAC ( const pcl::PointCloud<PointT> &cloud, PointCloudOut &output, const int center_idx );
      virtual void normalizeC3_HLAC ( PointCloudOut &output );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      virtual void computeFeature (PointCloudOut &output);

    protected:
      float voxel_size;
      pcl::VoxelGrid<PointT> grid;
      Eigen::MatrixXi relative_coordinates;
      int hist_num;
      float inverse_subdivision_size;
      int offset_x;
      int offset_y;
      int offset_z;
      Eigen::Vector3i div_b_;
      Eigen::Vector3i min_b_;
      Eigen::Vector3i subdiv_b_;
      Eigen::Vector3i subdivb_mul_;
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
  class C3_HLAC_Estimation: public C3_HLAC_RI_Estimation<PointT, PointOutT>
    {
    public:
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::feature_name_;
      //using C3_HLAC_RI_Estimation<PointT, PointOutT>::getClassName;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::indices_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::surface_;
      //using C3_HLAC_RI_Estimation<PointT, PointOutT>::k_;
      //using C3_HLAC_RI_Estimation<PointT, PointOutT>::search_parameter_;

      typedef typename C3_HLAC_RI_Estimation<PointT, PointOutT>::PointCloudOut PointCloudOut;

      //inline void setColorThreshold ( int thR, int thG, int thB );
      //inline bool setVoxelFilter ( pcl::VoxelGrid<PointT> grid_ );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //C3_HLAC_Estimation () : voxel_size (0)
      C3_HLAC_Estimation ()
      {
        feature_name_ = "C3_HLAC_Estimation";
      };

    protected:

      //* functions for C3_HLAC_Signature117 (rotation-invariant) *//
      virtual void addC3_HLAC_0 ( const int idx, PointCloudOut &output );
      virtual void addC3_HLAC_0_bin ( const int idx, PointCloudOut &output );
      virtual void addC3_HLAC_1 ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b, const int r_, const int g_, const int b_ );
      virtual void addC3_HLAC_1_bin ( const int idx, PointCloudOut &output, const int neighbor_idx, const int r, const int g, const int b );
      virtual void normalizeC3_HLAC ( PointCloudOut &output );

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Estimate normals for all points given in <setInputCloud (), setIndices ()> using the surface in
        * setSearchSurface () and the spatial locator in setSearchMethod ()
        * \note In situations where not enough neighbors are found, the normal and curvature values are set to -1.
        * \param output the resultant point cloud model dataset that contains surface normals and curvatures
        */
      virtual void computeFeature (PointCloudOut &output);

    protected:
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::voxel_size;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::grid;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::relative_coordinates;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::hist_num;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::inverse_subdivision_size;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::offset_x;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::offset_y;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::offset_z;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::div_b_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::min_b_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::subdiv_b_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::subdivb_mul_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::color;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_r;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_g;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_b;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_r_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_g_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_b_;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_bin_r;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_bin_g;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::center_bin_b;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::color_thR;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::color_thG;
      using C3_HLAC_RI_Estimation<PointT, PointOutT>::color_thB;
  };
}

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::C3_HLAC_Signature981,
                                   (float[981], histogram, c3_hlac));

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::C3_HLAC_Signature117,
                                   (float[117], histogram, c3_hlac_RI));

#endif  //#ifndef PCL_C3_HLAC_H_


