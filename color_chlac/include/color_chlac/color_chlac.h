#ifndef PCL_COLOR_CHLAC_H_
#define PCL_COLOR_CHLAC_H_

#include <pcl/features/feature.h>
#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>

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

      //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      /** \brief Empty constructor. */
      //ColorCHLACEstimation () : voxel_size (0)
      ColorCHLACEstimation () : color_thR (-1), color_thG (-1), color_thB (-1)
      {
        feature_name_ = "ColorCHLACEstimation";
      };

    protected:
      inline void computeColorCHLAC (const pcl::PointCloud<PointXYZRGB> &cloud, const std::vector<int> &indices, PointCloudOut &output, const int center_idx );
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

      //float voxel_size;
/*       /\** \brief 16-bytes aligned placeholder for the XYZ centroid of a surface patch. *\/ */
/*       Eigen3::Vector4f xyz_centroid_; */
  };
}

#include "color_chlac.hpp"

#endif  //#ifndef PCL_COLOR_CHLAC_H_


