#ifndef MY_COLOR_CHLAC_HPP
#define MY_COLOR_CHLAC_HPP

#include <pcl_cloud_algos/pcl_cloud_algos_point_types.h>
#include <pcl/features/feature.h>
#include <iostream>
#include <stdlib.h>
#include <vector>
#include "ColorVoxel.hpp"

/***********************************/
/* Color-CHLAC features extraction */
/* mask region: 3x3                */
/*  order: 0th and 1st             */
/***********************************/

const int DIM_COLOR_1_3 = 495;     // Dimension of feature vector (without RGB binalize)
const int DIM_COLOR_BIN_1_3 = 486; // Dimension of feature vector (with RGB binalize)

//* Mode for 90 degrees rotation
enum RotateMode{ R_MODE_1, R_MODE_2, R_MODE_3, R_MODE_4 };

class ColorCHLAC{
public:
  //* feature extraction (both RGB and RGB-binalized. 981(=495+486) dimension.)
  static void extractColorCHLAC981( pcl::PointCloud<pcl::ColorCHLACSignature981> &result, ColorVoxel &voxel, const unsigned char thR, const unsigned char thG, const unsigned char thB );

  //* feature extraction (both RGB and RGB-binalized. 981(=495+486) dimension.)
  static void extractColorCHLAC981( std::vector<float> &result, ColorVoxel &voxel, const unsigned char thR, const unsigned char thG, const unsigned char thB );

  //* feature extraction (without RGB binalize)
  static void extractColorCHLAC( std::vector<float> &result, ColorVoxel &voxel );
  static void extractColorCHLAC( std::vector<float> &result, ColorVoxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* feature extraction (with RGB binalize)
  static void extractColorCHLAC_bin( std::vector<float> &result, ColorVoxel &voxel );
  static void extractColorCHLAC_bin( std::vector<float> &result, ColorVoxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* calculate a feature vector when the target voxel data is rotated by 90 degrees.
  static void rotateFeature90( std::vector<float> &output, const std::vector<float> &input, RotateMode mode );

private:  
  static void extractColorCHLAC( std::vector<float> &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );
  static void extractColorCHLAC_bin( std::vector<float> &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );

  // no constructor
  ColorCHLAC();
};

#endif
