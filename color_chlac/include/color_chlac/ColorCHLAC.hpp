/**
Copyright (C) 2010 by Asako Kanezaki
Disclaimer:
This code cannot be used for a commercial enterprise.
Note that this code is under patent application.
**/
#ifndef MY_COLOR_CHLAC_HPP
#define MY_COLOR_CHLAC_HPP

#include <octave/config.h>
#include <octave/Matrix.h>
#include "Voxel.hpp"

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
  //* feature extraction (without RGB binalize)
  static void extractColorCHLAC( ColumnVector &result, Voxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* feature extraction (with RGB binalize)
  static void extractColorCHLAC_bin( ColumnVector &result, Voxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* calculate a feature vector when the target voxel data is rotated by 90 degrees.
  static void rotateFeature90( ColumnVector &output, const ColumnVector &input, RotateMode mode );

private:  
  static void extractColorCHLAC( ColumnVector &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );
  static void extractColorCHLAC_bin( ColumnVector &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );

  // no constructor
  ColorCHLAC();
};

#endif
