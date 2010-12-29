#ifndef MY_CCHLAC_HPP
#define MY_CCHLAC_HPP

#include <octave/config.h>
#include <octave/Matrix.h>
#include "Voxel.hpp"

/*****************************************************/
/* Color-CHLAC特徴をとるクラス                          */
/* マスク幅 3x3 高々1次 のみを実装                        */
/* 前処理としてRGB値を2値化してある場合としてない場合とで異なる */
/*****************************************************/

const int DIM_COLOR_1_3 = 495;     // RGB値を2値化しない場合の マスク幅3x3 高々1次のColor-CHLAC特徴ベクトルの次元数
const int DIM_COLOR_BIN_1_3 = 486; // RGB値を2値化する場合の  マスク幅3x3 高々1次のColor-CHLAC特徴ベクトルの次元数

//* 回転モード
//* 90度回転させる軸と向きによって4通り
enum RotateMode{ R_MODE_1, R_MODE_2, R_MODE_3, R_MODE_4 };

class CCHLAC{
public:
  //* RGB値を2値化しない場合の特徴抽出
  static void extractColorCHLAC( ColumnVector &result, Voxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* RGB値を2値化する場合の特徴抽出
  static void extractColorCHLAC_bin( ColumnVector &result, Voxel &voxel, int sx, int sy, int sz, int gx, int gy, int gz );
  
  //* ボクセルを90度回転したときの特徴量を計算
  static void rotateFeature90( ColumnVector &output, const ColumnVector &input, RotateMode mode );

private:  
  static void extractColorCHLAC( ColumnVector &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );
  static void extractColorCHLAC_bin( ColumnVector &result, const unsigned char *red, const unsigned char *nred, const unsigned char *green, const unsigned char *ngreen, const unsigned char *blue, const unsigned char *nblue, int sx, int sy, int sz, int gx, int gy, int gz, int rx, int ry, int rz, int xsize, int ysize, int zsize );

  // no constructor
  CCHLAC();
};

#endif
