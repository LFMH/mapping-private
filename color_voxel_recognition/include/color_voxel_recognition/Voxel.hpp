#ifndef MY_VOXEL_HPP
#define MY_VOXEL_HPP

#ifdef USE_OPENCV
#include <cv.h>
#endif

#ifdef USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

#include <stdio.h>
#include "objFile.hpp"

/********************************************************/
/* Voxelの作成/読み込みを行うクラス                          */
/* 特に読み込み時にはColor-CHLAC特徴抽出向けに6次元の色表現にする */
/*  OpenCVの配列を使うときは #define USE_OPENCV する        */
/*   ファイルフォーマット：                                 */
/*      xsize ysize zsize                               */
/*      x y z r g b                                     */
/*      x y z r g b                                     */
/*           .                                          */
/*           .                                          */
/*                                                      */
/********************************************************/

//* RGB値の補助色成分の定義
enum ReverseMode{ SIMPLE_REVERSE, TRIGONOMETRIC };

class Voxel{
public:
  Voxel();
  Voxel( bool _ascii ); // ボクセル読み書き用のファイルがバイナリかどうかを指定
  Voxel( const char *filename, const char *mode ); // ボクセル読み書き用のファイルとその開くモードを指定
  Voxel( const Voxel &another );
  ~Voxel();

  //***********************//
  //* ボクセル作成時に使う関数 *//
  //***********************//

  //* ボクセルの一辺の長さを指定
  void setVoxelSize( float val );

  //* ボクセル化対象データの三次元座表の最大値、最小値を指定
  void setMinMax( float _x_min, float _x_max, float _y_min, float _y_max, float _z_min, float _z_max );

  //* メッシュデータのボクセル化 RGB2値化せず
  void obj2voxel( Obj &object, const char *image_filename, ReverseMode mode );

  //* メッシュデータのボクセル化 RGB2値化する
  void obj2voxel( Obj &object, const char *image_filename, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );

  //* メッシュデータのボクセル化 ファイルに出力
  void obj2voxel_file( Obj &object, const char *image_filename );

#ifdef USE_PCL
  //* binarize RGB values
  void binarize( unsigned char thR, unsigned char thG, unsigned char thB );

  //* set the minimum and maximum xyz valuse of 3D data
  void setMinMax();

  //* transform point cloud into voxel data
  void points2voxel( pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster, ReverseMode mode );
  void points2voxel( pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_object_cluster, ReverseMode mode );

  //* ファイルに出力
  void writeVoxel();
#endif

#ifdef USE_OPENCV
  //* メッシュデータのボクセル化 画像配列はOpenCVのもの RGB2値化せず
  void obj2voxel( Obj &object, const cv::Mat& color_img, ReverseMode mode );

  //* メッシュデータのボクセル化 画像配列はOpenCVのもの RGB2値化する
  void obj2voxel( Obj &object, const cv::Mat& color_img, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );
#endif

  //**************************//
  //* ボクセル読み込み時に使う関数 *//
  //**************************//

  //* ボクセルのx軸、y軸、z軸上の個数を指定
  void setXYZsize( int _xsize, int _ysize, int _zsize );

  //* ボクセルのメモリ確保
  void createVoxelData();

  //* ボクセルの値を0で埋める
  void cleanVoxelData();

  //* 最大最小値を記録 (ただしexist_flagに値がちゃんと入っていることを仮定)
  void getMinMax( int &x_min_i, int &x_max_i, int &y_min_i, int &y_max_i, int &z_min_i, int &z_max_i );

  //* 補助成分値を計算しないボクセルの読み込み
  void readVoxel_normal( const char *filename );

  //* RGB二値化しないボクセルの読み込み （インスタンス宣言時にファイル名を指定する場合）
  void readVoxel( ReverseMode mode );

  //* RGB二値化するボクセルの読み込み  （インスタンス宣言時にファイル名を指定する場合）
  void readVoxel( unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );

  //* RGB二値化しないボクセルの読み込み （インスタンス宣言時にファイル名を指定しない場合）
  //* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
  void readVoxel( const char *filename, ReverseMode mode, bool size_read_flg = true );

  //* RGB二値化するボクセルの読み込み  （インスタンス宣言時にファイル名を指定しない場合）
  //* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
  void readVoxel( const char *filename, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b, bool size_read_flg = true );

  //* RGB二値化しないボクセルの読み込み （z方向にオフセット分ずらし、zsize以上のボクセル値を無視）
  //* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
  void readVoxelZoffset( const char *filename, int zoffset, ReverseMode mode, bool size_read_flg = true );

  //* RGB二値化するボクセルの読み込み  （z方向にオフセット分ずらし、zsize以上のボクセル値を無視）
  //* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
  void readVoxelZoffset( const char *filename, int zoffset, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b, bool size_read_flg = true );


  //***************//
  //* 各種変数の取得 *//
  //***************//

  const int Xsize() const { return xsize; }
  const int Ysize() const { return ysize; }
  const int Zsize() const { return zsize; }
  const unsigned char* Vr() const { return vr; }
  const unsigned char* Vg() const { return vg; }
  const unsigned char* Vb() const { return vb; }
  const unsigned char* _Vr() const { return _vr; }
  const unsigned char* _Vg() const { return _vg; }
  const unsigned char* _Vb() const { return _vb; }
  const bool* Exist() const { return exist_flag; }
  const bool exist( int x, int y, int z ) const { return exist_flag[ x + y*xsize + z*xysize ]; }
  const unsigned char red( int x, int y, int z ) const { return vr[ x + y*xsize + z*xysize ]; }
  const unsigned char green( int x, int y, int z ) const { return vg[ x + y*xsize + z*xysize ]; }
  const unsigned char blue( int x, int y, int z ) const { return vb[ x + y*xsize + z*xysize ]; }

  const Voxel& operator=(const Voxel &another);

private:
  bool ascii; // ボクセル読み書き用のファイルがバイナリかどうか
  FILE *fp_r;  // ボクセル読み込み用のファイル用
  FILE *fp_w;  // ボクセル書き込み用のファイル用
  bool *exist_flag;   // ボクセルが物体に占有されているか否かの情報
  int *exist_num;   // Number of points in a voxel
  int xsize;   // ボクセルのx軸上の個数
  int ysize;   // ボクセルのy軸上の個数
  int zsize;   // ボクセルのz軸上の個数
  int xysize;  // xsize * ysize
  int xyzsize; // xsize * ysize * zsize
  float x_min; // ボクセル化対象データの x座標の最小値
  float x_max; // ボクセル化対象データの x座標の最大値
  float y_min; // ボクセル化対象データの y座標の最小値
  float y_max; // ボクセル化対象データの y座標の最大値
  float z_min; // ボクセル化対象データの z座標の最小値
  float z_max; // ボクセル化対象データの z座標の最大値
  unsigned char *vr;  // R の主成分値
  unsigned char *vg;  // G の主成分値
  unsigned char *vb;  // B の主成分値
  unsigned char *_vr; // R の補助成分値
  unsigned char *_vg; // G の補助成分値
  unsigned char *_vb; // B の補助成分値
  float voxel_size; // ボクセルの一辺の長さ

  //* RGB の主成分値と補助成分値をセットする （二値化しない）
  //* 注 RreverseMode で処理内容が変わる
  void setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, ReverseMode mode );

  //* RGB の主成分値と補助成分値をセットする （二値化する）
  void setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );

  //* 各メッシュのボクセル化 ファイルに出力
  void face2voxel_file( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step );

  //* 各メッシュのボクセル化 RGB2値化せず
  void face2voxel( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step, ReverseMode mode );

  //* 各メッシュのボクセル化 RGB2値化する
  void face2voxel( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );
};

#endif
