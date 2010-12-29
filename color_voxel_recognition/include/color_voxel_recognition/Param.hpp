#ifndef MY_PARAM_HPP
#define MY_PARAM_HPP

/********************************************/
/* パラメータの読み込み（と一部書き込み）をするクラス */
/********************************************/

class Param{
public:

  //* 1ボクセルの辺の長さ を読み込む
  static float readVoxelSize( const char* filename = "param/parameters.txt" );

  //* 圧縮して得られるColor-CHLAC特徴ベクトルの次元 を読み込む
  static int readDim( const char* filename = "param/parameters.txt" );

  //* 環境の分割領域の単位（1ボックスの1辺が何ボクセルであるか を読み込む）
  static int readBoxSize_scene( const char* filename = "param/parameters.txt" );

  //* 検出対象物体の分割領域の単位（1ボックスの1辺が何ボクセルであるか） を読み込む
  static int readBoxSize_model( const char* filename = "param/parameters.txt" );

  //* 一度に作成するボクセル数の上限 を読み込む
  static int readMaxVoxelNum( const char* filename = "param/parameters.txt" );

  //* 物体の学習における姿勢のバリエーション を読み込む
  static int readRotateNum( const char* filename = "param/parameters.txt" );

  //* ボクセルのRGB値の2値化の閾値 を読み込む
  static void readColorThreshold( int &r, int &g, int &b, const char* filename = "param/color_threshold.txt" );

  //* 自動メッシュ化に必要な「SR自信度閾値」の読み込み
  static int readConfTh( const char* filename );

  //* 自動メッシュ化に必要なパラメータたちの読み込み
  static void readParamAuto( float& length_max_rate, float& length_th, float& distance_th, int& confidence_th, bool &relative_mode, const char* filename );

  //* 自動メッシュ化に必要なパラメータたちの書き込み
  static void writeParamAuto( float length_max_rate, float length_th, float distance_th, int confidence_th, bool relative_mode, const char* filename );

private:
  static bool readParam( const char* filename, const char *param_string, int &val );
  static bool readParam( const char* filename, const char *param_string, float &val );

  // no constructor
  Param();
};

#endif
