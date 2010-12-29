#include <iostream>
#include <stdio.h>
#include <string.h>

#include "color_voxel_recognition/Param.hpp"
#include "color_voxel_recognition/CCHLAC.hpp"

using namespace std;

//* 1ボクセルの辺の長さ を読み込む
float Param::readVoxelSize( const char* filename ){
  float val;
  if( readParam( filename, "voxel_size:", val ) ){
    if( val <= 0 ){
      cerr << "ERR (in Param::readVoxelSize): voxel_size must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readVoxelSize): fail" << endl;
  return -1;
}

//* 圧縮して得られるColor-CHLAC特徴ベクトルの次元 を読み込む
int Param::readDim( const char* filename ){
  int val;
  if( readParam( filename, "dim:", val ) ){
    const int dim_max = DIM_COLOR_1_3 + DIM_COLOR_BIN_1_3;
    if( ( val < 1 ) || ( val > dim_max ) ){
      cerr << "ERR (in Param::readDim): dim must be larger than 0 and less than " << dim_max << "." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readDim): fail" << endl;
  return -1;
}

//* 環境の分割領域の単位（1ボックスの1辺が何ボクセルであるか を読み込む）
int Param::readBoxSize_scene( const char* filename ){
  int val;
  if( readParam( filename, "box_size(scene):", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readBoxSize_scene): box_size must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readBoxSize_scene): fail" << endl;
  return -1;
}

//* 検出対象物体の分割領域の単位（1ボックスの1辺が何ボクセルであるか） を読み込む
int Param::readBoxSize_model( const char* filename ){
  int val;
  if( readParam( filename, "box_size(model):", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readBoxSize_model): box_size must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readBoxSize_model): fail" << endl;
  return -1;
}

//* 一度に作成するボクセル数の上限 を読み込む
int Param::readMaxVoxelNum( const char* filename ){
  int val;
  if( readParam( filename, "max_voxel_num:", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readMaxVoxelNum): max_voxel_num must be larger than 0." << endl;
      return -1;
    }
    else if( val < 10000000 )
      cerr << "Warning (in Param::readMaxVoxelNum): max_voxel_num is too small." << endl;
    return val;
  }
  cerr << "ERR (in Param::readMaxVoxelNum): fail" << endl;
  return -1;
}

//* 物体の学習における姿勢のバリエーション を読み込む
int Param::readRotateNum( const char* filename ){
  int val;
  if( readParam( filename, "rotate_num:", val ) ){
    if( val < 1 ){
      cerr << "ERR (in Param::readRotateNum): rotate_num must be larger than 0." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readRatateNum): fail" << endl;
  return -1;
}

//* ボクセルのRGB値の2値化の閾値 を読み込む
void Param::readColorThreshold( int &r, int &g, int &b, const char* filename ){
  FILE *fp = fopen( filename, "r" );
  fscanf(fp,"%d %d %d\n", &r, &g, &b);
  fclose(fp);
  if( ( r < 0 ) || ( r > 254 ) || ( g < 0 ) || ( g > 254 ) || ( b < 0 ) || ( b > 254 ) )
    cerr << "ERR (in Param::readColorThreshold): invalid RGB value." << endl;
}

//* 自動メッシュ化に必要な「SR自信度閾値」の読み込み
int Param::readConfTh( const char* filename ){
  int val;
  if( readParam( filename, "confidence_th:", val ) ){
    if( ( val < 0 ) || ( val > 255 ) ){
      cerr << "ERR (in Param::readConfTh): conf_th must be larger than -1 and less than 256." << endl;
      return -1;
    }
    return val;
  }
  cerr << "ERR (in Param::readConfTh): fail" << endl;
  return -1;
}

//* 自動メッシュ化に必要なパラメータたちの読み込み
void Param::readParamAuto( float& length_max_rate, float& length_th, float& distance_th, int& confidence_th, bool &relative_mode, const char* filename ){
  char line[ 100 ];
  FILE *fp = fopen( filename, "r" );
  int tmp;
  fscanf( fp, "%s %f\n", line, &length_max_rate );
  fscanf( fp, "%s %f\n", line, &length_th );
  fscanf( fp, "%s %f\n", line, &distance_th );
  fscanf( fp, "%s %d\n", line, &confidence_th );
  fscanf( fp, "%s %d\n", line, &tmp );
  relative_mode = tmp;
  fclose( fp );
}

//* 自動メッシュ化に必要なパラメータたちの書き込み
void Param::writeParamAuto( float length_max_rate, float length_th, float distance_th, int confidence_th, bool relative_mode, const char* filename ){
  FILE *fp = fopen( filename, "w" );
  fprintf( fp, "length_max_rate: %f\n", length_max_rate );
  fprintf( fp, "length_th:       %f\n", length_th );
  fprintf( fp, "distance_th:     %f\n", distance_th );
  fprintf( fp, "confidence_th:   %d\n", confidence_th );
  fprintf( fp, "relative_mode:   %d\n", relative_mode );
  fclose( fp );
}

//**************//
//* private関数 *//
//**************//

bool Param::readParam( const char* filename, const char *param_string, int &val ){
  char line[100];
  FILE *fp = fopen( filename, "r" );
  while( fscanf(fp,"%s %d",line,&val)!=EOF ){
    if( strcmp(line,param_string) == 0 ){
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  return false;
}

bool Param::readParam( const char* filename, const char *param_string, float &val ){
  char line[100];
  FILE *fp = fopen( filename, "r" );
  while( fscanf(fp,"%s %f",line,&val)!=EOF ){
    if( strcmp(line,param_string) == 0 ){
      fclose(fp);
      return true;
    }
  }
  fclose(fp);
  return false;
}
