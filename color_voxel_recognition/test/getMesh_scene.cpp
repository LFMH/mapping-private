#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>

#include <color_voxel_recognition/objFile.hpp>
#include <color_voxel_recognition/Param.hpp>
//#include "../param/CAM_SIZE"
#include "./CAM_SIZE"

/***************************************************/
/* 環境の計測データ（複数視点から取得したもの）をメッシュ化   */
/* 個々の視点の観測データが個々のファイルに保存される        */
/***************************************************/

using namespace std;

int main(int argc, char **argv)
{
  if( argc != 2 ){
    cerr<<"usage: "<<argv[0]<<" <registration_num>"<<endl;
    exit( EXIT_FAILURE );
  }
  const int obj_num = atoi( argv[1] );  // 書き込むメッシュデータの数

  //* SRとFlea2間のキャリブのパラメータ 読み込み
  float resize_rate, aspect_rate;
  int shiftX, shiftY, dis_1_pixel;
  char line[ 100 ];
  FILE *fp_param = fopen( "param/SR_Flea2_calib.txt", "r" );
  fscanf( fp_param, "%s %f\n", line, &resize_rate );
  fscanf( fp_param, "%s %f\n", line, &aspect_rate );
  fscanf( fp_param, "%s %d\n", line, &shiftX );
  fscanf( fp_param, "%s %d\n", line, &shiftY );
  fscanf( fp_param, "%s %d\n", line, &dis_1_pixel );
  fclose( fp_param );

  //* メッシュ作成のパラメータ 読み込み
  float length_max_rate;
  float length_th;
  float distance_th;
  int confidence_th;
  bool tmp;
  Param::readParamAuto( length_max_rate, length_th, distance_th, confidence_th, tmp, "scene/param.txt" );

  char tmpname1[ 100 ];
  char tmpname2[ 100 ];
  char tmpname3[ 100 ];
  char tmpname4[ 100 ];
  char tmpname5[ 100 ];
  char obj_filename[ 100 ];
  Obj object;
  for( int i=0; i<obj_num; i++ ){
    //* SRデータ 読み込み
    sprintf(tmpname1, "scene/X/%03d.dat",i );
    sprintf(tmpname2, "scene/Y/%03d.dat",i );
    sprintf(tmpname3, "scene/Z/%03d.dat",i );
    sprintf(tmpname4, "scene/C/%03d.dat",i );
    sprintf(tmpname5, "scene/Camera/%03d.dat",i );
    sprintf(obj_filename, "scene/Obj/%03d.obj",i );
    object.readPoints( tmpname1, tmpname2, tmpname3, tmpname4, SWIDTH, SHEIGHT, CWIDTH, CHEIGHT, resize_rate, aspect_rate, shiftX, shiftY, dis_1_pixel );

    //* メッシュ作成、グローバル座標への変換、書き込み
    object.getMesh( length_max_rate, length_th, distance_th, confidence_th );
    object.transMesh( tmpname5, METER_MODE );
    object.writeMesh( obj_filename );
    object.deleteData();
  }

  return(0);
}
