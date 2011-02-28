#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>

#include <color_voxel_recognition/objFile.hpp>
#include <color_voxel_recognition/Param.hpp>
//#include "../param/CAM_SIZE"
#include "./CAM_SIZE"

/****************************************************************************/
/* 検出対象物体の計測データ（複数方向から取得したもの）をメッシュ化                      */
/* 各メッシュデータは別個のファイルに保存される                                      */
/* RELATIVE MODEの場合、最も近い点からDISTANCE_TH(m)の奥行きまでのみをメッシュ化する   */
/* そうでなければ、Mask/以下のファイルを見て、全体データの中から一部を抽出してメッシュ化する */
/****************************************************************************/

using namespace std;

int main(int argc, char **argv)
{
  if( argc != 3 ){
    cerr<<"usage: "<<argv[0]<<" [model_name] <registration_num>"<<endl;
    exit( EXIT_FAILURE );
  }
  const int obj_num = atoi( argv[2] );  // 書き込むメッシュデータの数

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
  bool relative_mode;
  char tmpname[ 100 ];
  sprintf( tmpname, "models/%s/param.txt",argv[1] );
  Param::readParamAuto( length_max_rate, length_th, distance_th, confidence_th, relative_mode, tmpname );

  char tmpname1[ 100 ];
  char tmpname2[ 100 ];
  char tmpname3[ 100 ];
  char tmpname4[ 100 ];
  char tmpname5[ 100 ];
  char obj_filename[ 100 ];
  Obj object;
  for( int i=0; i<obj_num; i++ ){
    //* SRデータ 読み込み
    sprintf(tmpname1, "models/%s/X/%03d.dat",argv[1],i );
    sprintf(tmpname2, "models/%s/Y/%03d.dat",argv[1],i );
    sprintf(tmpname3, "models/%s/Z/%03d.dat",argv[1],i );
    sprintf(tmpname4, "models/%s/C/%03d.dat",argv[1],i );
    sprintf(tmpname5, "models/%s/Mask/%03d.dat",argv[1],i );
    sprintf(obj_filename, "models/%s/Obj/%03d.obj",argv[1],i );
    object.readPoints( tmpname1, tmpname2, tmpname3, tmpname4, SWIDTH, SHEIGHT, CWIDTH, CHEIGHT, resize_rate, aspect_rate, shiftX, shiftY, dis_1_pixel );

    //* メッシュ作成、書き込み
    if( relative_mode ){
      float dis_min = FLT_MAX;
      for (int v=0; v<object.v_num; v++)
	if( ( object.confidences[ v ] > confidence_th ) && ( dis_min > object.vertices[ v ].z ) ) dis_min = object.vertices[ v ].z;
      object.getMesh( length_max_rate, length_th, dis_min + distance_th, confidence_th );
    }
    else
      object.getMesh( tmpname5, length_max_rate, length_th, distance_th, confidence_th );
    object.writeMesh( obj_filename );
    object.deleteData();
  }

  return(0);
}
