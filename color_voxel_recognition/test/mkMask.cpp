#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>

#include <color_voxel_recognition/objFile.hpp>
#include <color_voxel_recognition/ppmFile.hpp>
#include <color_voxel_recognition/Param.hpp>
//#include "../param/CAM_SIZE"
#include "./CAM_SIZE"

/*****************************************************************/
/* カラー画像と同じ大きさの、物体抽出範囲のマスク画像（MaskImg/以下）に対して */
/* SR（スイスレンジャ）座標への変換を行い、                             */
/* 結果を保存する（Mask/以下）                                       */
/*****************************************************************/

using namespace std;

// 距離zに応じたx方向シフト量を計算
int calc_shiftx_add( int x, int y, const float* z_data, const float resize_rate, const float aspect_rate, const int shiftX, const int dis_1_pixel ) {
  int shiftXadd = 0;
  if( z_data[ x + y * SWIDTH ] < 0.0001 )
    shiftXadd = 0;
  else {
    shiftXadd = (int)( dis_1_pixel / z_data[ x + y * SWIDTH ] );
    while(1) {
      if( ( x + shiftX + shiftXadd )/( resize_rate * aspect_rate ) < CWIDTH )
        break;
      shiftXadd--;
    }
  }
  return shiftXadd;
}

// カラー画像座標からSR座標への変換
void msk2sr( int pos_x, int pos_y, int &sr_x, int &sr_y, const float* z_data, const unsigned char* c_data, const float resize_rate, const float aspect_rate, const int shiftX, const int shiftY, const int dis_1_pixel, const unsigned char conf_th ){
  int shiftXadd;
  sr_y = pos_y * resize_rate - shiftY;
  sr_x = 0;
  int min_val = 10000;
  for (int i=0; i<SWIDTH; i++){
    if( c_data[ i + sr_y * SWIDTH ] > conf_th ){
      shiftXadd = calc_shiftx_add( i, sr_y, z_data, resize_rate, aspect_rate, shiftX, dis_1_pixel );
      int hoge = abs( pos_x - ( i + shiftX + shiftXadd )/ ( resize_rate*aspect_rate ) );
      if( hoge < min_val ){
	min_val = hoge;
	sr_x = i;
      }
    }
  }
}

int main(int argc, char **argv)
{
  if( argc != 3 ){
    cerr<<"usage: "<<argv[0]<<" [model_name] <registration_num>"<<endl;
    exit( EXIT_FAILURE );
  }
  const int obj_num = atoi( argv[2] );  // 書き込むメッシュデータの数
  char tmpname[ 100 ];

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
  sprintf( tmpname, "models/%s/param.txt",argv[1] );
  int conf_th = Param::readConfTh( tmpname );

  char msk_filename[ 100 ];
  FILE *fpZ, *fpC, *fpM;
  float *z_data = new float[ SWIDTH*SHEIGHT ];
  unsigned char *c_data = new unsigned char[ SWIDTH*SHEIGHT ];
  unsigned char *delete_mask = new unsigned char[ SWIDTH*SHEIGHT ];
  Ppm ppm;
  for( int i=0; i<obj_num; i++ ){
    //* SRデータ 読み込み
    sprintf(tmpname, "models/%s/Z/%03d.dat",argv[1],i );
    fpZ = fopen( tmpname, "rb" );
    sprintf(tmpname, "models/%s/C/%03d.dat",argv[1],i );
    fpC = fopen( tmpname, "rb" );
    for (int y=0; y<SHEIGHT; y++){
      for (int x=0; x<SWIDTH; x++){
        fread( z_data + y * SWIDTH + x, sizeof(float), 1, fpZ );
        fread( c_data + y * SWIDTH + x, sizeof(unsigned char), 1, fpC );
	delete_mask[ x + y * SWIDTH ] = 0;
      }
    }
    fclose(fpZ);
    fclose(fpC);
    sprintf(tmpname, "models/%s/MaskImg/%03d.ppm",argv[1],i );
    ppm.read( tmpname );

    //* カラー画像座標からSR座標への変換
    int sr_x, sr_y;
    for (int y=0; y<CHEIGHT; y++){
      for (int x=0; x<CWIDTH; x++){
	if( ppm.ImageData()[ 3 * ( x + y * CWIDTH ) ] == 0 ) {
	  msk2sr( x, y, sr_x, sr_y, z_data, c_data, resize_rate, aspect_rate, shiftX, shiftY, dis_1_pixel, conf_th );
	  delete_mask[ sr_x + sr_y * SWIDTH ] = 255;
	  if( (sr_x>1) && (delete_mask[ sr_x-2 + sr_y * SWIDTH ]==255)&& (delete_mask[ sr_x-1 + sr_y * SWIDTH ]==0) )
	    delete_mask[ sr_x-1 + sr_y * SWIDTH ] = 255;
	}
      }
    }

    //* 書き込み
    sprintf(msk_filename, "models/%s/Mask/%03d.dat",argv[1],i );
    fpM = fopen( msk_filename, "wb" );
    for (int y=0; y<SHEIGHT; y++)
      for (int x=0; x<SWIDTH; x++)
	fwrite( &( delete_mask[ x + y * SWIDTH ] ), sizeof(unsigned char), 1, fpM );
    fclose( fpM );
  }

  return(0);
}
