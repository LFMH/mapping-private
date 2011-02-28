#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include <cv.h>
#include <highgui.h>

#include "pcl/io/pcd_io.h"
#include "pcl/point_types.h"
#include <color_voxel_recognition/Param.hpp>
//#include "../param/CAM_SIZE"
#include "./CAM_SIZE"

/****************************************************************************/
/* 検出対象物体の計測データ（複数方向から取得したもの）をcolor point cloud化                      */
/* 各データは別個のファイルに保存される                                      */
/* RELATIVE MODEの場合、最も近い点からDISTANCE_TH(m)の奥行きまでのみをcolor point cloud化する   */
/****************************************************************************/

//***************
//* モードの選択
#define BINARY_MODE // SRデータをバイナリで読み込み
#ifdef BINARY_MODE
#define READ_MODE "rb"
#else
#define READ_MODE "r"
#endif
#define RELATIVE_MODE true

using namespace std;
using namespace pcl;

const unsigned char CONF_TH = 200;
const float DIS_DEPTH = 0.3;

int shiftX, shiftY, dis_1_pixel;
float resize_rate, aspect_rate;

int getShiftX( int i, float z, int image_width ){
  int shiftXadd;
  if( z == 0 ) shiftXadd = 0;
  else{
    shiftXadd = (int)( dis_1_pixel / z );
    while(1){
      if( i + shiftXadd + shiftX < image_width ) break;
      shiftXadd--;
    }
  }
  //cout << z << " " << image_width << " " << shiftXadd << endl;
  return shiftXadd + shiftX;
}

const float setPointRGB( const int r, const int g, const int b ){
  //cout << r << " " << g << " " << b << endl;
  int rgb = r << 16 | g << 8 | b;
  return *reinterpret_cast<float*>(&rgb);
}

void readPointsSR( pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat image, int image_width, const char* filename1, const char* filename2, const char* filename3, const char* filename4 ){
  FILE *fpX = fopen( filename1, READ_MODE );
  FILE *fpY = fopen( filename2, READ_MODE );
  FILE *fpZ = fopen( filename3, READ_MODE );
  FILE *fpC = fopen( filename4, READ_MODE );

  int image_x, image_y;
  int idx = 0;
  for( int j=0; j<SHEIGHT; j++ ){
    for( int i=0; i<SWIDTH; i++ ){
#ifdef BINARY_MODE
      fread( &(cloud.points[idx].x), sizeof(float), 1, fpX );
      fread( &(cloud.points[idx].y), sizeof(float), 1, fpY );
      fread( &(cloud.points[idx].z), sizeof(float), 1, fpZ );
      unsigned char conf;
      fread( &conf, sizeof(unsigned char), 1, fpC );
#else
      fscanf( fpX, "%f ", &(cloud.points[idx].x) );
      fscanf( fpY, "%f ", &(cloud.points[idx].y) );
      fscanf( fpZ, "%f ", &(cloud.points[idx].z) );
      int conf;
      fscanf( fpC, "%d ", &conf );
#endif
      if( conf > CONF_TH ){ // only confident points
	image_x = (int)(( i + getShiftX( i, cloud.points[idx].z, image_width ) ) / (resize_rate*aspect_rate));
	image_y = (int)(( j + shiftY ) / resize_rate);
	//cout << image_x << " " << image_y << endl;
	cloud.points[idx].rgb = setPointRGB( (int)image.data[ 3*image_x + image_y*image.step + 2 ], (int)image.data[ 3*image_x + image_y*image.step + 1 ], (int)image.data[ 3*image_x + image_y*image.step ] );
	idx++;
      }
    }
  }
  fclose(fpX);
  fclose(fpY);
  fclose(fpZ);
  fclose(fpC);

  cloud.width = idx;
  cloud.points.resize (cloud.width * cloud.height);
}

void readPointsSR( pcl::PointCloud<pcl::PointXYZRGB> &cloud, const cv::Mat image, int image_width, const char* filename1, const char* filename2, const char* filename3, const char* filename4, const float dis_depth ){
  FILE *fpX = fopen( filename1, READ_MODE );
  FILE *fpY = fopen( filename2, READ_MODE );
  FILE *fpZ = fopen( filename3, READ_MODE );
  FILE *fpC = fopen( filename4, READ_MODE );

  float dis_min = FLT_MAX;
  for( int j=0; j<SHEIGHT; j++ ){
    for( int i=0; i<SWIDTH; i++ ){
      float z;
#ifdef BINARY_MODE
      fread( &z, sizeof(float), 1, fpZ );
      unsigned char conf;
      fread( &conf, sizeof(unsigned char), 1, fpC );
#else
      fscanf( fpZ, "%f ", &z) );
      int conf;
      fscanf( fpC, "%d ", &conf );
#endif
      if( ( conf > CONF_TH ) && ( dis_min > z ) ) dis_min = z;
    }
  }
  fclose(fpZ);
  fclose(fpC);
  fpZ = fopen( filename3, READ_MODE );
  fpC = fopen( filename4, READ_MODE );

  int image_x, image_y;
  int idx = 0;
  for( int j=0; j<SHEIGHT; j++ ){
    for( int i=0; i<SWIDTH; i++ ){
#ifdef BINARY_MODE
      fread( &(cloud.points[idx].x), sizeof(float), 1, fpX );
      fread( &(cloud.points[idx].y), sizeof(float), 1, fpY );
      fread( &(cloud.points[idx].z), sizeof(float), 1, fpZ );
      unsigned char conf;
      fread( &conf, sizeof(unsigned char), 1, fpC );
#else
      fscanf( fpX, "%f ", &(cloud.points[idx].x) );
      fscanf( fpY, "%f ", &(cloud.points[idx].y) );
      fscanf( fpZ, "%f ", &(cloud.points[idx].z) );
      int conf;
      fscanf( fpC, "%d ", &conf );
#endif
      if( ( conf > CONF_TH ) && ( cloud.points[idx].z < dis_min + dis_depth ) ){ // only confident and close points
	image_x = (int)(( i + getShiftX( i, cloud.points[idx].z, image_width ) ) / (resize_rate*aspect_rate));
	image_y = (int)(( j + shiftY ) / resize_rate);
	//cout << image_x << " " << image_y << endl;
	cloud.points[idx].rgb = setPointRGB( (int)image.data[ 3*image_x + image_y*image.step + 2 ], (int)image.data[ 3*image_x + image_y*image.step + 1 ], (int)image.data[ 3*image_x + image_y*image.step ] );
	idx++;
      }
    }
  }
  fclose(fpX);
  fclose(fpY);
  fclose(fpZ);
  fclose(fpC);

  cloud.width = idx;
  cloud.points.resize (cloud.width * cloud.height);
}

int main(int argc, char **argv)
{
  if( argc != 3 ){
    cerr<<"usage: "<<argv[0]<<" [model_name] <registration_num>"<<endl;
    exit( EXIT_FAILURE );
  }
  const int obj_num = atoi( argv[2] );  // 書き込むメッシュデータの数

  //* SRとFlea2間のキャリブのパラメータ 読み込み
  char line[ 100 ];
  FILE *fp_param = fopen( "param/SR_Flea2_calib.txt", "r" );
  fscanf( fp_param, "%s %f\n", line, &resize_rate );
  fscanf( fp_param, "%s %f\n", line, &aspect_rate );
  fscanf( fp_param, "%s %d\n", line, &shiftX );
  fscanf( fp_param, "%s %d\n", line, &shiftY );
  fscanf( fp_param, "%s %d\n", line, &dis_1_pixel );
  fclose( fp_param );

  // Fill in the cloud data
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  cloud.height   = 1;
  cloud.is_dense = false;

  char tmpname1[ 100 ];
  char tmpname2[ 100 ];
  char tmpname3[ 100 ];
  char tmpname4[ 100 ];
  char tmpname5[ 100 ];
  char tmpname6[ 100 ];
  char obj_filename[ 100 ];
  for( int i=0; i<obj_num; i++ ){
    cloud.width    = SWIDTH * SHEIGHT;
    cloud.points.resize (cloud.width * cloud.height);

    //* SRデータ 読み込み
    sprintf(tmpname1, "models/%s/X/%03d.dat",argv[1],i );
    sprintf(tmpname2, "models/%s/Y/%03d.dat",argv[1],i );
    sprintf(tmpname3, "models/%s/Z/%03d.dat",argv[1],i );
    sprintf(tmpname4, "models/%s/C/%03d.dat",argv[1],i );
    sprintf(tmpname5, "models/%s/Mask/%03d.dat",argv[1],i );
    sprintf(obj_filename, "models/%s/Points/%03d.pcd",argv[1],i );

    //* 画像データの読み込み
    sprintf( tmpname6, "models/%s/Textures/%03d.ppm",argv[1], i );
    cv::Mat image = cv::imread( tmpname6 );
    int image_width = image.cols * resize_rate * aspect_rate;

    //* 色付き点群作成、書き込み
    if( RELATIVE_MODE )
      readPointsSR( cloud, image, image_width, tmpname1, tmpname2, tmpname3, tmpname4, DIS_DEPTH );
    else
      readPointsSR( cloud, image, image_width, tmpname1, tmpname2, tmpname3, tmpname4 );
    pcl::io::savePCDFile (obj_filename, cloud, true);
    ROS_INFO ("Saved %d data points to %s.", (int)cloud.points.size (), obj_filename);
  }

  return(0);
}
