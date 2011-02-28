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

/***************************************************/
/* 環境の計測データ（複数視点から取得したもの）をcolor point cloud化   */
/* 個々の視点の観測データが個々のファイルに保存される        */
/***************************************************/

//***************
//* モードの選択
#define BINARY_MODE // SRデータをバイナリで読み込み
#ifdef BINARY_MODE
#define READ_MODE "rb"
#else
#define READ_MODE "r"
#endif

using namespace std;
using namespace pcl;

const unsigned char CONF_TH = 200;

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

void transPoints( pcl::PointCloud<pcl::PointXYZRGB> &cloud, const char* filename, bool meter_mode = true ){
  float x0,y0,z0;
  float R[9];
  FILE *fp=fopen(filename,"r");
  if( fp==NULL ){
    cerr << "Warning (in transPoints): Camera file \"" << filename << "\" not found. The mesh was not transformed." << endl;
    return;
  }

  fscanf(fp,"%f %f %f",&x0,&y0,&z0);
  for(int i=0;i<9;i++)
    fscanf(fp,"%f ",R+i);
  fclose(fp);

  if( !meter_mode ){
    x0 *= 1000; // 単位をmmにする
    y0 *= 1000; // 単位をmmにする
    z0 *= 1000; // 単位をmmにする
  }
    
  //* 頂点座標に回転/移動をかける
  float x,y,z;
  for( int i=0; i<cloud.points.size(); i++ ){
    x = cloud.points[ i ].x;
    y = cloud.points[ i ].y;
    z = cloud.points[ i ].z;
    cloud.points[ i ].x = R[0] * x + R[1] * y + R[2] * z + x0;
    cloud.points[ i ].y = R[3] * x + R[4] * y + R[5] * z + y0;
    cloud.points[ i ].z = R[6] * x + R[7] * y + R[8] * z + z0;
  }
}

int main(int argc, char **argv)
{
  if( argc != 2 ){
    cerr<<"usage: "<<argv[0]<<" <registration_num>"<<endl;
    exit( EXIT_FAILURE );
  }
  const int obj_num = atoi( argv[1] );  // 書き込むcolor point cloudの数

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
    sprintf(tmpname1, "scene/X/%03d.dat",i );
    sprintf(tmpname2, "scene/Y/%03d.dat",i );
    sprintf(tmpname3, "scene/Z/%03d.dat",i );
    sprintf(tmpname4, "scene/C/%03d.dat",i );
    sprintf(tmpname5, "scene/Camera/%03d.dat",i );
    sprintf(obj_filename, "scene/Points/%03d.pcd",i );

    //* 画像データの読み込み
    sprintf( tmpname6, "scene/Textures/%03d.ppm", i );
    cv::Mat image = cv::imread( tmpname6 );
    int image_width = image.cols * resize_rate * aspect_rate;

    //* 色付き点群作成、グローバル座標への変換、書き込み
    readPointsSR( cloud, image, image_width, tmpname1, tmpname2, tmpname3, tmpname4 );
    transPoints( cloud, tmpname5, true );
    pcl::io::savePCDFile (obj_filename, cloud, true);
    ROS_INFO ("Saved %d data points to %s.", (int)cloud.points.size (), obj_filename);
  }
  return(0);
}
