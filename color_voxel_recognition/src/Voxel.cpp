#include <iostream>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>

#include "color_voxel_recognition/Voxel.hpp"
#include "color_voxel_recognition/ppmFile.hpp"
#include "pcl/io/io.h"

using namespace std;

const double angle_norm = M_PI / 510;

//***************************//
//* コンストラクタとデストラクタ *//
//***************************//

Voxel::Voxel() :
  fp_r(NULL),
  fp_w(NULL),
  exist_flag(NULL),
  exist_num(NULL),
  xsize(0),
  ysize(0),
  zsize(0),
  xysize(0),
  xyzsize(0),
  vr (NULL),
  vg (NULL),
  vb (NULL),
  _vr(NULL),
  _vg(NULL),
  _vb(NULL),
  voxel_size(0)
{ }

Voxel::Voxel( bool _ascii ) :
  ascii(_ascii),
  fp_r(NULL),
  fp_w(NULL),
  exist_flag(NULL),
  exist_num(NULL),
  xsize(0),
  ysize(0),
  zsize(0),
  xysize(0),
  xyzsize(0),
  vr (NULL),
  vg (NULL),
  vb (NULL),
  _vr(NULL),
  _vg(NULL),
  _vb(NULL),
  voxel_size(0)
{ }

Voxel::Voxel( const char *filename, const char *mode ) :
  fp_r(NULL),
  fp_w(NULL),
  exist_flag(NULL),
  exist_num(NULL),
  xsize(0),
  ysize(0),
  zsize(0),
  xysize(0),
  xyzsize(0),
  vr (NULL),
  vg (NULL),
  vb (NULL),
  _vr(NULL),
  _vg(NULL),
  _vb(NULL),
  voxel_size(0) {

  if( strcmp( mode, "r" ) == 0 ){
    ascii = true;
    fp_r = fopen( filename, mode );
    fscanf(fp_r,"%d %d %d\n",&xsize,&ysize,&zsize);
    xysize = xsize * ysize;
    xyzsize = xysize * zsize;
  }
  else if( strcmp( mode, "rb" ) == 0 ){
    ascii = false;
    fp_r = fopen( filename, mode );
    fread(&xsize,sizeof(int),1,fp_r);
    fread(&ysize,sizeof(int),1,fp_r);
    fread(&zsize,sizeof(int),1,fp_r);
    xysize = xsize * ysize;
    xyzsize = xysize * zsize;
  }
  else if( strcmp( mode, "w" ) == 0 ){
    ascii = true;
    fp_w = fopen( filename, mode );
  }
  else if( strcmp( mode, "wb" ) == 0 ){
    ascii = false;
    fp_w = fopen( filename, mode );
  }
}

Voxel::Voxel( const Voxel &another ) :
  fp_r(NULL),
  fp_w(NULL) {
  *this = another;
}

Voxel::~Voxel(){ 
  if( fp_r != NULL ) { fclose( fp_r ); fp_r = NULL; }
  if( fp_w != NULL ) { fclose( fp_w ); fp_w = NULL; }
  if( exist_flag != NULL ) delete[] exist_flag;
  if( exist_num != NULL ) delete[] exist_num;
  if( vr != NULL ) delete[] vr;
  if( vg != NULL ) delete[] vg;
  if( vb != NULL ) delete[] vb;
  if( _vr != NULL ) delete[] _vr;
  if( _vg != NULL ) delete[] _vg;
  if( _vb != NULL ) delete[] _vb;
}

//***********************//
//* ボクセル作成時に使う関数 *//
//***********************//

//****************************
//* ボクセルの一辺の長さを指定
void Voxel::setVoxelSize( float val ){ voxel_size = val; }

//**********************************************
//* ボクセル化対象データの三次元座表の最大値、最小値を指定
void Voxel::setMinMax( float _x_min, float _x_max, float _y_min, float _y_max, float _z_min, float _z_max ){
  if( voxel_size == 0 ){
    cerr<< "ERR (in Voxel::setMinMax): Set voxel size!" <<endl;
    exit( EXIT_FAILURE );
  }

  x_min = _x_min;
  x_max = _x_max;
  y_min = _y_min;
  y_max = _y_max;
  z_min = _z_min;
  z_max = _z_max;
  double xsize_d=x_max-x_min;
  double ysize_d=y_max-y_min;
  double zsize_d=z_max-z_min;

  //* 注 先頭ボクセルと最終ボクセルに値を入れないようにするため、本来の大きさの+2になっている
  xsize = (int)(xsize_d/voxel_size)+3;
  ysize = (int)(ysize_d/voxel_size)+3;
  zsize = (int)(zsize_d/voxel_size)+3;
  if( xsize <= 3 ) xsize = 0;
  if( ysize <= 3 ) ysize = 0;
  if( zsize <= 3 ) zsize = 0;

  xysize = xsize * ysize;
  xyzsize = xysize * zsize;

  if( fp_w != NULL ){
    if( ascii )
      fprintf(fp_w,"%d %d %d\n",xsize,ysize,zsize); 
    else{
      fwrite(&xsize,sizeof(int),1,fp_w);
      fwrite(&ysize,sizeof(int),1,fp_w);
      fwrite(&zsize,sizeof(int),1,fp_w);
    }
  }

  if( exist_flag != NULL ) delete[] exist_flag;
  exist_flag = new bool[ xyzsize ];
  for(int i=0;i<xyzsize;i++)
    if( exist_flag[i] ) exist_flag[i] = false;
  if( exist_num != NULL ) delete[] exist_num;
  exist_num = new int[ xyzsize ];
  for(int i=0;i<xyzsize;i++)
    if( exist_num[i] ) exist_num[i] = 0;
}

//***********************************
//* メッシュデータのボクセル化 RGB2値化せず
void Voxel::obj2voxel( Obj &object, const char *image_filename, ReverseMode mode ){
  Ppm ppm;
  ppm.read(image_filename);
  for (int l_index=0;l_index<object.f_num;l_index++){
    face2voxel(  object.vertices[ object.faces[ l_index ].v1 ].x - x_min, 
		 object.vertices[ object.faces[ l_index ].v2 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v3 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v1 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v2 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v3 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v1 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v2 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v3 ].z - z_min,
		 object.verticesT[ object.faces[ l_index ].vt1 ].x,
		 object.verticesT[ object.faces[ l_index ].vt2 ].x,
		 object.verticesT[ object.faces[ l_index ].vt3 ].x,
		 object.verticesT[ object.faces[ l_index ].vt1 ].y,
		 object.verticesT[ object.faces[ l_index ].vt2 ].y,
		 object.verticesT[ object.faces[ l_index ].vt3 ].y,
		 ppm.ImageData(),
		 ppm.Width(), ppm.Height(), ppm.Width()*3, mode );
  }
}

//***********************************
//* メッシュデータのボクセル化 RGB2値化する
void Voxel::obj2voxel( Obj &object, const char *image_filename, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b ){
  Ppm ppm;
  ppm.read(image_filename);
  for (int l_index=0;l_index<object.f_num;l_index++){
    face2voxel(  object.vertices[ object.faces[ l_index ].v1 ].x - x_min, 
		 object.vertices[ object.faces[ l_index ].v2 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v3 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v1 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v2 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v3 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v1 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v2 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v3 ].z - z_min,
		 object.verticesT[ object.faces[ l_index ].vt1 ].x,
		 object.verticesT[ object.faces[ l_index ].vt2 ].x,
		 object.verticesT[ object.faces[ l_index ].vt3 ].x,
		 object.verticesT[ object.faces[ l_index ].vt1 ].y,
		 object.verticesT[ object.faces[ l_index ].vt2 ].y,
		 object.verticesT[ object.faces[ l_index ].vt3 ].y,
		 ppm.ImageData(),
		 ppm.Width(), ppm.Height(), ppm.Width()*3,
		 color_threshold_r, color_threshold_g, color_threshold_b );
  }
}

//************************************
//* メッシュデータのボクセル化 ファイルに出力
void Voxel::obj2voxel_file( Obj &object, const char *image_filename ){
  Ppm ppm;
  ppm.read(image_filename);
  for (int l_index=0;l_index<object.f_num;l_index++){
    face2voxel_file(  object.vertices[ object.faces[ l_index ].v1 ].x - x_min, 
		      object.vertices[ object.faces[ l_index ].v2 ].x - x_min,
		      object.vertices[ object.faces[ l_index ].v3 ].x - x_min,
		      object.vertices[ object.faces[ l_index ].v1 ].y - y_min,
		      object.vertices[ object.faces[ l_index ].v2 ].y - y_min,
		      object.vertices[ object.faces[ l_index ].v3 ].y - y_min,
		      object.vertices[ object.faces[ l_index ].v1 ].z - z_min,
		      object.vertices[ object.faces[ l_index ].v2 ].z - z_min,
		      object.vertices[ object.faces[ l_index ].v3 ].z - z_min,
		      object.verticesT[ object.faces[ l_index ].vt1 ].x,
		      object.verticesT[ object.faces[ l_index ].vt2 ].x,
		      object.verticesT[ object.faces[ l_index ].vt3 ].x,
		      object.verticesT[ object.faces[ l_index ].vt1 ].y,
		      object.verticesT[ object.faces[ l_index ].vt2 ].y,
		      object.verticesT[ object.faces[ l_index ].vt3 ].y,
		      ppm.ImageData(),
		      ppm.Width(), ppm.Height(), ppm.Width()*3 );
  }
}

#ifdef USE_PCL
//**********************
//* binarize RGB values
void Voxel::binarize( unsigned char thR, unsigned char thG, unsigned char thB ){
  for( int idx = 0; idx < xyzsize; idx++ )
    if( (vr[ idx ]!=0)||(_vr[ idx ]!=0) )
      setRGB( idx, vr[ idx ], vg[ idx ], vb[ idx ], thR, thG, thB );
}

//****************************************************
//* set the minimum and maximum xyz valuse of 3D data
void Voxel::setMinMax(){
  if( voxel_size == 0 ){
    cerr<< "ERR (in Voxel::setMinMax): Set voxel size!" <<endl;
    exit( EXIT_FAILURE );
  }

  double xsize_d=x_max-x_min;
  double ysize_d=y_max-y_min;
  double zsize_d=z_max-z_min;

  // Note: the size is larger than the actual size of 3D data so that the voxels on boundaries should be empty.
  xsize = (int)(xsize_d/voxel_size)+3;
  ysize = (int)(ysize_d/voxel_size)+3;
  zsize = (int)(zsize_d/voxel_size)+3;
  if( xsize <= 3 ) xsize = 0;
  if( ysize <= 3 ) ysize = 0;
  if( zsize <= 3 ) zsize = 0;

  xysize = xsize * ysize;
  xyzsize = xysize * zsize;

  if( exist_flag != NULL ) delete[] exist_flag;
  exist_flag = new bool[ xyzsize ];
  for(int i=0;i<xyzsize;i++)
    if( exist_flag[i] ) exist_flag[i] = false;
  if( exist_num != NULL ) delete[] exist_num;
  exist_num = new int[ xyzsize ];
  for(int i=0;i<xyzsize;i++)
    if( exist_num[i] ) exist_num[i] = 0;
}

//****************************************
//* transform point cloud into voxel data
void Voxel::points2voxel( pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster, ReverseMode mode ){

  // calculate the minimum and maximum xyz values of points.
  x_min = FLT_MAX; y_min = FLT_MAX; z_min = FLT_MAX;
  x_max = -FLT_MAX; y_max = -FLT_MAX; z_max = -FLT_MAX;
  for(unsigned int i =0; i < cloud_object_cluster.points.size(); i++){
    if( x_min > cloud_object_cluster.points[i].x )
      x_min = cloud_object_cluster.points[i].x;
    if( y_min > cloud_object_cluster.points[i].y )
      y_min = cloud_object_cluster.points[i].y;
    if( z_min > cloud_object_cluster.points[i].z )
      z_min = cloud_object_cluster.points[i].z;
    if( x_max < cloud_object_cluster.points[i].x )
      x_max = cloud_object_cluster.points[i].x;
    if( y_max < cloud_object_cluster.points[i].y )
      y_max = cloud_object_cluster.points[i].y;
    if( z_max < cloud_object_cluster.points[i].z )
      z_max = cloud_object_cluster.points[i].z;
  }
  setMinMax();
  createVoxelData();
  cleanVoxelData();

  float *red = new float[ xyzsize ];
  float *green = new float[ xyzsize ];
  float *blue = new float[ xyzsize ];
  for( int i=0; i<xyzsize; i++ ){
    red[ i ] = 0; green[ i ] = 0; blue[ i ] = 0;
  }

  for(unsigned int i =0; i < cloud_object_cluster.points.size(); i++){
    const int idx = (int)((cloud_object_cluster.points[i].x-x_min)/voxel_size) + 1
      + xsize * ((int)((cloud_object_cluster.points[i].y-y_min)/voxel_size) + 1)
      + xysize * ((int)((cloud_object_cluster.points[i].z-z_min)/voxel_size) + 1);
    exist_num[ idx ] ++;

    //convert back to r, g and b channels
    int color = *reinterpret_cast<const int*>(&(cloud_object_cluster.points[i].rgb));
    int r = (0xff0000 & color) >> 16;
    int g = (0x00ff00 & color) >> 8;
    int b =  0x0000ff & color;

    red[ idx ] += (float)r;  
    green[ idx ] += (float)g;
    blue[ idx ] += (float)b;
  }

  for( int i=0; i<xyzsize; i++ )
    if( exist_num[ i ] != 0 )
      setRGB( i, red[ i ]/exist_num[ i ], green[ i ]/exist_num[ i ], blue[ i ]/exist_num[ i ], mode );
  
  delete[] red;
  delete[] green;
  delete[] blue;

  for( int i=0; i<xyzsize; i++ )
    if( exist_num[ i ] > 0 ) exist_flag[ i ] = true;
}

void Voxel::writeVoxel(){
  if( fp_w != NULL ){
    if( ascii ){
      fprintf(fp_w,"%d %d %d\n",xsize,ysize,zsize); 
      int idx = 0;
      for( int z=0; z<zsize; z++ ){
	for( int y=0; y<ysize; y++ ){
	  for( int x=0; x<xsize; x++ ){
	    if( exist_num[ idx ] > 0 )
	      fprintf(fp_w,"%d %d %d %d %d %d\n",x,y,z,vr[ idx ],vg[ idx ],vb[ idx ]);
	    idx++;
	  }
	}
      }
    }
    else{
      fwrite(&xsize,sizeof(int),1,fp_w);
      fwrite(&ysize,sizeof(int),1,fp_w);
      fwrite(&zsize,sizeof(int),1,fp_w);
      int idx = 0;
      for( int z=0; z<zsize; z++ ){
	for( int y=0; y<ysize; y++ ){
	  for( int x=0; x<xsize; x++ ){
	    if( exist_num[ x + y*xsize + z*xysize ] > 0 ){
	      fwrite(&x,sizeof(int),1,fp_w);
	      fwrite(&y,sizeof(int),1,fp_w);
	      fwrite(&z,sizeof(int),1,fp_w);
	      fwrite(&(vr[ idx ]),sizeof(unsigned char),1,fp_w);
	      fwrite(&(vg[ idx ]),sizeof(unsigned char),1,fp_w);
	      fwrite(&(vb[ idx ]),sizeof(unsigned char),1,fp_w);
	    }
	    idx++;
	  }
	}
      }
    }
  }
}

void Voxel::points2voxel( pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_object_cluster, ReverseMode mode ){
  // calculate the minimum and maximum xyz values of points.
  x_min = FLT_MAX; y_min = FLT_MAX; z_min = FLT_MAX;
  x_max = -FLT_MAX; y_max = -FLT_MAX; z_max = -FLT_MAX;
  for(unsigned int i =0; i < cloud_object_cluster.points.size(); i++){
    if( x_min > cloud_object_cluster.points[i].x )
      x_min = cloud_object_cluster.points[i].x;
    if( y_min > cloud_object_cluster.points[i].y )
      y_min = cloud_object_cluster.points[i].y;
    if( z_min > cloud_object_cluster.points[i].z )
      z_min = cloud_object_cluster.points[i].z;
    if( x_max < cloud_object_cluster.points[i].x )
      x_max = cloud_object_cluster.points[i].x;
    if( y_max < cloud_object_cluster.points[i].y )
      y_max = cloud_object_cluster.points[i].y;
    if( z_max < cloud_object_cluster.points[i].z )
      z_max = cloud_object_cluster.points[i].z;
  }
  setMinMax();
  createVoxelData();
  cleanVoxelData();

  float *red = new float[ xyzsize ];
  float *green = new float[ xyzsize ];
  float *blue = new float[ xyzsize ];
  for( int i=0; i<xyzsize; i++ ){
    red[ i ] = 0; green[ i ] = 0; blue[ i ] = 0;
  }

  for(unsigned int i =0; i < cloud_object_cluster.points.size(); i++){
    const int idx = (int)((cloud_object_cluster.points[i].x-x_min)/voxel_size) + 1
      + xsize * ((int)((cloud_object_cluster.points[i].y-y_min)/voxel_size) + 1)
      + xysize * ((int)((cloud_object_cluster.points[i].z-z_min)/voxel_size) + 1);
    exist_num[ idx ] ++;

    //convert back to r, g and b channels
    int color = *reinterpret_cast<const int*>(&(cloud_object_cluster.points[i].rgb));
    int r = (0xff0000 & color) >> 16;
    int g = (0x00ff00 & color) >> 8;
    int b =  0x0000ff & color;

    red[ idx ] += (float)r;  
    green[ idx ] += (float)g;
    blue[ idx ] += (float)b;
  }

  for( int i=0; i<xyzsize; i++ )
    if( exist_num[ i ] != 0 )
      setRGB( i, red[ i ]/exist_num[ i ], green[ i ]/exist_num[ i ], blue[ i ]/exist_num[ i ], mode );
  
  delete[] red;
  delete[] green;
  delete[] blue;

  for( int i=0; i<xyzsize; i++ )
    if( exist_num[ i ] > 0 ) exist_flag[ i ] = true;
}
#endif

#ifdef USE_OPENCV
//*******************************************************
//* メッシュデータのボクセル化 画像配列はOpenCVのもの RGB2値化せず
void Voxel::obj2voxel( Obj &object, const cv::Mat& color_img, ReverseMode mode ){
  for (int l_index=0;l_index<object.f_num;l_index++){
    face2voxel(  object.vertices[ object.faces[ l_index ].v1 ].x - x_min, 
		 object.vertices[ object.faces[ l_index ].v2 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v3 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v1 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v2 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v3 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v1 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v2 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v3 ].z - z_min,
		 object.verticesT[ object.faces[ l_index ].vt1 ].x,
		 object.verticesT[ object.faces[ l_index ].vt2 ].x,
		 object.verticesT[ object.faces[ l_index ].vt3 ].x,
		 object.verticesT[ object.faces[ l_index ].vt1 ].y,
		 object.verticesT[ object.faces[ l_index ].vt2 ].y,
		 object.verticesT[ object.faces[ l_index ].vt3 ].y,
		 color_img.data,
		 color_img.cols, color_img.rows, color_img.step, mode );
  }
}

//*******************************************************
//* メッシュデータのボクセル化 画像配列はOpenCVのもの RGB2値化する
void Voxel::obj2voxel( Obj &object, const cv::Mat& color_img, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b ){
  for (int l_index=0;l_index<object.f_num;l_index++){
    face2voxel(  object.vertices[ object.faces[ l_index ].v1 ].x - x_min, 
		 object.vertices[ object.faces[ l_index ].v2 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v3 ].x - x_min,
		 object.vertices[ object.faces[ l_index ].v1 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v2 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v3 ].y - y_min,
		 object.vertices[ object.faces[ l_index ].v1 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v2 ].z - z_min,
		 object.vertices[ object.faces[ l_index ].v3 ].z - z_min,
		 object.verticesT[ object.faces[ l_index ].vt1 ].x,
		 object.verticesT[ object.faces[ l_index ].vt2 ].x,
		 object.verticesT[ object.faces[ l_index ].vt3 ].x,
		 object.verticesT[ object.faces[ l_index ].vt1 ].y,
		 object.verticesT[ object.faces[ l_index ].vt2 ].y,
		 object.verticesT[ object.faces[ l_index ].vt3 ].y,
		 color_img.data,
		 color_img.cols, color_img.rows, color_img.step,
		 color_threshold_r, color_threshold_g, color_threshold_b );
  }
}
#endif

//**************************//
//* ボクセル読み込み時に使う関数 *//
//**************************//

//********************************
//* ボクセルのx軸、y軸、z軸上の個数を指定
void Voxel::setXYZsize( int _xsize, int _ysize, int _zsize ){ 
  xsize = _xsize;
  ysize = _ysize;
  zsize = _zsize;
  xysize = xsize * ysize;
  xyzsize = xysize * zsize;
}

//******************
//* ボクセルのメモリ確保
void Voxel::createVoxelData(){
  if( vr != NULL ) delete[] vr;
  if( vg != NULL ) delete[] vg;
  if( vb != NULL ) delete[] vb;
  if( _vr != NULL ) delete[] _vr;
  if( _vg != NULL ) delete[] _vg;
  if( _vb != NULL ) delete[] _vb;
  //if( exist_flag != NULL ) delete[] exist_flag;
  //exist_flag = NULL;
  if( xyzsize != 0 ){
    vr  = new unsigned char [ xyzsize ];
    vg  = new unsigned char [ xyzsize ];
    vb  = new unsigned char [ xyzsize ];
    _vr = new unsigned char [ xyzsize ];
    _vg = new unsigned char [ xyzsize ];
    _vb = new unsigned char [ xyzsize ];
  }
  else{
    vr = NULL;
    vg = NULL;
    vb = NULL;
    _vr = NULL;
    _vg = NULL;
    _vb = NULL;
  }
}

//*********************
//* ボクセルの値を0で埋める
void Voxel::cleanVoxelData(){
  for( int i=0; i<xyzsize; i++ ){
    vr[ i ] = 0;
    vg[ i ] = 0;
    vb[ i ] = 0;
    _vr[ i ] = 0;
    _vg[ i ] = 0;
    _vb[ i ] = 0;
  }
}

//************************************************************
//* 最大最小値を記録 (ただしexist_flagに値がちゃんと入っていることを仮定)
void Voxel::getMinMax( int &x_min_i, int &x_max_i, int &y_min_i, int &y_max_i, int &z_min_i, int &z_max_i ){
  x_min_i = 1000000;
  x_max_i = 0;
  y_min_i = 1000000;
  y_max_i = 0;
  z_min_i = 1000000;
  z_max_i = 0;

  for( int x = 0; x < xsize; x++ ){
    for( int y = 0; y < ysize; y++ ){
      for( int z = 0; z < zsize; z++ ){
	if( exist_flag[ z * xysize + y * xsize + x ] ){
	  if(x<x_min_i) x_min_i = x;
	  if(x>x_max_i) x_max_i = x;
	  if(y<y_min_i) y_min_i = y;
	  if(y>y_max_i) y_max_i = y;
	  if(z<z_min_i) z_min_i = z;
	  if(z>z_max_i) z_max_i = z;
	}
      }
    }
  }
}

//***********************************
//* 補助成分値を計算しないボクセルの読み込み
void Voxel::readVoxel_normal( const char *filename ){
  int x,y,z;

  if( ascii ){
    int r,g,b;
    FILE *fp = fopen(filename,"r");
    fscanf(fp,"%d %d %d\n",&xsize,&ysize,&zsize);
    xysize = xsize * ysize;
    xyzsize = xysize * zsize;
    if( vr == NULL ){
      vr  = new unsigned char [ xyzsize ];
      vg  = new unsigned char [ xyzsize ];
      vb  = new unsigned char [ xyzsize ];
      for( int i=0; i<xyzsize; i++ ){
	vr[ i ] = 0;
	vg[ i ] = 0;
	vb[ i ] = 0;
      }
    }
    if( exist_flag == NULL ){
      exist_flag = new bool[ xyzsize ];
      for(int i=0;i<xyzsize;i++)
	if( exist_flag[i] ) exist_flag[i] = false;
    }
    
    while( fscanf(fp,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF ){
      exist_flag[ z * xysize + y * xsize + x ] = true;
      vr[ z * xysize + y * xsize + x ] = r;
      vg[ z * xysize + y * xsize + x ] = g;
      vb[ z * xysize + y * xsize + x ] = b;
    }
    fclose(fp);
  }
  else{
    unsigned char r,g,b;
    FILE *fp = fopen(filename,"rb");
    fread(&xsize,sizeof(int),1,fp);
    fread(&ysize,sizeof(int),1,fp);
    fread(&zsize,sizeof(int),1,fp);
    xysize = xsize * ysize;
    xyzsize = xysize * zsize;
    if( vr == NULL ){
      vr  = new unsigned char [ xyzsize ];
      vg  = new unsigned char [ xyzsize ];
      vb  = new unsigned char [ xyzsize ];
      for( int i=0; i<xyzsize; i++ ){
	vr[ i ] = 0;
	vg[ i ] = 0;
	vb[ i ] = 0;
      }
    }
    if( exist_flag == NULL ){
      exist_flag = new bool[ xyzsize ];
      for(int i=0;i<xyzsize;i++)
	if( exist_flag[i] ) exist_flag[i] = false;
    }
    while( fread(&x,sizeof(int),1,fp) > 0 ){
      fread(&y,sizeof(int),1,fp);
      fread(&z,sizeof(int),1,fp);
      fread(&r,sizeof(unsigned char),1,fp);
      fread(&g,sizeof(unsigned char),1,fp);
      fread(&b,sizeof(unsigned char),1,fp);
      exist_flag[ z * xysize + y * xsize + x ] = true;
      vr[ z * xysize + y * xsize + x ] = r;
      vg[ z * xysize + y * xsize + x ] = g;
      vb[ z * xysize + y * xsize + x ] = b;
    }
    fclose(fp);
  }
}

//********************************************************************
//* RGB二値化しないボクセルの読み込み （インスタンス宣言時にファイル名を指定する場合）
void Voxel::readVoxel( ReverseMode mode ){
  int x,y,z;
  if( ascii ){
    int r,g,b;
    while( fscanf(fp_r,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF )
      setRGB( z * xysize + y * xsize + x, (unsigned char)r, (unsigned char)g, (unsigned char)b, mode );
  }
  else{
    unsigned char r,g,b;
    while( fread(&x,sizeof(int),1,fp_r) > 0 ){
      fread(&y,sizeof(int),1,fp_r);
      fread(&z,sizeof(int),1,fp_r);
      fread(&r,sizeof(unsigned char),1,fp_r);
      fread(&g,sizeof(unsigned char),1,fp_r);
      fread(&b,sizeof(unsigned char),1,fp_r);
      setRGB( z * xysize + y * xsize + x, r, g, b, mode );
    }
  }
  fclose(fp_r);
  fp_r = NULL;
}

//********************************************************************
//* RGB二値化するボクセルの読み込み  （インスタンス宣言時にファイル名を指定する場合）
void Voxel::readVoxel( unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b ){
  int x,y,z;
  if( ascii ){
    int r,g,b;
    while( fscanf(fp_r,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF )
      setRGB( z * xysize + y * xsize + x, (unsigned char)r, (unsigned char)g, (unsigned char)b, color_threshold_r, color_threshold_g, color_threshold_b );
  }
  else{
    unsigned char r,g,b;
    while( fread(&x,sizeof(int),1,fp_r) > 0 ){
      fread(&y,sizeof(int),1,fp_r);
      fread(&z,sizeof(int),1,fp_r);
      fread(&r,sizeof(unsigned char),1,fp_r);
      fread(&g,sizeof(unsigned char),1,fp_r);
      fread(&b,sizeof(unsigned char),1,fp_r);
      setRGB( z * xysize + y * xsize + x, r, g, b, color_threshold_r, color_threshold_g, color_threshold_b );
    }
  }
  fclose(fp_r);
  fp_r = NULL;
}

//********************************************************************
//* RGB二値化しないボクセルの読み込み （インスタンス宣言時にファイル名を指定しない場合）
//* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
void Voxel::readVoxel( const char *filename, ReverseMode mode, bool size_read_flg ){
  int x,y,z;

  if( ascii ){
    int r,g,b;
    FILE *fp = fopen(filename,"r");
    if( size_read_flg ){
      fscanf(fp,"%d %d %d\n",&xsize,&ysize,&zsize);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fscanf(fp,"%d %d %d\n",&tmp,&tmp,&tmp);
    }
    
    while( fscanf(fp,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF )
      setRGB( z * xysize + y * xsize + x, (unsigned char)r, (unsigned char)g, (unsigned char)b, mode );
    fclose(fp);
  }
  else{
    unsigned char r,g,b;
    FILE *fp = fopen(filename,"rb");
    if( size_read_flg ){
      fread(&xsize,sizeof(int),1,fp);
      fread(&ysize,sizeof(int),1,fp);
      fread(&zsize,sizeof(int),1,fp);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
    }
    while( fread(&x,sizeof(int),1,fp) > 0 ){
      fread(&y,sizeof(int),1,fp);
      fread(&z,sizeof(int),1,fp);
      fread(&r,sizeof(unsigned char),1,fp);
      fread(&g,sizeof(unsigned char),1,fp);
      fread(&b,sizeof(unsigned char),1,fp);
      setRGB( z * xysize + y * xsize + x, r, g, b, mode );
    }
    fclose(fp);
  }
}

//********************************************************************
//* RGB二値化するボクセルの読み込み  （インスタンス宣言時にファイル名を指定しない場合）
//* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
void Voxel::readVoxel( const char *filename, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b,  bool size_read_flg ){
  int x,y,z;

  if( ascii ){
    int r,g,b;
    FILE *fp = fopen(filename,"r");
    if( size_read_flg ){
      fscanf(fp,"%d %d %d\n",&xsize,&ysize,&zsize);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fscanf(fp,"%d %d %d\n",&tmp,&tmp,&tmp);
    }
    
    while( fscanf(fp,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF )
      setRGB( z * xysize + y * xsize + x, (unsigned char)r, (unsigned char)g, (unsigned char)b, color_threshold_r, color_threshold_g, color_threshold_b );
    fclose(fp);
  }
  else{
    unsigned char r,g,b;
    FILE *fp = fopen(filename,"rb");
    if( size_read_flg ){
      fread(&xsize,sizeof(int),1,fp);
      fread(&ysize,sizeof(int),1,fp);
      fread(&zsize,sizeof(int),1,fp);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
    }
    while( fread(&x,sizeof(int),1,fp) > 0 ){
      fread(&y,sizeof(int),1,fp);
      fread(&z,sizeof(int),1,fp);
      fread(&r,sizeof(unsigned char),1,fp);
      fread(&g,sizeof(unsigned char),1,fp);
      fread(&b,sizeof(unsigned char),1,fp);
      setRGB( z * xysize + y * xsize + x, r, g, b, color_threshold_r, color_threshold_g, color_threshold_b );
    }
    fclose(fp);
  }
}

//*****************************************************************************
//* RGB二値化しないボクセルの読み込み （z方向にオフセット分ずらし、zsize以上のボクセル値を無視）
//* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
void Voxel::readVoxelZoffset( const char *filename, int zoffset, ReverseMode mode, bool size_read_flg ){
  int x,y,z;

  if( ascii ){
    int r,g,b;
    FILE *fp = fopen(filename,"r");
    if( size_read_flg ){
      fscanf(fp,"%d %d %d\n",&xsize,&ysize,&zsize);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fscanf(fp,"%d %d %d\n",&tmp,&tmp,&tmp);
    }
    
    while( fscanf(fp,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF ){
      z -= zoffset;
      if( z >= 0 && z <zsize ) // x,y方向には分割しない仕様になっているので注意
	setRGB( z * xysize + y * xsize + x, (unsigned char)r, (unsigned char)g, (unsigned char)b, mode );
    }
    fclose(fp);
  }
  else{
    unsigned char r,g,b;
    FILE *fp = fopen(filename,"rb");
    if( size_read_flg ){
      fread(&xsize,sizeof(int),1,fp);
      fread(&ysize,sizeof(int),1,fp);
      fread(&zsize,sizeof(int),1,fp);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
    }
    while( fread(&x,sizeof(int),1,fp) > 0 ){
      fread(&y,sizeof(int),1,fp);
      fread(&z,sizeof(int),1,fp);
      fread(&r,sizeof(unsigned char),1,fp);
      fread(&g,sizeof(unsigned char),1,fp);
      fread(&b,sizeof(unsigned char),1,fp);
      z -= zoffset;
      if( z >= 0 && z <zsize ) // x,y方向には分割しない仕様になっているので注意
	setRGB( z * xysize + y * xsize + x, r, g, b, mode );
    }
    fclose(fp);
  }
}

//****************************************************************************
//* RGB二値化するボクセルの読み込み  （z方向にオフセット分ずらし、zsize以上のボクセル値を無視）
//* 注 xsize, ysize, zsize を手動で指定する場合は size_read_flg=false として setXYZsize()関数を使ってください
void Voxel::readVoxelZoffset( const char *filename, int zoffset, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b, bool size_read_flg ){
  int x,y,z;

  if( ascii ){
    int r,g,b;
    FILE *fp = fopen(filename,"r");
    if( size_read_flg ){
      fscanf(fp,"%d %d %d\n",&xsize,&ysize,&zsize);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fscanf(fp,"%d %d %d\n",&tmp,&tmp,&tmp);
    }
    
    while( fscanf(fp,"%d %d %d %d %d %d",&x,&y,&z,&r,&g,&b)!=EOF ){
      z -= zoffset;
      if( z >= 0 && z <zsize ) // x,y方向には分割しない仕様になっているので注意
	setRGB( z * xysize + y * xsize + x, (unsigned char)r, (unsigned char)g, (unsigned char)b, color_threshold_r, color_threshold_g, color_threshold_b );
    }
    fclose(fp);
  }
  else{
    unsigned char r,g,b;
    FILE *fp = fopen(filename,"rb");
    if( size_read_flg ){
      fread(&xsize,sizeof(int),1,fp);
      fread(&ysize,sizeof(int),1,fp);
      fread(&zsize,sizeof(int),1,fp);
      xysize = xsize * ysize;
      xyzsize = xysize * zsize;
      if( vr == NULL ){
	createVoxelData();
	cleanVoxelData();
      }
    }
    else{
      int tmp;
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
      fread(&tmp,sizeof(int),1,fp);
    }
    while( fread(&x,sizeof(int),1,fp) > 0 ){
      fread(&y,sizeof(int),1,fp);
      fread(&z,sizeof(int),1,fp);
      fread(&r,sizeof(unsigned char),1,fp);
      fread(&g,sizeof(unsigned char),1,fp);
      fread(&b,sizeof(unsigned char),1,fp);
      z -= zoffset;
      if( z >= 0 && z <zsize ) // x,y方向には分割しない仕様になっているので注意
	setRGB( z * xysize + y * xsize + x, r, g, b, color_threshold_r, color_threshold_g, color_threshold_b );
    }
    fclose(fp);
  }
}

const Voxel& Voxel::operator=(const Voxel &another) {
  assert (fp_r == NULL);
  assert (fp_w == NULL);
  assert (another.fp_r == NULL);
  assert (another.fp_w == NULL);

  // destruct
  bool reuse_memory = (xyzsize == another.xyzsize);

  if (!reuse_memory) {
    if( exist_flag != NULL ) { delete[] exist_flag; exist_flag = NULL; }
    if( exist_num != NULL ) { delete[] exist_num; exist_num = NULL; }
    if( vr != NULL ) { delete[] vr; vr = NULL; }
    if( vg != NULL ) { delete[] vg; vg = NULL; }
    if( vb != NULL ) { delete[] vb; vb = NULL; }
    if( _vr != NULL ) { delete[] _vr; _vr = NULL; }
    if( _vg != NULL ) { delete[] _vg; _vg = NULL; }
    if( _vb != NULL ) { delete[] _vb; _vb = NULL; }
  }

  fp_r = NULL;
  fp_w = NULL;

  ascii = another.ascii;
  xsize = another.xsize;
  ysize = another.ysize;
  zsize = another.zsize;
  xysize = another.xysize;
  xyzsize = another.xyzsize;
  x_min = another.x_min;
  x_max = another.x_max;
  y_min = another.y_min;
  y_max = another.y_max;
  z_min = another.z_min;
  z_max = another.z_max;
  voxel_size = another.voxel_size;

  if (!reuse_memory) {
    if (xyzsize < 1) {
      exist_flag = NULL;
      exist_num = NULL;
      vr = NULL;
      vg = NULL;
      vb = NULL;
      _vr = NULL;
      _vg = NULL;
      _vb = NULL;
    } else {
      exist_flag = new bool[xyzsize];
      exist_num = new int[xyzsize];
      vr = new unsigned char[xyzsize];
      vg = new unsigned char[xyzsize];
      vb = new unsigned char[xyzsize];
      _vr = new unsigned char[xyzsize];
      _vg = new unsigned char[xyzsize];
      _vb = new unsigned char[xyzsize];
    }
  }

  for (int i=0; i<xyzsize; i++) {
    exist_flag[i] = another.exist_flag[i];
    exist_num[i] = another.exist_num[i];
    vr[i] = another.vr[i];
    vg[i] = another.vg[i];
    vb[i] = another.vb[i];
    _vr[i] = another._vr[i];
    _vg[i] = another._vg[i];
    _vb[i] = another._vb[i];
  }

  return *this;
}

//**************//
//* private関数 *//
//**************//

//***********************************************
//* RGB の主成分値と補助成分値をセットする （二値化しない）
//* 注 RreverseMode で処理内容が変わる
inline void Voxel::setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, ReverseMode mode ){
  switch( mode ){
  case SIMPLE_REVERSE: 
    vr[ idx  ]  = r;
    vg[ idx  ]  = g;
    vb[ idx  ]  = b;
    _vr[ idx  ] = 255 - r;
    _vg[ idx  ] = 255 - g;
    _vb[ idx  ] = 255 - b;
    break;
    
  case TRIGONOMETRIC:
    vr[ idx  ]  = 255 * sin( r * angle_norm );
    vg[ idx  ]  = 255 * sin( g * angle_norm );
    vb[ idx  ]  = 255 * sin( b * angle_norm );
    _vr[ idx  ] = 255 * cos( r * angle_norm );
    _vg[ idx  ] = 255 * cos( g * angle_norm );
    _vb[ idx  ] = 255 * cos( b * angle_norm );
    break;

  default:
    cerr << "ERR (in Voxel::setRGB): unknown ReverseMode." << endl;
    exit( EXIT_FAILURE );
    break;
  }
}

//*********************************************
//* RGB の主成分値と補助成分値をセットする （二値化する）
inline void Voxel::setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b ){
  if( r > color_threshold_r ){
    vr[ idx ]  = 1;
    _vr[ idx ] = 0;
  }
  else{
    vr[ idx ]  = 0;
    _vr[ idx ] = 1;
  }
  if( g > color_threshold_g ){
    vg[ idx ]  = 1;
    _vg[ idx ] = 0;
  }
  else{
    vg[ idx ]  = 0;
    _vg[ idx ] = 1;
  }
  if( b > color_threshold_b ){
    vb[ idx ]  = 1;
    _vb[ idx ] = 0;
  }
  else{
    vb[ idx ]  = 0;
    _vb[ idx ] = 1;
  }
}

//********************************
//* 各メッシュのボクセル化 ファイルに出力
void Voxel::face2voxel_file( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step ){
  //* ボクセルのつくりかた
  //* 点1から点2まで、1ボクセルずつつくり、
  //* つくったらそこから＜点2→点3方向ベクトル＞*＜点から点までの距離に応じた長さ＞上のボクセルをつくる

  double x[3];
  double y[3];
  double z[3];
  double u[3];
  double v[3];
  x[0] = x1; x[1] = x2; x[2] = x3;
  y[0] = y1; y[1] = y2; y[2] = y3;
  z[0] = z1; z[1] = z2; z[2] = z3;
  u[0] = u1; u[1] = u2; u[2] = u3;
  v[0] = v1; v[1] = v2; v[2] = v3;

  double dx[3];
  double dy[3];
  double dz[3];
  double du[3];
  double dv[3];
  int voxel_num[3];
  for( int i=0; i<3; i++ ){
    if( i==2 ){
      dx[ 2 ] = x[ 0 ] - x[ 2 ];
      dy[ 2 ] = y[ 0 ] - y[ 2 ];
      dz[ 2 ] = z[ 0 ] - z[ 2 ];
      du[ 2 ] = u[ 0 ] - u[ 2 ];
      dv[ 2 ] = v[ 0 ] - v[ 2 ];
    }
    else{
      dx[ i ] = x[ i+1 ] - x[ i ];
      dy[ i ] = y[ i+1 ] - y[ i ];
      dz[ i ] = z[ i+1 ] - z[ i ];
      du[ i ] = u[ i+1 ] - u[ i ];
      dv[ i ] = v[ i+1 ] - v[ i ];
    }
    double tmp_norm = dx[ i ] * dx[ i ] + dy[ i ] * dy[ i ] + dz[ i ] * dz[ i ];
    tmp_norm = sqrt(tmp_norm);
    voxel_num[ i ] = (int)(tmp_norm/voxel_size)+1;
    dx[ i ] = dx[ i ]*voxel_size/tmp_norm;
    dy[ i ] = dy[ i ]*voxel_size/tmp_norm;
    dz[ i ] = dz[ i ]*voxel_size/tmp_norm;
    du[ i ] /= voxel_num[ i ];
    dv[ i ] /= voxel_num[ i ];
  }
  for( int i=0; i<3; i++ ){
    //* 点をプロット
    int ax = (int)(x[ i ]/voxel_size) + 1;
    int ay = (int)(y[ i ]/voxel_size) + 1;
    int az = (int)(z[ i ]/voxel_size) + 1;
    int image_x = (int)(width*u[ i ]);
    while(image_x<0) image_x += width;
    while(image_x>=width) image_x -= width;
    int image_y = (int)(height*v[ i ]);
    while(image_y<0) image_y += height;
    while(image_y>=height) image_y -= height;

    if(!exist_flag[ax + ay*xsize + az*xysize]){
      exist_flag[ax + ay*xsize + az*xysize] = true;
      unsigned char red= image[ 3 * image_x + width_step * image_y ];
      unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
      unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
      if( ascii )
	fprintf(fp_w,"%d %d %d %d %d %d\n",ax,ay,az,red,green,blue);
      else{
	fwrite(&ax,sizeof(int),1,fp_w);
	fwrite(&ay,sizeof(int),1,fp_w);
	fwrite(&az,sizeof(int),1,fp_w);
	fwrite(&red,sizeof(unsigned char),1,fp_w);
	fwrite(&green,sizeof(unsigned char),1,fp_w);
	fwrite(&blue,sizeof(unsigned char),1,fp_w);
      }
    }

    //* 点から次の点への直線をひく
    for(int b=1;b<voxel_num[ i ];b++){
      double bx_ = x[ i ]+dx[ i ]*b;
      double by_ = y[ i ]+dy[ i ]*b;
      double bz_ = z[ i ]+dz[ i ]*b;
      int bx = (int)(bx_/voxel_size) + 1;
      int by = (int)(by_/voxel_size) + 1;
      int bz = (int)(bz_/voxel_size) + 1;
      if( bx!=ax || by!=ay || bz!=az ){
	image_x = (int)(width*(u[ i ]+du[ i ]*b));
	while(image_x<0) image_x += width;
	while(image_x>=width) image_x -= width;
	image_y = (int)(height*(v[ i ]+dv[ i ]*b));
	while(image_y<0) image_y += height;
	while(image_y>=height) image_y -= height;

	if(!exist_flag[bx + by*xsize + bz*xysize]){
	  exist_flag[bx + by*xsize + bz*xysize] = true;		  
	  unsigned char red= image[ 3 * image_x + width_step * image_y ];
	  unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
	  unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
	  if( ascii )
	    fprintf(fp_w,"%d %d %d %d %d %d\n",bx,by,bz,red,green,blue);		  
	  else{
	    fwrite(&bx,sizeof(int),1,fp_w);
	    fwrite(&by,sizeof(int),1,fp_w);
	    fwrite(&bz,sizeof(int),1,fp_w);
	    fwrite(&red,sizeof(unsigned char),1,fp_w);
	    fwrite(&green,sizeof(unsigned char),1,fp_w);
	    fwrite(&blue,sizeof(unsigned char),1,fp_w);
	  }
	}
      }
      ax=bx;
      ay=by;
      az=bz;

      //* 先の直線上の各点をとおる、別の直線と平行な線上の点を埋めていく
      int tmp_num1 = i+1;
      if( i==2 ) tmp_num1 = 0;
      int ax2=bx;
      int ay2=by;
      int az2=bz;
      int tmp_num2 = (int)(b*voxel_num[ tmp_num1 ]/voxel_num[ i ]);
      for(int b2=1;b2<tmp_num2;b2++){
	int bx2 = (int)((bx_+dx[ tmp_num1 ]*b2)/voxel_size) + 1;
	int by2 = (int)((by_+dy[ tmp_num1 ]*b2)/voxel_size) + 1;
	int bz2 = (int)((bz_+dz[ tmp_num1 ]*b2)/voxel_size) + 1;
	if( bx2!=ax2 || by2!=ay2 || bz2!=az2 ){
	  image_x = (int)(width*(u[ i ]+du[ i ]*b+du[ tmp_num1 ]*b2));
	  while(image_x<0) image_x += width;
	  while(image_x>=width) image_x -= width;
	  image_y = (int)(height*(v[ i ]+dv[ i ]*b+dv[ tmp_num1 ]*b2));
	  while(image_y<0) image_y += height;
	  while(image_y>=height) image_y -= height;
	  
	  if(!exist_flag[bx2 + by2*xsize + bz2*xysize]){
	    exist_flag[bx2 + by2*xsize + bz2*xysize] = true;
	    unsigned char red= image[ 3 * image_x + width_step * image_y ];
	    unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
	    unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
	    if( ascii )
	      fprintf(fp_w,"%d %d %d %d %d %d\n",bx2,by2,bz2,red,green,blue);
	    else{
	      fwrite(&bx2,sizeof(int),1,fp_w);
	      fwrite(&by2,sizeof(int),1,fp_w);
	      fwrite(&bz2,sizeof(int),1,fp_w);
	      fwrite(&red,sizeof(unsigned char),1,fp_w);
	      fwrite(&green,sizeof(unsigned char),1,fp_w);
	      fwrite(&blue,sizeof(unsigned char),1,fp_w);
	    }
	  }
	}
	ax2=bx2;
	ay2=by2;
	az2=bz2;
      }
    }
  }
}

//********************************
//* 各メッシュのボクセル化 RGB2値化せず
void Voxel::face2voxel( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step, ReverseMode mode ){
  //* ボクセルのつくりかた
  //* 点1から点2まで、1ボクセルずつつくり、
  //* つくったらそこから＜点2→点3方向ベクトル＞*＜点から点までの距離に応じた長さ＞上のボクセルをつくる

  double x[3];
  double y[3];
  double z[3];
  double u[3];
  double v[3];
  x[0] = x1; x[1] = x2; x[2] = x3;
  y[0] = y1; y[1] = y2; y[2] = y3;
  z[0] = z1; z[1] = z2; z[2] = z3;
  u[0] = u1; u[1] = u2; u[2] = u3;
  v[0] = v1; v[1] = v2; v[2] = v3;

  double dx[3];
  double dy[3];
  double dz[3];
  double du[3];
  double dv[3];
  int voxel_num[3];
  for( int i=0; i<3; i++ ){
    if( i==2 ){
      dx[ 2 ] = x[ 0 ] - x[ 2 ];
      dy[ 2 ] = y[ 0 ] - y[ 2 ];
      dz[ 2 ] = z[ 0 ] - z[ 2 ];
      du[ 2 ] = u[ 0 ] - u[ 2 ];
      dv[ 2 ] = v[ 0 ] - v[ 2 ];
    }
    else{
      dx[ i ] = x[ i+1 ] - x[ i ];
      dy[ i ] = y[ i+1 ] - y[ i ];
      dz[ i ] = z[ i+1 ] - z[ i ];
      du[ i ] = u[ i+1 ] - u[ i ];
      dv[ i ] = v[ i+1 ] - v[ i ];
    }
    double tmp_norm = dx[ i ] * dx[ i ] + dy[ i ] * dy[ i ] + dz[ i ] * dz[ i ];
    tmp_norm = sqrt(tmp_norm);
    voxel_num[ i ] = (int)(tmp_norm/voxel_size)+1;
    dx[ i ] = dx[ i ]*voxel_size/tmp_norm;
    dy[ i ] = dy[ i ]*voxel_size/tmp_norm;
    dz[ i ] = dz[ i ]*voxel_size/tmp_norm;
    du[ i ] /= voxel_num[ i ];
    dv[ i ] /= voxel_num[ i ];
  }
  for( int i=0; i<3; i++ ){
    //* 点をプロット
    int ax = (int)(x[ i ]/voxel_size) + 1;
    int ay = (int)(y[ i ]/voxel_size) + 1;
    int az = (int)(z[ i ]/voxel_size) + 1;
    int image_x = (int)(width*u[ i ]);
    while(image_x<0) image_x += width;
    while(image_x>=width) image_x -= width;
    int image_y = (int)(height*v[ i ]);
    while(image_y<0) image_y += height;
    while(image_y>=height) image_y -= height;

    if(!exist_flag[ax + ay*xsize + az*xysize]){
      exist_flag[ax + ay*xsize + az*xysize] = true;
      unsigned char red= image[ 3 * image_x + width_step * image_y ];
      unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
      unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
      setRGB( az * xysize + ay * xsize + ax, red, green, blue, mode );
    }

    //* 点から次の点への直線をひく
    for(int b=1;b<voxel_num[ i ];b++){
      double bx_ = x[ i ]+dx[ i ]*b;
      double by_ = y[ i ]+dy[ i ]*b;
      double bz_ = z[ i ]+dz[ i ]*b;
      int bx = (int)(bx_/voxel_size) + 1;
      int by = (int)(by_/voxel_size) + 1;
      int bz = (int)(bz_/voxel_size) + 1;
      if( bx!=ax || by!=ay || bz!=az ){
	image_x = (int)(width*(u[ i ]+du[ i ]*b));
	while(image_x<0) image_x += width;
	while(image_x>=width) image_x -= width;
	image_y = (int)(height*(v[ i ]+dv[ i ]*b));
	while(image_y<0) image_y += height;
	while(image_y>=height) image_y -= height;

	if(!exist_flag[bx + by*xsize + bz*xysize]){
	  exist_flag[bx + by*xsize + bz*xysize] = true;		  
	  unsigned char red= image[ 3 * image_x + width_step * image_y ];
	  unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
	  unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
	  setRGB( bz * xysize + by * xsize + bx, red, green, blue, mode );
	}
      }
      ax=bx;
      ay=by;
      az=bz;

      //* 先の直線上の各点をとおる、別の直線と平行な線上の点を埋めていく
      int tmp_num1 = i+1;
      if( i==2 ) tmp_num1 = 0;
      int ax2=bx;
      int ay2=by;
      int az2=bz;
      int tmp_num2 = (int)(b*voxel_num[ tmp_num1 ]/voxel_num[ i ]);
      for(int b2=1;b2<tmp_num2;b2++){
	int bx2 = (int)((bx_+dx[ tmp_num1 ]*b2)/voxel_size) + 1;
	int by2 = (int)((by_+dy[ tmp_num1 ]*b2)/voxel_size) + 1;
	int bz2 = (int)((bz_+dz[ tmp_num1 ]*b2)/voxel_size) + 1;
	if( bx2!=ax2 || by2!=ay2 || bz2!=az2 ){
	  image_x = (int)(width*(u[ i ]+du[ i ]*b+du[ tmp_num1 ]*b2));
	  while(image_x<0) image_x += width;
	  while(image_x>=width) image_x -= width;
	  image_y = (int)(height*(v[ i ]+dv[ i ]*b+dv[ tmp_num1 ]*b2));
	  while(image_y<0) image_y += height;
	  while(image_y>=height) image_y -= height;
	  
	  if(!exist_flag[bx2 + by2*xsize + bz2*xysize]){
	    exist_flag[bx2 + by2*xsize + bz2*xysize] = true;
	    unsigned char red= image[ 3 * image_x + width_step * image_y ];
	    unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
	    unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
	    setRGB( bz2 * xysize + by2 * xsize + bx2, red, green, blue, mode );
	  }
	}
	ax2=bx2;
	ay2=by2;
	az2=bz2;
      }
    }
  }
}

//********************************
//* 各メッシュのボクセル化 RGB2値化する
void Voxel::face2voxel( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b ){
  //* ボクセルのつくりかた
  //* 点1から点2まで、1ボクセルずつつくり、
  //* つくったらそこから＜点2→点3方向ベクトル＞*＜点から点までの距離に応じた長さ＞上のボクセルをつくる

  double x[3];
  double y[3];
  double z[3];
  double u[3];
  double v[3];
  x[0] = x1; x[1] = x2; x[2] = x3;
  y[0] = y1; y[1] = y2; y[2] = y3;
  z[0] = z1; z[1] = z2; z[2] = z3;
  u[0] = u1; u[1] = u2; u[2] = u3;
  v[0] = v1; v[1] = v2; v[2] = v3;

  double dx[3];
  double dy[3];
  double dz[3];
  double du[3];
  double dv[3];
  int voxel_num[3];
  for( int i=0; i<3; i++ ){
    if( i==2 ){
      dx[ 2 ] = x[ 0 ] - x[ 2 ];
      dy[ 2 ] = y[ 0 ] - y[ 2 ];
      dz[ 2 ] = z[ 0 ] - z[ 2 ];
      du[ 2 ] = u[ 0 ] - u[ 2 ];
      dv[ 2 ] = v[ 0 ] - v[ 2 ];
    }
    else{
      dx[ i ] = x[ i+1 ] - x[ i ];
      dy[ i ] = y[ i+1 ] - y[ i ];
      dz[ i ] = z[ i+1 ] - z[ i ];
      du[ i ] = u[ i+1 ] - u[ i ];
      dv[ i ] = v[ i+1 ] - v[ i ];
    }
    double tmp_norm = dx[ i ] * dx[ i ] + dy[ i ] * dy[ i ] + dz[ i ] * dz[ i ];
    tmp_norm = sqrt(tmp_norm);
    voxel_num[ i ] = (int)(tmp_norm/voxel_size)+1;
    dx[ i ] = dx[ i ]*voxel_size/tmp_norm;
    dy[ i ] = dy[ i ]*voxel_size/tmp_norm;
    dz[ i ] = dz[ i ]*voxel_size/tmp_norm;
    du[ i ] /= voxel_num[ i ];
    dv[ i ] /= voxel_num[ i ];
  }
  for( int i=0; i<3; i++ ){
    //* 点をプロット
    int ax = (int)(x[ i ]/voxel_size) + 1;
    int ay = (int)(y[ i ]/voxel_size) + 1;
    int az = (int)(z[ i ]/voxel_size) + 1;
    int image_x = (int)(width*u[ i ]);
    while(image_x<0) image_x += width;
    while(image_x>=width) image_x -= width;
    int image_y = (int)(height*v[ i ]);
    while(image_y<0) image_y += height;
    while(image_y>=height) image_y -= height;

    if(!exist_flag[ax + ay*xsize + az*xysize]){
      exist_flag[ax + ay*xsize + az*xysize] = true;
      unsigned char red= image[ 3 * image_x + width_step * image_y ];
      unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
      unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
      setRGB( az * xysize + ay * xsize + ax, red, green, blue, color_threshold_r, color_threshold_g, color_threshold_b );
    }

    //* 点から次の点への直線をひく
    for(int b=1;b<voxel_num[ i ];b++){
      double bx_ = x[ i ]+dx[ i ]*b;
      double by_ = y[ i ]+dy[ i ]*b;
      double bz_ = z[ i ]+dz[ i ]*b;
      int bx = (int)(bx_/voxel_size) + 1;
      int by = (int)(by_/voxel_size) + 1;
      int bz = (int)(bz_/voxel_size) + 1;
      if( bx!=ax || by!=ay || bz!=az ){
	image_x = (int)(width*(u[ i ]+du[ i ]*b));
	while(image_x<0) image_x += width;
	while(image_x>=width) image_x -= width;
	image_y = (int)(height*(v[ i ]+dv[ i ]*b));
	while(image_y<0) image_y += height;
	while(image_y>=height) image_y -= height;

	if(!exist_flag[bx + by*xsize + bz*xysize]){
	  exist_flag[bx + by*xsize + bz*xysize] = true;		  
	  unsigned char red= image[ 3 * image_x + width_step * image_y ];
	  unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
	  unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
	  setRGB( bz * xysize + by * xsize + bx, red, green, blue, color_threshold_r, color_threshold_g, color_threshold_b );
	}
      }
      ax=bx;
      ay=by;
      az=bz;

      //* 先の直線上の各点をとおる、別の直線と平行な線上の点を埋めていく
      int tmp_num1 = i+1;
      if( i==2 ) tmp_num1 = 0;
      int ax2=bx;
      int ay2=by;
      int az2=bz;
      int tmp_num2 = (int)(b*voxel_num[ tmp_num1 ]/voxel_num[ i ]);
      for(int b2=1;b2<tmp_num2;b2++){
	int bx2 = (int)((bx_+dx[ tmp_num1 ]*b2)/voxel_size) + 1;
	int by2 = (int)((by_+dy[ tmp_num1 ]*b2)/voxel_size) + 1;
	int bz2 = (int)((bz_+dz[ tmp_num1 ]*b2)/voxel_size) + 1;
	if( bx2!=ax2 || by2!=ay2 || bz2!=az2 ){
	  image_x = (int)(width*(u[ i ]+du[ i ]*b+du[ tmp_num1 ]*b2));
	  while(image_x<0) image_x += width;
	  while(image_x>=width) image_x -= width;
	  image_y = (int)(height*(v[ i ]+dv[ i ]*b+dv[ tmp_num1 ]*b2));
	  while(image_y<0) image_y += height;
	  while(image_y>=height) image_y -= height;
	  
	  if(!exist_flag[bx2 + by2*xsize + bz2*xysize]){
	    exist_flag[bx2 + by2*xsize + bz2*xysize] = true;
	    unsigned char red= image[ 3 * image_x + width_step * image_y ];
	    unsigned char green= image[ 3 * image_x + width_step * image_y + 1 ];
	    unsigned char blue= image[ 3 * image_x + width_step * image_y + 2 ];
	    setRGB( bz2 * xysize + by2 * xsize + bx2, red, green, blue, color_threshold_r, color_threshold_g, color_threshold_b );
	  }
	}
	ax2=bx2;
	ay2=by2;
	az2=bz2;
      }
    }
  }
}
