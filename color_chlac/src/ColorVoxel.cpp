#include <iostream>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <assert.h>
#include <float.h>

#include "color_chlac/ColorVoxel.hpp"

using namespace std;

const double angle_norm = M_PI / 510;

//******************************//
//* Constructor and Destructor *//
//******************************//

ColorVoxel::ColorVoxel() :
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

ColorVoxel::ColorVoxel( const ColorVoxel &another ){
  *this = another;
}

ColorVoxel::~ColorVoxel(){ 
  if( exist_num != NULL ) delete[] exist_num;
  if( vr != NULL ) delete[] vr;
  if( vg != NULL ) delete[] vg;
  if( vb != NULL ) delete[] vb;
  if( _vr != NULL ) delete[] _vr;
  if( _vg != NULL ) delete[] _vg;
  if( _vb != NULL ) delete[] _vb;
}

//*******************************************
//* set the length of the side of each voxel
void ColorVoxel::setVoxelSize( float val ){ voxel_size = val; }

//****************************************************
//* set the minimum and maximum xyz valuse of 3D data
void ColorVoxel::setMinMax(){
  if( voxel_size == 0 ){
    cerr<< "ERR (in ColorVoxel::setMinMax): Set voxel size!" <<endl;
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

  if( exist_num != NULL ) delete[] exist_num;
  exist_num = new int[ xyzsize ];
  for(int i=0;i<xyzsize;i++)
    if( exist_num[i] ) exist_num[i] = 0;
}

//****************************************
//* transform point cloud into voxel data
void ColorVoxel::points2voxel( pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster, ReverseMode mode ){

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
    red[ idx ] += cloud_object_cluster.points[i].rgb;   // TODO: change it to the actual R value
    green[ idx ] += cloud_object_cluster.points[i].rgb; // TODO: change it to the actual G value
    blue[ idx ] += cloud_object_cluster.points[i].rgb;  // TODO: change it to the actual B value
  }

  for( int i=0; i<xyzsize; i++ )
    if( exist_num[ i ] != 0 )
      setRGB( i, red[ i ]/exist_num[ i ], green[ i ]/exist_num[ i ], blue[ i ]/exist_num[ i ], mode );
  
  delete[] red;
  delete[] green;
  delete[] blue;
}

//**********************
//* binarize RGB values
void ColorVoxel::binarize( unsigned char thR, unsigned char thG, unsigned char thB ){
  for( int idx = 0; idx < xyzsize; idx++ )
    if( (vr[ idx ]!=0)||(_vr[ idx ]!=0) )
      setRGB( idx, vr[ idx ], vg[ idx ], vb[ idx ], thR, thG, thB );
}

//************************
//* allocate voxel memory
void ColorVoxel::createVoxelData(){
  if( vr != NULL ) delete[] vr;
  if( vg != NULL ) delete[] vg;
  if( vb != NULL ) delete[] vb;
  if( _vr != NULL ) delete[] _vr;
  if( _vg != NULL ) delete[] _vg;
  if( _vb != NULL ) delete[] _vb;
  //if( exist_num != NULL ) delete[] exist_num;
  //exist_num = NULL;
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

//*******************************
//* initialize voxel values by 0
void ColorVoxel::cleanVoxelData(){
  for( int i=0; i<xyzsize; i++ ){
    vr[ i ] = 0;
    vg[ i ] = 0;
    vb[ i ] = 0;
    _vr[ i ] = 0;
    _vg[ i ] = 0;
    _vb[ i ] = 0;
  }
}

const ColorVoxel& ColorVoxel::operator=(const ColorVoxel &another) {
  // destruct
  bool reuse_memory = (xyzsize == another.xyzsize);

  if (!reuse_memory) {
    if( exist_num != NULL ) { delete[] exist_num; exist_num = NULL; }
    if( vr != NULL ) { delete[] vr; vr = NULL; }
    if( vg != NULL ) { delete[] vg; vg = NULL; }
    if( vb != NULL ) { delete[] vb; vb = NULL; }
    if( _vr != NULL ) { delete[] _vr; _vr = NULL; }
    if( _vg != NULL ) { delete[] _vg; _vg = NULL; }
    if( _vb != NULL ) { delete[] _vb; _vb = NULL; }
  }
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
      exist_num = NULL;
      vr = NULL;
      vg = NULL;
      vb = NULL;
      _vr = NULL;
      _vg = NULL;
      _vb = NULL;
    } else {
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

//*********************//
//* private functions *//
//*********************//

//***********************************************************
//* set RGB values and inverse-RGB values (without binarize)
//* ! Note that TRIGONOMETRIC mode is not available before my paper is published.
inline void ColorVoxel::setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, ReverseMode mode ){
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
    cerr << "ERR (in ColorVoxel::setRGB): TRIGONOMETRIC mode is not available. Please use SIMPLE_REVERSE mode." << endl;
    exit( EXIT_FAILURE );
//     vr[ idx  ]  = 255 * sin( r * angle_norm );
//     vg[ idx  ]  = 255 * sin( g * angle_norm );
//     vb[ idx  ]  = 255 * sin( b * angle_norm );
//     _vr[ idx  ] = 255 * cos( r * angle_norm );
//     _vg[ idx  ] = 255 * cos( g * angle_norm );
//     _vb[ idx  ] = 255 * cos( b * angle_norm );
    break;

  default:
    cerr << "ERR (in ColorVoxel::setRGB): unknown ReverseMode." << endl;
    exit( EXIT_FAILURE );
    break;
  }
}

//*******************************************************
//* set RGB values and inverse-RGB values (with binarize)
inline void ColorVoxel::setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b ){
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
