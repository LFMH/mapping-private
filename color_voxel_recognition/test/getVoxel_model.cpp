#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>

#include <color_voxel_recognition/objFile.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include "../param/FILE_MODE"

#define PCL_MODE
#ifdef PCL_MODE
#include <pcl/io/pcd_io.h>
#endif

/*****************************************************************/
/* 検出対象物体の計測データ（複数方向から取得したもの）を様々な姿勢でボクセル化 */
/* 各ボクセルデータは別個のファイルに保存される                           */
/* 姿勢は、3軸に関して(90/rotate_num)度ずつ回転を加えたもの              */
/*  （例） rotate_num = 3: 30度ずつ 21姿勢                          */
/* なお、バウンディングボックスのサイズをsize.txtに保存する                */
/*****************************************************************/

using namespace std;

//* バウンディングボックスのサイズ決定のための関数
void setSize( float &s1, float &s2, float &s3, const float val ){
  if( val > s1 ){
    s3 = s2; s2 = s1; s1 = val;
  }
  else if( val > s2 ){
    s3 = s2; s2 = val;
  }
  else if( val > s3 ){
    s3 = val;
  }
}

#ifdef PCL_MODE
template <typename T>
void getMinMax_points( const T& input_cloud, float &val1, float &val2, float &val3 ){
  float x_min = FLT_MAX, y_min = FLT_MAX, z_min = FLT_MAX;
  float x_max = - FLT_MAX, y_max = - FLT_MAX, z_max = - FLT_MAX;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    if( x_min > input_cloud.points[ i ].x ) x_min = input_cloud.points[ i ].x;
    if( y_min > input_cloud.points[ i ].y ) y_min = input_cloud.points[ i ].y;
    if( z_min > input_cloud.points[ i ].z ) z_min = input_cloud.points[ i ].z;
    if( x_max < input_cloud.points[ i ].x ) x_max = input_cloud.points[ i ].x;
    if( y_max < input_cloud.points[ i ].y ) y_max = input_cloud.points[ i ].y;
    if( z_max < input_cloud.points[ i ].z ) z_max = input_cloud.points[ i ].z;
  }
  val1 = x_max - x_min;
  val2 = y_max - y_min;
  val3 = z_max - z_min;
}

template <typename T>
bool rotatePoints( const T& input_cloud, T& output_cloud, const double roll, const double pan, const double roll2 ){
  output_cloud = input_cloud;
  double R1[9];
  R1[0]=cos(roll);
  R1[1]=-sin(roll);
  R1[2]=0;
  R1[3]=sin(roll);
  R1[4]=cos(roll);
  R1[5]=0;
  R1[6]=0;
  R1[7]=0;
  R1[8]=1;
  double R2[9];
  R2[0]=cos(pan);
  R2[1]=0;
  R2[2]=sin(pan);
  R2[3]=0;
  R2[4]=1;
  R2[5]=0;
  R2[6]=-sin(pan);
  R2[7]=0;
  R2[8]=cos(pan);
  double R3[9];
  R3[0]=cos(roll2);
  R3[1]=-sin(roll2);
  R3[2]=0;
  R3[3]=sin(roll2);
  R3[4]=cos(roll2);
  R3[5]=0;
  R3[6]=0;
  R3[7]=0;
  R3[8]=1;

  float x1, y1, z1, x2, y2, z2;
  const int v_num = input_cloud.points.size();
  for( int i=0; i<v_num; i++ ){
    x1 = R1[0] * input_cloud.points[ i ].x + R1[1] * input_cloud.points[ i ].y + R1[2] * input_cloud.points[ i ].z;
    y1 = R1[3] * input_cloud.points[ i ].x + R1[4] * input_cloud.points[ i ].y + R1[5] * input_cloud.points[ i ].z;
    z1 = R1[6] * input_cloud.points[ i ].x + R1[7] * input_cloud.points[ i ].y + R1[8] * input_cloud.points[ i ].z;
    
    x2 = R2[0]*x1 + R2[1]*y1 + R2[2]*z1;
    y2 = R2[3]*x1 + R2[4]*y1 + R2[5]*z1;
    z2 = R2[6]*x1 + R2[7]*y1 + R2[8]*z1;
	    
    output_cloud.points[ i ].x = R3[0]*x2 + R3[1]*y2 + R3[2]*z2;
    output_cloud.points[ i ].y = R3[3]*x2 + R3[4]*y2 + R3[5]*z2;
    output_cloud.points[ i ].z = R3[6]*x2 + R3[7]*y2 + R3[8]*z2;
  }

  return(1);
}
#endif

int main(int argc, char **argv)
{
  if( argc != 3 ){
    cerr << "usage: " << argv[0] << " [model_name] <registration_num>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 100 ];
  char file_mode[ 3 ];

  //* バウンディングボックスの3辺の長さ（初期化）
  float size1 = 0;
  float size2 = 0;
  float size3 = 0;
  float vals[3];

  //* rotate_numの読み込み
  const int rotate_num = Param::readRotateNum();

  //* ボクセルの一辺の長さ（mm）の読み込み
  const float voxel_size = Param::readVoxelSize();

#ifdef PCL_MODE
  const int obj_num = atoi(argv[2]);
  int write_count = 0;         // 出力するボクセルファイルの番号
  for( int i=0; i<obj_num; i++ ){
    pcl::PointCloud<pcl::PointXYZRGB> ref_cloud;
    sprintf( tmpname, "models/%s/Points/%03d.pcd", argv[1], i );
    pcl::io::loadPCDFile (tmpname, ref_cloud);

    //* T姿勢に回転させ、それぞれの姿勢でのボクセルデータを出力
    //* T姿勢： 3軸に関して(90/rotate_num)度ずつ回転を加えた姿勢
    for(int r3=0; r3 < rotate_num; r3++){
      for(int r2=0; r2 < rotate_num; r2++){
	for(int r1=0; r1 < rotate_num; r1++){
	  const double roll  = r3 * M_PI / (2*rotate_num);
	  const double pan   = r2 * M_PI / (2*rotate_num);
	  const double roll2 = r1 * M_PI / (2*rotate_num);
	  
	  sprintf( tmpname, "models/%s/Voxel/%03d.dat", argv[1], write_count++ );

	  if( ASCII_MODE_V ) sprintf( file_mode, "w" );
	  else sprintf( file_mode, "wb" );
	  Voxel voxel( tmpname, file_mode );
	  voxel.setVoxelSize( voxel_size );
	  
	  pcl::PointCloud<pcl::PointXYZRGB> cloud;
	  rotatePoints( ref_cloud, cloud, roll, pan, roll2 ); // 回転させる
	  voxel.points2voxel( cloud, SIMPLE_REVERSE );
	  voxel.writeVoxel();

	  //* バウンディングボックスのサイズ決定（Maxなものを探す）
	  getMinMax_points( cloud, vals[0], vals[1], vals[2] );
	  float tmp_size1 = 0;
	  float tmp_size2 = 0;
	  float tmp_size3 = 0;
	  for( int j=0; j<3; j++ )
	    setSize( tmp_size1, tmp_size2, tmp_size3, vals[ j ] );
	  if( tmp_size1 > size1 ) size1 = tmp_size1;
	  if( tmp_size2 > size2 ) size2 = tmp_size2;
	  if( tmp_size3 > size3 ) size3 = tmp_size3;
	  
	  if(r2==0) break;
	}
      }
    }
  }
#else
  //* メッシュデータの読み込み, ボクセル変換, 結果を保存
  const int obj_num = atoi(argv[2]); // 読み込むメッシュデータの数
  int write_count = 0;         // 出力するボクセルファイルの番号
  for( int i=0; i<obj_num; i++ ){
    Obj ref_object;
    sprintf( tmpname, "models/%s/Obj/%03d.obj", argv[1], i );
    ref_object.readMesh( tmpname );

    //* T姿勢に回転させ、それぞれの姿勢でのボクセルデータを出力
    //* T姿勢： 3軸に関して(90/rotate_num)度ずつ回転を加えた姿勢
    for(int r3=0; r3 < rotate_num; r3++){
      for(int r2=0; r2 < rotate_num; r2++){
	for(int r1=0; r1 < rotate_num; r1++){
	  const double roll  = r3 * M_PI / (2*rotate_num);
	  const double pan   = r2 * M_PI / (2*rotate_num);
	  const double roll2 = r1 * M_PI / (2*rotate_num);
	  
	  sprintf( tmpname, "models/%s/Voxel/%03d.dat", argv[1], write_count++ );

	  if( ASCII_MODE_V ) sprintf( file_mode, "w" );
	  else sprintf( file_mode, "wb" );
	  Voxel voxel( tmpname, file_mode );
	  voxel.setVoxelSize( voxel_size );
	  
	  Obj object;
	  object.rotateMesh( ref_object, roll, pan, roll2 ); // メッシュを回転させる
	  voxel.setMinMax( object.x_min, object.x_max, object.y_min, object.y_max, object.z_min, object.z_max );
	  
	  sprintf( tmpname, "models/%s/Textures/%03d.ppm", argv[1], i );
	  voxel.obj2voxel_file( object, tmpname );

	  //* バウンディングボックスのサイズ決定（Maxなものを探す）
	  vals[0] = object.x_max - object.x_min;
	  vals[1] = object.y_max - object.y_min;
	  vals[2] = object.z_max - object.z_min;
	  float tmp_size1 = 0;
	  float tmp_size2 = 0;
	  float tmp_size3 = 0;
	  for( int j=0; j<3; j++ )
	    setSize( tmp_size1, tmp_size2, tmp_size3, vals[ j ] );
	  if( tmp_size1 > size1 ) size1 = tmp_size1;
	  if( tmp_size2 > size2 ) size2 = tmp_size2;
	  if( tmp_size3 > size3 ) size3 = tmp_size3;
	  
	  if(r2==0) break;
	}
      }
    }
  }
#endif
  //* バウンディングボックスのサイズ書き込み
  sprintf( tmpname, "models/%s/size.txt", argv[1] );
  FILE *fp = fopen( tmpname, "w" );
  fprintf( fp, "%f %f %f\n", size1, size2, size3 );
  fclose( fp );

  return(0);    
}
