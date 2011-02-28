#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>

#include <color_voxel_recognition/objFile.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include "./FILE_MODE"

#define PCL_MODE
#ifdef PCL_MODE
#include <pcl/io/pcd_io.h>
#endif

/***************************************************/
/* �Ķ��η�¬�ǡ�����ʣ�������������������Ρˤ�ܥ����벽   */
/* ��Ĥ��礭�ʥܥ�����ǡ����Ȥ���ñ��Υե��������¸�����  */
/***************************************************/

using namespace std;

int main(int argc, char **argv)
{
  if( argc != 2 ){
    cerr << "usage: " << argv[0] << " <registration_num>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 100 ];
  char file_mode[ 3 ];

  //* �ܥ�����ΰ��դ�Ĺ����mm�ˤ��ɤ߹���
  const float voxel_size = Param::readVoxelSize();

#ifdef PCL_MODE
  const int obj_num = atoi(argv[1]);
  if( ASCII_MODE_V ) sprintf( file_mode, "w" );
  else sprintf( file_mode, "wb" );
  Voxel voxel( "scene/voxel_scene.dat", file_mode );
  voxel.setVoxelSize( voxel_size );
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for(int i=0;i<obj_num;i++){
    sprintf( tmpname, "scene/Points/%03d.pcd", i );
    pcl::io::loadPCDFile (tmpname, cloud);
    voxel.points2voxel( cloud, SIMPLE_REVERSE );
    voxel.writeVoxel();
  }
#else
  //* ��å���ǡ������ɤ߹���
  const int obj_num = atoi(argv[1]); // �ɤ߹����å���ǡ����ο�
  Obj *object = new Obj[ obj_num ];
  float x_min = FLT_MAX;
  float x_max = - FLT_MAX;
  float y_min = FLT_MAX;
  float y_max = - FLT_MAX;
  float z_min = FLT_MAX;
  float z_max = - FLT_MAX;
  for( int i=0; i<obj_num; i++ ){
    sprintf( tmpname, "scene/Obj/%03d.obj", i );
    object[ i ].readMesh( tmpname );
    if( object[ i ].x_min < x_min )	x_min = object[ i ].x_min;
    if( object[ i ].x_max > x_max )	x_max = object[ i ].x_max;
    if( object[ i ].y_min < y_min )	y_min = object[ i ].y_min;
    if( object[ i ].y_max > y_max )	y_max = object[ i ].y_max;
    if( object[ i ].z_min < z_min )	z_min = object[ i ].z_min;
    if( object[ i ].z_max > z_max )	z_max = object[ i ].z_max;
  }

  //* ��å��夫��ܥ�������Ѵ�������̤���¸
  if( ASCII_MODE_V ) sprintf( file_mode, "w" );
  else sprintf( file_mode, "wb" );
  Voxel voxel( "scene/voxel_scene.dat", file_mode );
  voxel.setVoxelSize( voxel_size );
  voxel.setMinMax( x_min, x_max, y_min, y_max, z_min, z_max );
  for(int i=0;i<obj_num;i++){
    sprintf( tmpname, "scene/Textures/%03d.ppm", i );
    voxel.obj2voxel_file( object[ i ], tmpname );
  }
#endif

  return(0);    
}
