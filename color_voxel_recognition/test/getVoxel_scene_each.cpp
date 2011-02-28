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
/* �ƥܥ�����ǡ������̸ĤΥե��������¸�����             */
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
  pcl::PointCloud<pcl::PointXYZRGB> cloud;
  for(int i=0;i<obj_num;i++){
    sprintf( tmpname, "scene/Voxel/%03d.dat", i ); 
    Voxel voxel( tmpname, file_mode );
    voxel.setVoxelSize( voxel_size );
    sprintf( tmpname, "scene/Points/%03d.pcd", i );
    pcl::io::loadPCDFile (tmpname, cloud);
    voxel.points2voxel( cloud, SIMPLE_REVERSE );
    voxel.writeVoxel();
  }
#else
  //* ��å���ǡ������ɤ߹���, �ܥ������Ѵ�, ��̤���¸
  const int obj_num = atoi(argv[1]); // �ɤ߹����å���ǡ����ο�
  for( int i=0; i<obj_num; i++ ){
    Obj object;
    sprintf( tmpname, "scene/Obj/%03d.obj", i );
    object.readMesh( tmpname );

    sprintf( tmpname, "scene/Voxel/%03d.dat", i );    
    if( ASCII_MODE_V ) sprintf( file_mode, "w" );
    else sprintf( file_mode, "wb" );
    Voxel voxel( tmpname, file_mode );
    voxel.setVoxelSize( voxel_size );
    voxel.setMinMax( object.x_min, object.x_max, object.y_min, object.y_max, object.z_min, object.z_max );
    
    sprintf( tmpname, "scene/Textures/%03d.ppm", i );
    voxel.obj2voxel_file( object, tmpname );
  }
#endif

  return(0);    
}
