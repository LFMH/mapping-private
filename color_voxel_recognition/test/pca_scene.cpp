#include <octave/config.h>
#include <octave/Matrix.h>

#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include "../param/FILE_MODE"

/********************************************************************/
/* �Ķ����Τ�ʬ�䤷�����Ƥ���ʬ�ΰ褫��Color-CHLAC��ħ��Ȥ����ʬʬ�Ϥ���      */
/* ����������Color-CHLAC��ħ�٥��ȥ�ΰ��̤��Ѥ���                      */
/* �ʴĶ��������ħ��Ф����ˤ˹Ԥ����Ͼ�ά��ǽ���ǽ�˵�᤿��������Ѥ��Ƥ褤�� */
/* ��PCA-SIFT�μ��Τ褦�ʤ�Ρ�                                         */
/********************************************************************/

using namespace std;

int main(int argc, char** argv){

  //* ʬ���ΰ���礭�����ɤ߹���
  const int box_size = Param::readBoxSize_scene();

  //* ���٤˺�������ܥ�������ξ�¤��ɤ߹���
  const int max_voxel_num = Param::readMaxVoxelNum();

  //* �ܥ�����ǡ������礭���Τ� �ɤ߹���
  int xsize, ysize, zsize;
  FILE *fp;
  if( ASCII_MODE_V ){
    fp = fopen("scene/voxel_scene.dat","r");
    fscanf(fp,"%d %d %d\n",&xsize,&ysize,&zsize);
  }
  else{
    fp = fopen("scene/voxel_scene.dat","rb");
    fread(&xsize,sizeof(int),1,fp);
    fread(&ysize,sizeof(int),1,fp);
    fread(&zsize,sizeof(int),1,fp);
  }
  fclose(fp);

  //* RGB���Ͳ������ͤ��ɤ߹���
  int color_threshold_r, color_threshold_g, color_threshold_b;
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );

  //* ʬ���ΰ�θĿ���Ĵ�٤�
  int x_num = xsize/box_size;
  if(xsize%box_size > 0)
    x_num++;
  int y_num = ysize/box_size;
  if(ysize%box_size > 0)
    y_num++;
  int z_num = zsize/box_size;
  if(zsize%box_size > 0)
    z_num++;

  //* �����餯���꤬­��ʤ��ʤ�Τǡ�ʣ�����ʬ���ƥܥ����벽��CCHLAC��Ф�ԤäƤ����ޤ�
  int z_num_middle = max_voxel_num / ( x_num * box_size * y_num * box_size * box_size );
  int z_num_num = z_num / z_num_middle; // �Ǹ�Υ롼�פΰ�����
  int zsize_middle = z_num_middle*box_size;
  if( z_num_middle >= z_num ){ // �Ķ�����ʬ�˾������ä���
    z_num_middle = z_num;
    z_num_num = 0;
    zsize_middle = zsize;
  }

  //* �ܥ������ΰ�γ���
  Voxel voxel( ASCII_MODE_V );     // RGB���Ͳ����ʤ�������ħ�����
  Voxel voxel_bin( ASCII_MODE_V ); // RGB���Ͳ����������ħ�����
  voxel.setXYZsize( xsize, ysize, zsize_middle+1 );
  voxel_bin.setXYZsize( xsize, ysize, zsize_middle+1 );
  voxel.createVoxelData();
  voxel_bin.createVoxelData();

  //*********************************//
  //* ���Ƥ�ʬ���ΰ褫���CCHLAC��ħ��� *//
  //*********************************//

  PCA pca( false ); // ��ħ�٥��ȥ뤫��ʿ�ѥ٥��ȥ��Ҥ��ʤ���ɬ��false�ˤ��뤳��
  ColumnVector feature(DIM_COLOR_1_3+DIM_COLOR_BIN_1_3);
  ColumnVector feature1; // RGB���Ͳ����ʤ�CCHLAC��ħ
  ColumnVector feature2; // RGB���Ͳ�����CCHLAC��ħ

  for(int tmp_num=0; tmp_num<z_num_num+1; tmp_num++){
    printf("%d in %d...\n",tmp_num,z_num_num);

    //* �ܥ�����ǡ������ɤ߹���
    voxel.cleanVoxelData();
    voxel_bin.cleanVoxelData();
    voxel.readVoxelZoffset( "scene/voxel_scene.dat", zsize_middle*tmp_num, REVERSE_MODE, false );
    voxel_bin.readVoxelZoffset( "scene/voxel_scene.dat", zsize_middle*tmp_num, color_threshold_r, color_threshold_g, color_threshold_b, false );

    int sx, sy, sz, gx, gy, gz;
    int zend = z_num_middle;
    if( tmp_num == z_num_num ) zend = z_num-z_num_num*z_num_middle;
    for(int z=0;z<zend;z++){
      printf("  %d in %d...\n",z,zend);
      for(int y=0;y<y_num;y++){
	for(int x=0;x<x_num;x++){
	  sx = x*box_size+1;
	  sy = y*box_size+1;
	  sz = z*box_size+1;
	  gx = sx+box_size;
	  gy = sy+box_size;
	  gz = sz+box_size;
	  if(gx>xsize-1) gx = xsize-1;
	  if(gy>ysize-1) gy = ysize-1;
	  if(gz>zsize_middle+1) gz = zsize_middle+1;

	  //* CCHLAC��ħ���
	  CCHLAC::extractColorCHLAC( feature1, voxel, sx, sy, sz, gx, gy, gz );
	  bool exist_flg = false;
	  for(int t=0;t<6;t++){
	    if(feature1(t)>0){
	      exist_flg = true;
	      break;
	    }
	  }
	  if(exist_flg){ // �ܥ�����Τʤ����ΰ�Ǥʤ���С�=CCHLAC��ħ����٥��ȥ�Ǥʤ���С�
	    CCHLAC::extractColorCHLAC_bin( feature2, voxel_bin, sx, sy, sz, gx, gy, gz );
	    for(int t=0;t<DIM_COLOR_1_3;t++)
	      feature(t) = feature1(t);
	    for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
	      feature(t+DIM_COLOR_1_3) = feature2(t);
	    pca.addData( feature ); // PCA��򤯽���
	  }
	}
      }
    }
  }

  //* PCA��򤤤Ʒ�̤���¸
  pca.solve();
  pca.write( "scene/pca_result", ASCII_MODE_P );

  return 0;
}
