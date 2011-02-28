#include <iostream>
#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include "./FILE_MODE"

/********************************************************************/
/* �Ķ����Τ�ʬ�䤷�����Ƥ���ʬ�ΰ褫��Color-CHLAC��ħ��Ȥ����ʬʬ�Ϥ���      */
/* ����������Color-CHLAC��ħ�٥��ȥ�ΰ��̤��Ѥ���                      */
/* �ʴĶ��������ħ��Ф����ˤ˹Ԥ����Ͼ�ά��ǽ���ǽ�˵�᤿��������Ѥ��Ƥ褤�� */
/* ��PCA-SIFT�μ��Τ褦�ʤ�Ρ�                                         */
/* Voxel/�ե�����ʲ���ʣ���ե�������оݤȤ���                             */
/********************************************************************/

using namespace std;

int main(int argc, char** argv){
  if( argc != 2 ){
    cerr << "usage: " << argv[0] << " <registration_num>" << endl;
    exit( EXIT_FAILURE );
  }
  int file_num = atoi( argv[1] );

  //* ʬ���ΰ���礭�����ɤ߹���
  const int box_size = Param::readBoxSize_scene();

  //* RGB���Ͳ������ͤ��ɤ߹���
  int color_threshold_r, color_threshold_g, color_threshold_b;
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );
  //cout << color_threshold_r << " " << color_threshold_g << " " << color_threshold_b << endl;

  PCA pca( false ); // ��ħ�٥��ȥ뤫��ʿ�ѥ٥��ȥ��Ҥ��ʤ���ɬ��false�ˤ��뤳��
  //ColumnVector feature(DIM_COLOR_1_3+DIM_COLOR_BIN_1_3);
  std::vector<float> feature(DIM_COLOR_1_3+DIM_COLOR_BIN_1_3);
  std::vector<float> feature1; // RGB���Ͳ����ʤ�CCHLAC��ħ
  std::vector<float> feature2; // RGB���Ͳ�����CCHLAC��ħ

  int xsize, ysize, zsize;
  FILE *fp;
  char tmpname[ 100 ];

  for( int i=0; i<file_num; i++ ){
    if( ASCII_MODE_V ){
      sprintf( tmpname, "scene/Voxel/%03d.dat", i );
      fp = fopen( tmpname,"r" );
      fscanf(fp,"%d %d %d\n",&xsize,&ysize,&zsize);
    }
    else{
      sprintf( tmpname, "scene/Voxel/%03d.dat", i );
      fp = fopen( tmpname,"rb" );
      fread(&xsize,sizeof(int),1,fp);
      fread(&ysize,sizeof(int),1,fp);
      fread(&zsize,sizeof(int),1,fp);
    }

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
    
    //* �ܥ������ΰ�γ���
    Voxel voxel( ASCII_MODE_V );     // RGB���Ͳ����ʤ�������ħ�����
    Voxel voxel_bin( ASCII_MODE_V ); // RGB���Ͳ����������ħ�����
    voxel.setXYZsize( xsize, ysize, zsize );
    voxel_bin.setXYZsize( xsize, ysize, zsize );
    voxel.createVoxelData();
    voxel_bin.createVoxelData();
    
    //*********************************//
    //* ���Ƥ�ʬ���ΰ褫���CCHLAC��ħ��� *//
    //*********************************//
    
    //* �ܥ�����ǡ������ɤ߹���
    voxel.cleanVoxelData();
    voxel_bin.cleanVoxelData();
    sprintf( tmpname, "scene/Voxel/%03d.dat", i );
    voxel.readVoxel( tmpname, REVERSE_MODE, false );
    voxel_bin.readVoxel( tmpname, color_threshold_r, color_threshold_g, color_threshold_b, false );
    
    int sx, sy, sz, gx, gy, gz;
    for(int z=0;z<z_num;z++){
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
	  if(gz>zsize-1) gz = zsize-1;
	  
	  //* CCHLAC��ħ���
	  CCHLAC::extractColorCHLAC( feature1, voxel, sx, sy, sz, gx, gy, gz );
	  bool exist_flg = false;
	  for(int t=0;t<6;t++){
	    if(feature1[t]>0){
	      exist_flg = true;
	      break;
	    }
	  }
	  if(exist_flg){ // �ܥ�����Τʤ����ΰ�Ǥʤ���С�=CCHLAC��ħ����٥��ȥ�Ǥʤ���С�
	    CCHLAC::extractColorCHLAC_bin( feature2, voxel_bin, sx, sy, sz, gx, gy, gz );
	    for(int t=0;t<DIM_COLOR_1_3;t++)
	      feature[t] = feature1[t];
	    for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
	      feature[t+DIM_COLOR_1_3] = feature2[t];
	    pca.addData( feature ); // PCA��򤯽���
	  }
	}
      }
    }
    fclose( fp );
  }

  //* PCA��򤤤Ʒ�̤���¸
  pca.solve();
  pca.write( "scene/pca_result", ASCII_MODE_P );

  return 0;
}
