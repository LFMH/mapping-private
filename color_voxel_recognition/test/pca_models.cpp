#include <iostream>
#include <octave/config.h>
#include <octave/Matrix.h>
#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/libPCA.hpp>
#include "../param/FILE_MODE"

/********************************************************************/
/* �����о�ʪ�Τ�ʬ�䤷�����Ƥ���ʬ�ΰ褫��Color-CHLAC��ħ��Ȥ����ʬʬ�Ϥ���   */
/* ���������ϴĶ���γ��ΰ�ȸ����о�ʪ�ΤȤ�����ٷ׻����Ѥ������ʬ����ˡ��   */
/********************************************************************/

using namespace std;

int main( int argc, char* argv[])
{
  if( argc != 3 ){
    cerr << "usage: " << argv[0] << " [label] <file_num>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[100];
  char file_mode[ 3 ];

  //* ʬ���ΰ���礭�����ɤ߹���
  //* ���� scene��ʬ���ΰ���礭����Ʊ���ǤϤʤ��Ȥ�褤��
  const int box_size = Param::readBoxSize_model();

  //* RGB���Ͳ������ͤ��ɤ߹���
  int color_threshold_r, color_threshold_g, color_threshold_b;
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );

  //* ���̤���CCHLAC��ħ�٥��ȥ�μ��������ɤ߹���
  const int dim = Param::readDim();

  //* CCHLAC��ħ�򰵽̤���ݤ˻��Ѥ������ʬ�����ɤ߹���
  PCA pca;
  pca.read("scene/pca_result", ASCII_MODE_P );
  Matrix axis = pca.Axis();
  axis.resize( DIM_COLOR_1_3+DIM_COLOR_BIN_1_3, dim );
  Matrix axis_t = axis.transpose();
  ColumnVector variance = pca.Variance();

  //****************************************************//
  //* ���ƤΥܥ�����ե���������Ƥ�ʬ���ΰ褫���CCHLAC��ħ��� *//
  //****************************************************//

  PCA pca_each( false ); // ʪ�Τ���ʬ���֤δ������뤿��μ���ʬʬ��
  const int num_files = atoi(argv[2]); // �ܥ�����ե�����ο�
  for( int n = 0; n < num_files; n++ ){
    printf("%d in %d...\n",n,num_files);

    //* �ܥ������ΰ�γ��ݤȥܥ�����ǡ������ɤ߹���
    sprintf(tmpname,"models/%s/Voxel/%03d.dat",argv[1],n);
    if( ASCII_MODE_V ) sprintf( file_mode, "r" );
    else sprintf( file_mode, "rb" );
    Voxel voxel( tmpname, file_mode ); // RGB���Ͳ����ʤ�������ħ�����
    int xsize = voxel.Xsize();
    int ysize = voxel.Ysize();
    int zsize = voxel.Zsize();
    voxel.createVoxelData();
    voxel.cleanVoxelData();
    voxel.readVoxel( REVERSE_MODE );

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

    //* �ܥ������ΰ�γ��ݤȥܥ�����ǡ������ɤ߹���
    Voxel voxel_bin( tmpname, file_mode ); // RGB���Ͳ����������ħ�����
    voxel_bin.createVoxelData();
    voxel_bin.cleanVoxelData();
    voxel_bin.readVoxel( color_threshold_r, color_threshold_g, color_threshold_b );
        
    ColumnVector feature[24];
    ColumnVector feature_bin;
    ColumnVector feature_tmp;
    ColumnVector feature_tmp2;
    ColumnVector feature_final;

    for(int k=0;k<z_num;k++){
      for(int j=0;j<y_num;j++){
	for(int i=0;i<x_num;i++){
	  int sx = i*box_size+1;
	  int sy = j*box_size+1;
	  int sz = k*box_size+1;
	  int gx = sx+box_size;
	  int gy = sy+box_size;
	  int gz = sz+box_size;
	  if(gx>xsize-1) gx = xsize-1;
	  if(gy>ysize-1) gy = ysize-1;
	  if(gz>zsize-1) gz = zsize-1;
	  
	  //* CCHLAC��ħ���
	  CCHLAC::extractColorCHLAC( feature[0], voxel, sx, sy, sz, gx, gy, gz );
	  bool exist_flag = false;
	  for(int j=0;j<2;j++){
	    if( feature[0](j)!=0 ){
	      exist_flag = true;
	      break;
	    }
	  }
	  if(exist_flag){ // �ܥ�����Τʤ����ΰ�Ǥʤ���С�=CCHLAC��ħ����٥��ȥ�Ǥʤ���С�
	    CCHLAC::extractColorCHLAC_bin( feature_bin, voxel_bin, sx, sy, sz, gx, gy, gz );
	    feature[0].resize(DIM_COLOR_1_3+DIM_COLOR_BIN_1_3);
	    for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
	      feature[0](t+DIM_COLOR_1_3) = feature_bin(t);
	    
	    //* 90�٤����Ѳ�������24�����Τ��줾��ˤ�������ħ�٥��ȥ��׻�
	    for(int i=0;i<3;i++)
	      CCHLAC::rotateFeature90( feature[i+1],feature[i],R_MODE_2);
	    CCHLAC::rotateFeature90( feature[4],feature[0],R_MODE_3);
	    for(int i=0;i<3;i++)
	      CCHLAC::rotateFeature90( feature[i+5],feature[i+4],R_MODE_2);
	    CCHLAC::rotateFeature90( feature[8],feature[4],R_MODE_3);
	    for(int i=0;i<3;i++)
	      CCHLAC::rotateFeature90( feature[i+9],feature[i+8],R_MODE_2);
	    CCHLAC::rotateFeature90( feature[12],feature[8],R_MODE_3);
	    for(int i=0;i<3;i++)
	      CCHLAC::rotateFeature90( feature[i+13],feature[i+12],R_MODE_2);
	    
	    CCHLAC::rotateFeature90( feature[16],feature[0],R_MODE_1);
	    for(int i=0;i<3;i++)
	      CCHLAC::rotateFeature90( feature[i+17],feature[i+16],R_MODE_2);
	    CCHLAC::rotateFeature90( feature[20],feature[0],R_MODE_4);
	    for(int i=0;i<3;i++)
	      CCHLAC::rotateFeature90( feature[i+21],feature[i+20],R_MODE_2);
	    
	    //* ���줾���CCHLAC��ħ�٥��ȥ���㼡���˰���
	    for(int i=0;i<24;i++){
	      feature_final = axis_t*feature[i];
	      if( WHITENING )
		for(int t=0;t<dim;t++)
		  feature_final(t) /= sqrt( variance(t) );
	      pca_each.addData( feature_final ); // PCA��򤯽���
	    }
	  }
	}
      }
    }
  }

  //* PCA��򤤤Ʒ�̤���¸
  pca_each.solve();  
  sprintf(tmpname,"models/%s/pca_result",argv[1]);
  pca_each.write( tmpname, ASCII_MODE_P );

  return 0;
}
