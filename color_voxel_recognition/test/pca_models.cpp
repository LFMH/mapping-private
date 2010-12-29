#include <iostream>
#include <octave/config.h>
#include <octave/Matrix.h>
#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/libPCA.hpp>
#include "../param/FILE_MODE"

/********************************************************************/
/* 検出対象物体を分割した全ての部分領域からColor-CHLAC特徴をとり主成分分析する   */
/* 得られる基底は環境内の各領域と検出対象物体との類似度計算に用いる（部分空間法）   */
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

  //* 分割領域の大きさの読み込み
  //* （注 sceneの分割領域の大きさと同じではなくともよい）
  const int box_size = Param::readBoxSize_model();

  //* RGB二値化の閾値の読み込み
  int color_threshold_r, color_threshold_g, color_threshold_b;
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );

  //* 圧縮したCCHLAC特徴ベクトルの次元数の読み込み
  const int dim = Param::readDim();

  //* CCHLAC特徴を圧縮する際に使用する主成分軸の読み込み
  PCA pca;
  pca.read("scene/pca_result", ASCII_MODE_P );
  Matrix axis = pca.Axis();
  axis.resize( DIM_COLOR_1_3+DIM_COLOR_BIN_1_3, dim );
  Matrix axis_t = axis.transpose();
  ColumnVector variance = pca.Variance();

  //****************************************************//
  //* 全てのボクセルファイルの全ての分割領域からのCCHLAC特徴抽出 *//
  //****************************************************//

  PCA pca_each( false ); // 物体の部分空間の基底を求めるための主成分分析
  const int num_files = atoi(argv[2]); // ボクセルファイルの数
  for( int n = 0; n < num_files; n++ ){
    printf("%d in %d...\n",n,num_files);

    //* ボクセル領域の確保とボクセルデータの読み込み
    sprintf(tmpname,"models/%s/Voxel/%03d.dat",argv[1],n);
    if( ASCII_MODE_V ) sprintf( file_mode, "r" );
    else sprintf( file_mode, "rb" );
    Voxel voxel( tmpname, file_mode ); // RGB二値化しない場合の特徴抽出用
    int xsize = voxel.Xsize();
    int ysize = voxel.Ysize();
    int zsize = voxel.Zsize();
    voxel.createVoxelData();
    voxel.cleanVoxelData();
    voxel.readVoxel( REVERSE_MODE );

    //* 分割領域の個数を調べる
    int x_num = xsize/box_size;
    if(xsize%box_size > 0)
      x_num++;
    int y_num = ysize/box_size;
    if(ysize%box_size > 0)
      y_num++;
    int z_num = zsize/box_size;
    if(zsize%box_size > 0)
      z_num++;

    //* ボクセル領域の確保とボクセルデータの読み込み
    Voxel voxel_bin( tmpname, file_mode ); // RGB二値化する場合の特徴抽出用
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
	  
	  //* CCHLAC特徴抽出
	  CCHLAC::extractColorCHLAC( feature[0], voxel, sx, sy, sz, gx, gy, gz );
	  bool exist_flag = false;
	  for(int j=0;j<2;j++){
	    if( feature[0](j)!=0 ){
	      exist_flag = true;
	      break;
	    }
	  }
	  if(exist_flag){ // ボクセルのない空領域でなければ（=CCHLAC特徴が零ベクトルでなければ）
	    CCHLAC::extractColorCHLAC_bin( feature_bin, voxel_bin, sx, sy, sz, gx, gy, gz );
	    feature[0].resize(DIM_COLOR_1_3+DIM_COLOR_BIN_1_3);
	    for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
	      feature[0](t+DIM_COLOR_1_3) = feature_bin(t);
	    
	    //* 90度ずつ変化させた24姿勢のそれぞれにおける特徴ベクトルを計算
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
	    
	    //* それぞれのCCHLAC特徴ベクトルを低次元に圧縮
	    for(int i=0;i<24;i++){
	      feature_final = axis_t*feature[i];
	      if( WHITENING )
		for(int t=0;t<dim;t++)
		  feature_final(t) /= sqrt( variance(t) );
	      pca_each.addData( feature_final ); // PCAを解く準備
	    }
	  }
	}
      }
    }
  }

  //* PCAを解いて結果を保存
  pca_each.solve();  
  sprintf(tmpname,"models/%s/pca_result",argv[1]);
  pca_each.write( tmpname, ASCII_MODE_P );

  return 0;
}
