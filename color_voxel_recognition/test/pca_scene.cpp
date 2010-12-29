#include <octave/config.h>
#include <octave/Matrix.h>

#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include "../param/FILE_MODE"

/********************************************************************/
/* 環境全体を分割した全ての部分領域からColor-CHLAC特徴をとり主成分分析する      */
/* 得られる基底はColor-CHLAC特徴ベクトルの圧縮に用いる                      */
/* （環境からの特徴抽出を頻繁に行う場合は省略可能。最初に求めた軸を再利用してよい） */
/* （PCA-SIFTの軸のようなもの）                                         */
/********************************************************************/

using namespace std;

int main(int argc, char** argv){

  //* 分割領域の大きさの読み込み
  const int box_size = Param::readBoxSize_scene();

  //* 一度に作成するボクセル数の上限の読み込み
  const int max_voxel_num = Param::readMaxVoxelNum();

  //* ボクセルデータの大きさのみ 読み込み
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

  //* RGB二値化の閾値の読み込み
  int color_threshold_r, color_threshold_g, color_threshold_b;
  Param::readColorThreshold( color_threshold_r, color_threshold_g, color_threshold_b );

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

  //* おそらくメモリが足りなくなるので、複数回に分けてボクセル化→CCHLAC抽出を行っていきます
  int z_num_middle = max_voxel_num / ( x_num * box_size * y_num * box_size * box_size );
  int z_num_num = z_num / z_num_middle; // 最後のループの一回手前
  int zsize_middle = z_num_middle*box_size;
  if( z_num_middle >= z_num ){ // 環境が十分に小さかったら
    z_num_middle = z_num;
    z_num_num = 0;
    zsize_middle = zsize;
  }

  //* ボクセル領域の確保
  Voxel voxel( ASCII_MODE_V );     // RGB二値化しない場合の特徴抽出用
  Voxel voxel_bin( ASCII_MODE_V ); // RGB二値化する場合の特徴抽出用
  voxel.setXYZsize( xsize, ysize, zsize_middle+1 );
  voxel_bin.setXYZsize( xsize, ysize, zsize_middle+1 );
  voxel.createVoxelData();
  voxel_bin.createVoxelData();

  //*********************************//
  //* 全ての分割領域からのCCHLAC特徴抽出 *//
  //*********************************//

  PCA pca( false ); // 特徴ベクトルから平均ベクトルをひかない。必ずfalseにすること
  ColumnVector feature(DIM_COLOR_1_3+DIM_COLOR_BIN_1_3);
  ColumnVector feature1; // RGB二値化しないCCHLAC特徴
  ColumnVector feature2; // RGB二値化するCCHLAC特徴

  for(int tmp_num=0; tmp_num<z_num_num+1; tmp_num++){
    printf("%d in %d...\n",tmp_num,z_num_num);

    //* ボクセルデータの読み込み
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

	  //* CCHLAC特徴抽出
	  CCHLAC::extractColorCHLAC( feature1, voxel, sx, sy, sz, gx, gy, gz );
	  bool exist_flg = false;
	  for(int t=0;t<6;t++){
	    if(feature1(t)>0){
	      exist_flg = true;
	      break;
	    }
	  }
	  if(exist_flg){ // ボクセルのない空領域でなければ（=CCHLAC特徴が零ベクトルでなければ）
	    CCHLAC::extractColorCHLAC_bin( feature2, voxel_bin, sx, sy, sz, gx, gy, gz );
	    for(int t=0;t<DIM_COLOR_1_3;t++)
	      feature(t) = feature1(t);
	    for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
	      feature(t+DIM_COLOR_1_3) = feature2(t);
	    pca.addData( feature ); // PCAを解く準備
	  }
	}
      }
    }
  }

  //* PCAを解いて結果を保存
  pca.solve();
  pca.write( "scene/pca_result", ASCII_MODE_P );

  return 0;
}
