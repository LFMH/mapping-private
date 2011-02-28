#include <color_voxel_recognition/libPCA.hpp>
#include <color_voxel_recognition/CCHLAC.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include <color_voxel_recognition/Param.hpp>
#include "./FILE_MODE"

/***********************************************************************/
/* 1. 環境全体を分割した全ての部分領域からColor-CHLAC特徴をとり主成分分析する      */
/*    得られる基底はColor-CHLAC特徴ベクトルの圧縮に用いる                      */
/*    （環境からの特徴抽出を頻繁に行う場合は省略可能。最初に求めた軸を再利用してよい） */
/*    （PCA-SIFTの軸のようなもの）                                         */
/* 2. 環境の分割領域毎にColor-CHLAC特徴をとり、次元圧縮                        */
/*    Viora-Jones積分画像と同様に、領域の特徴量を端から積分する                 */
/*    積分した特徴量とボクセルの個数をファイルに出力                            */
/***********************************************************************/

using namespace std;

int main(int argc, char** argv){

  PCA pca( false ); // 特徴ベクトルから平均ベクトルをひかない。必ずfalseにすること

  //* 圧縮したCCHLAC特徴ベクトルの次元数の読み込み
  const int dim = Param::readDim();

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
  int xy_num = x_num * y_num;

  //* 領域内のボクセルの個数と領域の特徴量のメモリ確保
  int *exist_num = new int[ xy_num * z_num ];  // 領域内のボクセルの個数
  Eigen3::VectorXf *nFeatures = new Eigen3::VectorXf[ xy_num * z_num ]; // 領域の特徴量

  //* おそらくメモリが足りなくなるので、複数回に分けてボクセル化→CCHLAC抽出を行っていきます
  int z_num_middle = max_voxel_num / ( x_num * box_size * y_num * box_size * box_size );
  int z_num_num = z_num / z_num_middle; // 最後のループの一回手前
  int zsize_middle = z_num_middle*box_size;
  if( z_num_middle >= z_num ){ // 環境が十分に小さかったら
    z_num_middle = z_num;
    z_num_num = 0;
    zsize_middle = zsize;
  }

  {     // ボクセルの生存する範囲
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

    std::vector<float> feature( DIM_COLOR_1_3 + DIM_COLOR_BIN_1_3 );
    std::vector<float> feature1; // RGB二値化しないCCHLAC特徴
    std::vector<float> feature2; // RGB二値化するCCHLAC特徴

    int idx = 0;
    for(int tmp_num=0; tmp_num<z_num_num+1; tmp_num++){
      printf("%d in %d...\n",tmp_num,z_num_num);

      //* ボクセルデータの読み込み
      voxel.cleanVoxelData();
      voxel_bin.cleanVoxelData();
      voxel.readVoxelZoffset( "scene/voxel_scene.dat", zsize_middle*tmp_num, SIMPLE_REVERSE, false );
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
	    CCHLAC::extractColorCHLAC_bin( feature2, voxel_bin, sx, sy, sz, gx, gy, gz );

	    exist_num[ idx ] = ( feature2[0] + feature2[1] ) * 3 + 0.001;

	    if( exist_num[ idx ] > 0 ){ // ボクセルのない空領域でなければ
	      CCHLAC::extractColorCHLAC( feature1, voxel, sx, sy, sz, gx, gy, gz );
	      for(int t=0;t<DIM_COLOR_1_3;t++)
		feature[t] = feature1[t];
	      for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
		feature[t+DIM_COLOR_1_3] = feature2[t];
	      for( int t=0; t<DIM_COLOR_1_3 + DIM_COLOR_BIN_1_3; t++ )
		nFeatures[ idx ][ t ] = feature[ t ];
	      pca.addData( feature ); // PCAを解く準備
	    }
	    else
	      for( int t=0; t<DIM_COLOR_1_3 + DIM_COLOR_BIN_1_3; t++ )
		nFeatures[ idx ][ t ] = 0;
	    idx++;
	  }
	}
      }
    }
  }

  //*********************//
  //* PCAを解いて結果を保存 *//
  //*********************//
  pca.solve();
  pca.write( "scene/pca_result", ASCII_MODE_P );
  printf("PCA solved.\n");    

  //****************************************************//
  //* 特徴量を圧縮しつつ積分する（viola-jones積分画像の特徴版） *//
  //****************************************************//

  //* CCHLAC特徴を圧縮する際に使用する主成分軸の読み込み
  Eigen3::MatrixXf tmpaxis = pca.Axis();
  Eigen3::MatrixXf axis = tmpaxis.block( 0,0,tmpaxis.rows(),dim );
  Eigen3::MatrixXf axis_t = axis.transpose();
  Eigen3::VectorXf variance = pca.Variance();

  //* 書き込む先のファイルを開く
  //* 特徴ファイルとボクセルの有無ファイル
  FILE *fp1, *fp2;
  if( ASCII_MODE_F ){
    fp1 = fopen("scene/cchlac_small_add.dat","w");
    fprintf(fp1,"%d %d %d\n",x_num,y_num,z_num);
    fp2 = fopen("scene/existNum_add.dat","w");
  }
  else{
    fp1 = fopen("scene/cchlac_small_add.dat","wb");
    fwrite(&x_num,sizeof(x_num),1,fp1);
    fwrite(&y_num,sizeof(y_num),1,fp1);
    fwrite(&z_num,sizeof(z_num),1,fp1);
    fp2 = fopen("scene/existNum_add.dat","wb");
  }

  for(int z=0;z<z_num;z++){
    printf("  %d in %d...\n",z,z_num);
    for(int y=0;y<y_num;y++){
      for(int x=0;x<x_num;x++){
	nFeatures[ x + y*x_num + z*xy_num ] = axis_t * nFeatures[ x + y*x_num + z*xy_num ];
	if( WHITENING )
	  for(int t=0;t<dim;t++)
	    nFeatures[ x + y*x_num + z*xy_num ](t) /= sqrt( variance(t) );
	if(z==0){
	  if(y==0){
	    if(x!=0){ // (1,0,0)
	      exist_num[ x + y*x_num + z*xy_num ] += exist_num[ ( x - 1 ) + y*x_num + z*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ] += nFeatures[ ( x - 1 ) + y*x_num + z*xy_num ];
	    }
	  }
	  else{
	    if(x==0){ // (0,1,0)
	      exist_num[ x + y*x_num + z*xy_num ] += exist_num[ x + ( y - 1 )*x_num + z*xy_num ];	    
	      nFeatures[ x + y*x_num + z*xy_num ] += nFeatures[ x + ( y - 1 )*x_num + z*xy_num ];	    
	    }
	    else{ // (1,1,0)
	      exist_num[ x + y*x_num + z*xy_num ] 
		+= exist_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_num[ x + ( y - 1 )*x_num + z*xy_num ]
		-  exist_num[ ( x - 1 ) + ( y - 1 )*x_num + z*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ]
		+= nFeatures[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  nFeatures[ x + ( y - 1 )*x_num + z*xy_num ]
		-  nFeatures[ ( x - 1 ) + ( y - 1 )*x_num + z*xy_num ];
	    }
	  }
	}
	else{
	  if(y==0){
	    if(x==0){ // (0,0,1)	
	      exist_num[ x + y*x_num + z*xy_num ] += exist_num[ x + y*x_num + ( z - 1 )*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ] += nFeatures[ x + y*x_num + ( z - 1 )*xy_num ];
	    }
	    else {// (1,0,1)
	      exist_num[ x + y*x_num + z*xy_num ] 
		+= exist_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_num[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ]
		+= nFeatures[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  nFeatures[ x + y *x_num + ( z - 1 )*xy_num ]
		-  nFeatures[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ];
	    }
	  }
	  else{
	    if(x==0){ // (0,1,1)
	      exist_num[ x + y*x_num + z*xy_num ] 
		+= exist_num[ x + ( y - 1 )*x_num + z *xy_num ]
		+  exist_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_num[ x + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ]
		+= nFeatures[ x + ( y - 1 )*x_num + z *xy_num ]
		+  nFeatures[ x + y *x_num + ( z - 1 )*xy_num ]
		-  nFeatures[ x + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	    }
	    else{ // (1,1,1)
	      exist_num[ x + y*x_num + z*xy_num ] 
		+= exist_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_num[ x + ( y - 1 )*x_num + z*xy_num ]
		+  exist_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_num[ ( x - 1 ) + ( y - 1 )*x_num + z *xy_num ]
		-  exist_num[ x + ( y - 1 )*x_num + ( z - 1 )*xy_num ]
		-  exist_num[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ]
		+  exist_num[ ( x - 1 ) + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ] 
		+= nFeatures[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  nFeatures[ x + ( y - 1 )*x_num + z*xy_num ]
		+  nFeatures[ x + y *x_num + ( z - 1 )*xy_num ]
		-  nFeatures[ ( x - 1 ) + ( y - 1 )*x_num + z *xy_num ]
		-  nFeatures[ x + ( y - 1 )*x_num + ( z - 1 )*xy_num ]
		-  nFeatures[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ]
		+  nFeatures[ ( x - 1 ) + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	    }
	  }
	}
	
	//* ファイルに書き込み
	if( ASCII_MODE_F ){
	  for(int t=0;t<dim;t++)
	    fprintf(fp1,"%d:%f ",t+1,nFeatures[ x + y*x_num + z*xy_num ](t));
	  fprintf(fp1,"\n");
	  fprintf(fp2,"%d\n",exist_num[ x + y*x_num + z*xy_num ]);
	}
	else{
	  for(int t=0;t<dim;t++)
	    fwrite(&(nFeatures[ x + y*x_num + z*xy_num ](t)),sizeof(nFeatures[ x + y*x_num + z*xy_num ](t)),1,fp1);
	  fwrite( exist_num + x + y*x_num + z*xy_num, sizeof(exist_num[ x + y*x_num + z*xy_num ]),1,fp2);
	}
      }
    }
  }
  
  fclose(fp1);
  fclose(fp2);	

  return 0;
}
