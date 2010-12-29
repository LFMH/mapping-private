#include <iostream>
#include <stdio.h>
//#include <stdlib.h>
//#include <string>
//#include <unistd.h>
#include <sys/time.h>
//#include <math.h>
//#include <fstream>
#include <octave/config.h>
#include <octave/Matrix.h>
#include "color_voxel_recognition/libPCA.hpp"
#include "color_voxel_recognition/CCHLAC.hpp"
#include "color_voxel_recognition/Search.hpp"

//* 時間計測用
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

//***************************//
//* コンストラクタとデストラクタ *//
//***************************//

SearchObj::SearchObj() :
  max_x(NULL),
  max_y(NULL),
  max_z(NULL),
  max_mode(NULL),
  max_dot (NULL),
  exist_voxel_num(NULL),
  nFeatures(NULL) {
}

SearchObj::~SearchObj(){
  if( max_x != NULL ) delete[] max_x;
  if( max_y != NULL ) delete[] max_y;
  if( max_z != NULL ) delete[] max_z;
  if( max_mode != NULL ) delete[] max_mode;
  if( max_dot  != NULL ) delete[] max_dot;
  if( exist_voxel_num != NULL ) delete[] exist_voxel_num;
  if( nFeatures != NULL ) delete[] nFeatures;
}

//**************//
//* 変数のセット *//
//**************//

//***************************
//* 検出対象物体のサイズ情報をセット
void SearchObj::setRange( int _range1, int _range2, int _range3 ){
  range1 = _range1;
  range2 = _range2;
  range3 = _range3;
}

//************************************
//* ランク何位までの領域を出力するか、をセット
void SearchObj::setRank( int _rank_num ){
  rank_num = _rank_num;
  if( max_x != NULL ) delete[] max_x;
  if( max_y != NULL ) delete[] max_y;
  if( max_z != NULL ) delete[] max_z;
  if( max_mode != NULL ) delete[] max_mode;
  if( max_dot  != NULL ) delete[] max_dot;
  max_x = new int[ rank_num ];
  max_y = new int[ rank_num ];
  max_z = new int[ rank_num ];
  max_mode = new SearchMode[ rank_num ];
  max_dot = new double[ rank_num ];
  for( int i=0; i<rank_num; i++ )  max_dot[ i ] = 0;
}

//***************************************************************
//* 検出領域の中に含まれるボクセル数がいくつ未満だったら打ちきるかの閾値をセット
void SearchObj::setThreshold( int _exist_voxel_num_threshold ){
  exist_voxel_num_threshold = _exist_voxel_num_threshold;
}

//**********************************
//* 検出対象物体の部分空間の基底軸の読み込み
void SearchObj::readAxis( const char *filename, int dim, int dim_model, bool ascii, bool multiple_similarity ){
  PCA pca_each;
  pca_each.read( filename, ascii );
  Matrix tmpaxis = pca_each.Axis();
  tmpaxis.resize( dim, dim_model );
  Matrix tmpaxis_t = tmpaxis.transpose();
  axis_q = tmpaxis.transpose();
  if( multiple_similarity ){
    ColumnVector variance = pca_each.Variance();
    for( int i=0; i<dim_model; i++ )
      for( int j=0; j<dim; j++ )
	axis_q( i, j ) = sqrt( variance( i ) ) * axis_q( i, j );
  }
}

//********************************************
//* sceneの（分割領域毎の）特徴量とボクセル数の読み込み
void SearchObj::readData( const char *filenameF, const char *filenameN, int dim, bool ascii ){
  double tmpval_d;

  FILE *fp, *fp2;
  if( ascii ){
    fp = fopen( filenameF, "r" );
    fscanf(fp,"%d %d %d\n",&x_num,&y_num,&z_num);
    fp2 = fopen( filenameN,"r" );
  }
  else{
    fp = fopen( filenameF, "rb" );
    fread(&x_num,sizeof(int),1,fp);
    fread(&y_num,sizeof(int),1,fp);
    fread(&z_num,sizeof(int),1,fp);
    fp2 = fopen( filenameN, "rb" );
  }

  xy_num = x_num*y_num;
  const int xyz_num = xy_num * z_num;
  exist_voxel_num = new int[ xyz_num ];
  nFeatures = new ColumnVector[ xyz_num ];
  for(int n=0;n<xyz_num;n++){
    nFeatures[ n ].resize(dim);
    if( ascii ){
      int tmpval;
      for(int j=0;j<dim;j++){
	fscanf(fp,"%d:%lf ",&tmpval,&tmpval_d);
	nFeatures[ n ]( j ) = tmpval_d;
      }
      fscanf(fp,"\n");
      fscanf(fp2,"%d\n",exist_voxel_num+n);
    }
    else{
      for(int j=0;j<dim;j++){
	fread(&tmpval_d,sizeof(tmpval_d),1,fp);
	nFeatures[ n ]( j ) = tmpval_d;
      }
      fread(exist_voxel_num+n,sizeof(exist_voxel_num[n]),1,fp2);
    }
  }
  fclose(fp);
}

//*********************************//
//* 検出ボックスの辺長の取得に関する関数 *//
//*********************************//

//*************************************************
//* 引数の検出モードにおける検出ボックスのx,y,z辺の長さを取得
inline void SearchObj::getRange( int &xrange, int &yrange, int &zrange, SearchMode mode ){
  switch( mode ){
  case S_MODE_1:
    xrange = range1;
    yrange = range2;
    zrange = range3;
    break;
  case S_MODE_2:
    xrange = range1;
    yrange = range3;
    zrange = range2;
    break;
  case S_MODE_3:
    xrange = range2;
    yrange = range1;
    zrange = range3;
    break;
  case S_MODE_4:
    xrange = range2;
    yrange = range3;
    zrange = range1;
    break;
  case S_MODE_5:
    xrange = range3;
    yrange = range1;
    zrange = range2;
    break;
  case S_MODE_6:
    xrange = range3;
    yrange = range2;
    zrange = range1;
    break;
  }
}

//*********************************************
//* 引数の検出モードにおける検出ボックスのx辺の長さを返す
inline int SearchObj::xRange( SearchMode mode ){
  switch( mode ){
  case S_MODE_1:
  case S_MODE_2:
    return range1;
    break;
  case S_MODE_3:
  case S_MODE_4:
    return range2;
    break;
  case S_MODE_5:
  case S_MODE_6:
    return range3;
    break;
  default:
    break;
  }
  return 0;
}

//*********************************************
//* 引数の検出モードにおける検出ボックスのy辺の長さを返す
inline int SearchObj::yRange( SearchMode mode ){
  switch( mode ){
  case S_MODE_3:
  case S_MODE_5:
    return range1;
    break;
  case S_MODE_1:
  case S_MODE_6:
    return range2;
    break;
  case S_MODE_2:
  case S_MODE_4:
    return range3;
    break;
  default:
    break;
  }
  return 0;
}

//*********************************************
//* 引数の検出モードにおける検出ボックスのz辺の長さを返す
inline int SearchObj::zRange( SearchMode mode ){
  switch( mode ){
  case S_MODE_4:
  case S_MODE_6:
    return range1;
    break;
  case S_MODE_2:
  case S_MODE_5:
    return range2;
    break;
  case S_MODE_1:
  case S_MODE_3:
    return range3;
    break;
  default:
    break;
  }
  return 0;
}

//*************************//
//* 検出領域の保存に関する関数 *//
//*************************//

//***********************************************************************
//* 今までに発見した領域の中で、いま発見した領域とかぶる領域があれば、そのランク番号を返す
//* かぶる領域がなければ最低ランクの番号を返す
inline int SearchObj::checkOverlap( int x, int y, int z, SearchMode mode ){
  int num;
  int xrange = 0, yrange = 0, zrange = 0;
  int val1, val2, val3;
  getRange( xrange, yrange, zrange, mode );

  for( num = 0; num < rank_num-1; num++ ){
    val1 = max_x[ num ] - x;
    if( val1 < 0 )
      val1 = - val1 - xRange( max_mode[ num ] );
    else
      val1 -= xrange;

    val2 = max_y[ num ] - y;
    if( val2 < 0 )
      val2 = - val2 - yRange( max_mode[ num ] );
    else
      val2 -= yrange;

    val3 = max_z[ num ] - z;
    if( val3 < 0 )
      val3 = - val3 - zRange( max_mode[ num ] );
    else
      val3 -= zrange;

    if( ( val1 <= 0 ) && ( val2 <= 0 ) && ( val3 <= 0 ) )
      return num;
  }
  return num;
}

//****************************
//* 既に発見した領域のランクをずらす
//* 例 src_num=2 dest_num=3: 3位の領域を4位に下げる
inline void SearchObj::max_cpy( int src_num, int dest_num ){
  max_dot[ dest_num ] = max_dot[ src_num ];
  max_x[ dest_num ] = max_x[ src_num ];
  max_y[ dest_num ] = max_y[ src_num ];
  max_z[ dest_num ] = max_z[ src_num ];
  max_mode[ dest_num ] = max_mode[ src_num ];
}

//*************************************
//* 今発見した領域を既に発見した領域と置き換える
inline void SearchObj::max_assign( int dest_num, double dot, int x, int y, int z, SearchMode mode ){
  max_dot[ dest_num ] = dot;
  max_x[ dest_num ] = x;
  max_y[ dest_num ] = y;
  max_z[ dest_num ] = z;
  max_mode[ dest_num ] = mode;
}

//****************//
//* 物体検出の関数 *//
//****************//

//*************************
//* 全ての検出モードでの物体検出
void SearchObj::search(){
  double t1,t2;//時間計測用の変数
  t1 = my_clock();
  if( range1 == range2 ){
    if( range2 == range3 ){ // range1 = range2 = range3
      search_part( S_MODE_1 );
    }
    else{ // range1 = range2
      search_part( S_MODE_1 );
      search_part( S_MODE_2 );
      search_part( S_MODE_5 );
    }
  }
  else if( range2 == range3 ){ // range2 = range3
    search_part( S_MODE_1 );
    search_part( S_MODE_5 );
    search_part( S_MODE_6 );
  }
  else if( range1==range3 ){ // range1 = range3
    search_part( S_MODE_1 );
    search_part( S_MODE_5 );
    search_part( S_MODE_3 );
  }
  else{ // range1 != range2 != range3
    search_part( S_MODE_1 );
    search_part( S_MODE_2 );
    search_part( S_MODE_3 );
    search_part( S_MODE_4 );
    search_part( S_MODE_5 );
    search_part( S_MODE_6 );
  }
  t2 = my_clock();
  search_time = t2 - t1;
}

//*************************
//* 引数の検出モードでの物体検出
void SearchObj::search_part( SearchMode mode ){
  int xrange = 0, yrange = 0, zrange = 0;
  getRange( xrange, yrange, zrange, mode );

  const int x_end = x_num - xrange + 1;
  const int y_end = y_num - yrange + 1;
  const int z_end = z_num - zrange + 1;
  
  ColumnVector feature_tmp;
  ColumnVector tmpVector;
  double dot,sum;
  int overlap_num;
  int exist_num;
  if((x_end>0)&&(y_end>0)&&(z_end>0)){    
    for(int z=0;z<z_end;z++){	      
      for(int y=0;y<y_end;y++){
	for(int x=0;x<x_end;x++){

	  //* 領域内にボクセルが存在するか否かをチェック
	  exist_num = clipValue( exist_voxel_num, x, y, z, xrange, yrange, zrange );
	  if(exist_num > exist_voxel_num_threshold){ // 領域内にボクセルが一定数以上あれば

	    feature_tmp = clipValue( nFeatures, x, y, z, xrange, yrange, zrange );
	    
	    //* 類似度計算
	    sum = feature_tmp.transpose()*feature_tmp;
	    tmpVector = axis_q * feature_tmp;
#if 0
	    sum = sqrt(sum);
	    dot = tmpVector.transpose()*tmpVector;
	    dot = sqrt( dot );
	    dot /= sum ;
#else // 少しでも高速化したいときはこちらでもよい
	    dot = tmpVector.transpose()*tmpVector;
	    dot /= sum ;
#endif
	    //* 類似度が高ければ保存
	    for( int i=0; i<rank_num; i++ ){
	      if(dot>max_dot[ i ]){
		overlap_num = checkOverlap( x, y, z, mode );
		for( int j=0; j<overlap_num-i; j++ )
		  max_cpy( overlap_num-1-j, overlap_num-j );		  

		if( i<=overlap_num )
		  max_assign( i, dot, x, y, z, mode );
		break;
	      }
	    }
	  }
	}
      }
    }
  }
}

//**********************************************
//* 順に値が積分された配列からその位置における値を取り出す
template <typename T>
T SearchObj::clipValue( T* ptr, const int x, const int y, const int z, const int xrange, const int yrange, const int zrange ) {
  T result;
  if(z==0){
    if(y==0){
      if(x==0) // (0,0,0)
        result = ptr[ xrange-1 + (yrange-1)*x_num + (zrange-1)*xy_num ];
      else // (*,0,0)
        result = ptr[ x+xrange-1 + (yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ x-1 + (yrange-1)*x_num + (zrange-1)*xy_num ];
    }
    else{
      if(x==0) // (0,*,0)
        result = ptr[ xrange-1 + (y+yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ xrange-1 + (y-1)*x_num + (zrange-1)*xy_num ];
      else // (*,*,0)
        result = ptr[ x+xrange-1 + (y+yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ x-1 + (y+yrange-1)*x_num + (zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (y-1)*x_num + (zrange-1)*xy_num ]
          + ptr[ x-1 + (y-1)*x_num + (zrange-1)*xy_num ];
    }
  }
  else{
    if(y==0){
      if(x==0) // (0,0,*)	
        result = ptr[ xrange-1 + (yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ xrange-1 + (yrange-1)*x_num + (z-1)*xy_num ];
      else // (*,0,*)
        result = ptr[ x+xrange-1 + (yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x-1 + (yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (yrange-1)*x_num + (z-1)*xy_num ]
          + ptr[ x-1 + (yrange-1)*x_num + (z-1)*xy_num ];
    }
    else{
      if(x==0) // (0,*,*)
        result = ptr[ xrange-1 + (y+yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ xrange-1 + (y+yrange-1)*x_num + (z-1)*xy_num ]
          - ptr[ xrange-1 + (y-1)*x_num + (z+zrange-1)*xy_num ]
          + ptr[ xrange-1 + (y-1)*x_num + (z-1)*xy_num ];
      else // (*,*,*)
        result = ptr[ x+xrange-1 + (y+yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x-1 + (y+yrange-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (y-1)*x_num + (z+zrange-1)*xy_num ]
          - ptr[ x+xrange-1 + (y+yrange-1)*x_num + (z-1)*xy_num ]
          + ptr[ x-1 + (y-1)*x_num + (z+zrange-1)*xy_num ]
          + ptr[ x-1 + (y+yrange-1)*x_num + (z-1)*xy_num ]
          + ptr[ x+xrange-1 + (y-1)*x_num + (z-1)*xy_num ]
          - ptr[ x-1 + (y-1)*x_num + (z-1)*xy_num ];
    }
  }
  return result;
}

//*******************
//* 結果をファイルに出力
void SearchObj::writeResult( const char *filename, int box_size ){
  FILE *fp = fopen(filename,"w");
  int xrange = 0, yrange = 0, zrange = 0;
  for( int r=0; r<rank_num; r++ ){
    if( max_dot[ r ] == 0 ) break;
    int x_tmp_min = max_x[ r ] * box_size;
    int y_tmp_min = max_y[ r ] * box_size;
    int z_tmp_min = max_z[ r ] * box_size;
    getRange( xrange, yrange, zrange, max_mode[ r ] );
    int x_tmp_max = (max_x[ r ] + xrange) * box_size;
    int y_tmp_max = (max_y[ r ] + yrange) * box_size;
    int z_tmp_max = (max_z[ r ] + zrange) * box_size;
    //printf("max: %4d~%4d %4d~%4d %4d~%4d | max_dot: %f\n",x_tmp_min,x_tmp_max,y_tmp_min,y_tmp_max,z_tmp_min,z_tmp_max,max_dot[ r ]);
    fprintf(fp,"%d %d %d %d %d %d %f\n",x_tmp_min,x_tmp_max-x_tmp_min,y_tmp_min,y_tmp_max-y_tmp_min,z_tmp_min,z_tmp_max-z_tmp_min,max_dot[ r ]);
  }    
  fprintf(fp,"time: %f\n",search_time);//経過時間
  fclose(fp);
}

void SearchObj::cleanMax(){
  for( int i=0; i<rank_num; i++ ){
    max_x[ i ] = 0;
    max_y[ i ] = 0;
    max_z[ i ] = 0;
    max_dot[ i ] = 0;
  }
}

////////////////////////////
///    オンライン処理向け    ///
////////////////////////////

void SearchObj::setSceneAxis( Matrix _axis ){
  axis_p = _axis;
}

void SearchObj::setSceneAxis( Matrix _axis, ColumnVector var, int dim ){
  for( int i=0; i<dim; i++ ){
    for( int j=0; j<DIM_COLOR_1_3+DIM_COLOR_BIN_1_3; j++ ){
      const float tmpval = 1/ sqrt( var( i ) );
      _axis( i, j ) = tmpval * _axis( i, j ) ;
    }
  }
  axis_p = _axis;
}

void SearchObj::cleanData(){
  x_num = 0;
  y_num = 0;
  z_num = 0;
  xy_num = 0;
  for( int i=0; i<rank_num; i++ ){
    max_x[ i ] = 0;
    max_y[ i ] = 0;
    max_z[ i ] = 0;
    //max_mode[ i ] = 0;
    max_dot[ i ] = 0;
  }
  if( exist_voxel_num != NULL ) delete[] exist_voxel_num;
  if( nFeatures != NULL ) delete[] nFeatures;
  exist_voxel_num = NULL;
  nFeatures = NULL;
}

void SearchObj::setData( Voxel& voxel, Voxel& voxel_bin, int dim, int box_size ){
  //* 分割領域の個数を調べる
  const int xsize = voxel.Xsize();
  const int ysize = voxel.Ysize();
  const int zsize = voxel.Zsize();
  x_num = xsize/box_size;
  if(xsize%box_size > 0)   x_num++;
  y_num = ysize/box_size;
  if(ysize%box_size > 0)   y_num++;
  z_num = zsize/box_size;
  if(zsize%box_size > 0)   z_num++;
  xy_num = x_num*y_num;
  const int xyz_num = xy_num * z_num;
  if( xyz_num < 1 )
    return;

  exist_voxel_num = new int[ xyz_num ];
  nFeatures = new ColumnVector[ xyz_num ];

  //*********************************//
  //* 全ての分割領域からのCCHLAC特徴抽出 *//
  //*********************************//
      
  ColumnVector feature(DIM_COLOR_1_3+DIM_COLOR_BIN_1_3);
  ColumnVector feature1;
  ColumnVector feature2;
            
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
	    
	//* CCHLAC特徴抽出
	CCHLAC::extractColorCHLAC( feature1, voxel, sx, sy, sz, gx, gy, gz );
	CCHLAC::extractColorCHLAC_bin( feature2, voxel_bin, sx, sy, sz, gx, gy, gz );
	for(int t=0;t<DIM_COLOR_1_3;t++)
	  feature(t) = feature1(t);
	for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
	  feature(t+DIM_COLOR_1_3) = feature2(t);
	exist_voxel_num[ x + y*x_num + z*xy_num ] = ( feature2(0) + feature2(1) ) * 3 + 0.001; // ボクセルの数
	    
	nFeatures[ x + y*x_num + z*xy_num ] = axis_p * feature;
// 	if( whitening ){
// 	  for(int t=0;t<dim;t++)
// 	    nFeatures[ x + y*x_num + z*xy_num ](t) /= sqrt( var(t) );
// 	}
	    
	//*************************************//
	//* 積分する（viola-jones積分画像の特徴版） *//
	//*************************************//
	    
	if(z==0){
	  if(y==0){
	    if(x!=0){ // (1,0,0)
	      exist_voxel_num[ x + y*x_num + z*xy_num ] += exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ] += nFeatures[ ( x - 1 ) + y*x_num + z*xy_num ];
	    }
	  }
	  else{
	    if(x==0){ // (0,1,0)
	      exist_voxel_num[ x + y*x_num + z*xy_num ] += exist_voxel_num[ x + ( y - 1 )*x_num + z*xy_num ];	    
	      nFeatures[ x + y*x_num + z*xy_num ] += nFeatures[ x + ( y - 1 )*x_num + z*xy_num ];	    
	    }
	    else{ // (1,1,0)
	      exist_voxel_num[ x + y*x_num + z*xy_num ] 
		+= exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_voxel_num[ x + ( y - 1 )*x_num + z*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + ( y - 1 )*x_num + z*xy_num ];
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
	      exist_voxel_num[ x + y*x_num + z*xy_num ] += exist_voxel_num[ x + y*x_num + ( z - 1 )*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ] += nFeatures[ x + y*x_num + ( z - 1 )*xy_num ];
	    }
	    else {// (1,0,1)
	      exist_voxel_num[ x + y*x_num + z*xy_num ] 
		+= exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_voxel_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ]
		+= nFeatures[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  nFeatures[ x + y *x_num + ( z - 1 )*xy_num ]
		-  nFeatures[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ];
	    }
	  }
	  else{
	    if(x==0){ // (0,1,1)
	      exist_voxel_num[ x + y*x_num + z*xy_num ] 
		+= exist_voxel_num[ x + ( y - 1 )*x_num + z *xy_num ]
		+  exist_voxel_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ x + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	      nFeatures[ x + y*x_num + z*xy_num ]
		+= nFeatures[ x + ( y - 1 )*x_num + z *xy_num ]
		+  nFeatures[ x + y *x_num + ( z - 1 )*xy_num ]
		-  nFeatures[ x + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
	    }
	    else{ // (1,1,1)
	      exist_voxel_num[ x + y*x_num + z*xy_num ] 
		+= exist_voxel_num[ ( x - 1 ) + y*x_num + z*xy_num ]
		+  exist_voxel_num[ x + ( y - 1 )*x_num + z*xy_num ]
		+  exist_voxel_num[ x + y *x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + ( y - 1 )*x_num + z *xy_num ]
		-  exist_voxel_num[ x + ( y - 1 )*x_num + ( z - 1 )*xy_num ]
		-  exist_voxel_num[ ( x - 1 ) + y *x_num + ( z - 1 )*xy_num ]
		+  exist_voxel_num[ ( x - 1 ) + ( y - 1 ) *x_num + ( z - 1 )*xy_num ];
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
      }
    }
  }
}

const int SearchObj::maxXrange( int num ){ return xRange( max_mode[ num ] ); }
const int SearchObj::maxYrange( int num ){ return yRange( max_mode[ num ] ); }
const int SearchObj::maxZrange( int num ){ return zRange( max_mode[ num ] ); }
