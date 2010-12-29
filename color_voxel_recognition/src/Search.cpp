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

//* ���ַ�¬��
double my_clock()
{
  struct timeval tv;
  gettimeofday(&tv, NULL);
  return tv.tv_sec + (double)tv.tv_usec*1e-6;
}

//***************************//
//* ���󥹥ȥ饯���ȥǥ��ȥ饯�� *//
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
//* �ѿ��Υ��å� *//
//**************//

//***************************
//* �����о�ʪ�ΤΥ���������򥻥å�
void SearchObj::setRange( int _range1, int _range2, int _range3 ){
  range1 = _range1;
  range2 = _range2;
  range3 = _range3;
}

//************************************
//* ��󥯲��̤ޤǤ��ΰ����Ϥ��뤫���򥻥å�
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
//* �����ΰ����˴ޤޤ��ܥ��������������̤�����ä����Ǥ����뤫�����ͤ򥻥å�
void SearchObj::setThreshold( int _exist_voxel_num_threshold ){
  exist_voxel_num_threshold = _exist_voxel_num_threshold;
}

//**********************************
//* �����о�ʪ�Τ���ʬ���֤δ��켴���ɤ߹���
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
//* scene�Ρ�ʬ���ΰ���Ρ���ħ�̤ȥܥ���������ɤ߹���
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
//* ���Хܥå�������Ĺ�μ����˴ؤ���ؿ� *//
//*********************************//

//*************************************************
//* �����θ��Х⡼�ɤˤ����븡�Хܥå�����x,y,z�դ�Ĺ�������
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
//* �����θ��Х⡼�ɤˤ����븡�Хܥå�����x�դ�Ĺ�����֤�
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
//* �����θ��Х⡼�ɤˤ����븡�Хܥå�����y�դ�Ĺ�����֤�
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
//* �����θ��Х⡼�ɤˤ����븡�Хܥå�����z�դ�Ĺ�����֤�
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
//* �����ΰ����¸�˴ؤ���ؿ� *//
//*************************//

//***********************************************************************
//* ���ޤǤ�ȯ�������ΰ����ǡ�����ȯ�������ΰ�Ȥ��֤��ΰ褬����С����Υ���ֹ���֤�
//* ���֤��ΰ褬�ʤ���к����󥯤��ֹ���֤�
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
//* ����ȯ�������ΰ�Υ�󥯤򤺤餹
//* �� src_num=2 dest_num=3: 3�̤��ΰ��4�̤˲�����
inline void SearchObj::max_cpy( int src_num, int dest_num ){
  max_dot[ dest_num ] = max_dot[ src_num ];
  max_x[ dest_num ] = max_x[ src_num ];
  max_y[ dest_num ] = max_y[ src_num ];
  max_z[ dest_num ] = max_z[ src_num ];
  max_mode[ dest_num ] = max_mode[ src_num ];
}

//*************************************
//* ��ȯ�������ΰ�����ȯ�������ΰ���֤�������
inline void SearchObj::max_assign( int dest_num, double dot, int x, int y, int z, SearchMode mode ){
  max_dot[ dest_num ] = dot;
  max_x[ dest_num ] = x;
  max_y[ dest_num ] = y;
  max_z[ dest_num ] = z;
  max_mode[ dest_num ] = mode;
}

//****************//
//* ʪ�θ��Фδؿ� *//
//****************//

//*************************
//* ���Ƥθ��Х⡼�ɤǤ�ʪ�θ���
void SearchObj::search(){
  double t1,t2;//���ַ�¬�Ѥ��ѿ�
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
//* �����θ��Х⡼�ɤǤ�ʪ�θ���
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

	  //* �ΰ���˥ܥ����뤬¸�ߤ��뤫�ݤ�������å�
	  exist_num = clipValue( exist_voxel_num, x, y, z, xrange, yrange, zrange );
	  if(exist_num > exist_voxel_num_threshold){ // �ΰ���˥ܥ����뤬������ʾ夢���

	    feature_tmp = clipValue( nFeatures, x, y, z, xrange, yrange, zrange );
	    
	    //* ����ٷ׻�
	    sum = feature_tmp.transpose()*feature_tmp;
	    tmpVector = axis_q * feature_tmp;
#if 0
	    sum = sqrt(sum);
	    dot = tmpVector.transpose()*tmpVector;
	    dot = sqrt( dot );
	    dot /= sum ;
#else // �����Ǥ��®���������Ȥ��Ϥ�����Ǥ�褤
	    dot = tmpVector.transpose()*tmpVector;
	    dot /= sum ;
#endif
	    //* ����٤��⤱�����¸
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
//* ����ͤ���ʬ���줿���󤫤餽�ΰ��֤ˤ������ͤ���Ф�
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
//* ��̤�ե�����˽���
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
  fprintf(fp,"time: %f\n",search_time);//�в����
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
///    ����饤���������    ///
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
  //* ʬ���ΰ�θĿ���Ĵ�٤�
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
  //* ���Ƥ�ʬ���ΰ褫���CCHLAC��ħ��� *//
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
	    
	//* CCHLAC��ħ���
	CCHLAC::extractColorCHLAC( feature1, voxel, sx, sy, sz, gx, gy, gz );
	CCHLAC::extractColorCHLAC_bin( feature2, voxel_bin, sx, sy, sz, gx, gy, gz );
	for(int t=0;t<DIM_COLOR_1_3;t++)
	  feature(t) = feature1(t);
	for(int t=0;t<DIM_COLOR_BIN_1_3;t++)
	  feature(t+DIM_COLOR_1_3) = feature2(t);
	exist_voxel_num[ x + y*x_num + z*xy_num ] = ( feature2(0) + feature2(1) ) * 3 + 0.001; // �ܥ�����ο�
	    
	nFeatures[ x + y*x_num + z*xy_num ] = axis_p * feature;
// 	if( whitening ){
// 	  for(int t=0;t<dim;t++)
// 	    nFeatures[ x + y*x_num + z*xy_num ](t) /= sqrt( var(t) );
// 	}
	    
	//*************************************//
	//* ��ʬ�����viola-jones��ʬ��������ħ�ǡ� *//
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
