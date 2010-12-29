#include <cstdio>
#include <iostream>
#include <fstream>
#include "color_voxel_recognition/libPCA.hpp"

using namespace std;

PCA::PCA( bool _mean_flg ) :
  dim(-1),
  mean_flg(_mean_flg),
  nsample(0) {
}

//*********************************************
//* ��ħ�٥��ȥ���ɤ߹��ߤʤ��鼫����ع����׻����Ƥ���
void PCA::addData( ColumnVector &feature ){
  if( dim == -1 ){ // ���
    dim = feature.length();
    correlation.resize( dim, dim );
    mean.resize( dim );
  }
  else if( feature.length() != dim ){
    cerr << "ERR (in PCA::addData): vector length differs" << endl;
    exit( EXIT_FAILURE );
  }

  if(mean_flg)
    mean += feature;
  for( int i = 0; i < dim; i++ ){
    const double val = feature( i );
    int idx = i + i * dim;
    for(int j = i; j < dim; j++ )
      correlation.xelem( idx++ ) += val * feature( j );
  }
  nsample++;
}

//***************
//* ����ʬʬ�Ϥ�Ԥ�
void PCA::solve(){
  if( dim==-1 ){
    cerr << "ERR (in PCA::solve): there is no data" << endl;
    exit( EXIT_FAILURE );
  }
  printf("sample: %lld\n",nsample);

  const double inv_nsample = 1.0 / (double)nsample;
  for(int i = 0; i < dim; i++ ){
    int idx = i + i * dim;
    for( int j = i; j < dim; j++ )
      correlation.xelem( idx++ ) *= inv_nsample;
  }
  for( int i = 0; i < dim; i++ )
    for( int j = i + 1; j < dim; j++ )
      correlation.xelem( i, j ) = correlation.xelem( j, i );

  if( mean_flg ){
    for(int i = 0; i < dim; i++ )
      mean(i) *= inv_nsample;    
    correlation -= mean * mean.transpose();
  }
  
  //* ��ͭ��������
  EIG eig( correlation );

  //* ��̤�����Ȥ�
  Matrix tmp_vectors = real( eig.eigenvectors() );
  ColumnVector tmp_values = real( eig.eigenvalues() );
  
  //* ��ͭ�ͤ��礭�����
  sortVecAndVal( tmp_vectors, tmp_values );  
}

//************************
//* �ǡ�����ʿ�ѥ٥��ȥ�μ���
inline const ColumnVector& PCA::Mean() const {
  if( !mean_flg ){
    cerr << "ERR (in PCA::Mean): There is no mean vector (mean_flg=false)." << endl;
    exit( EXIT_FAILURE );
  }
  return mean;
}

//*******************
//* �ե����뤫���ɤ߹���
void PCA::read( const char *filename, bool ascii )
{
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "r" );
  else
    fp = fopen( filename, "rb" );

  if( ascii ){
    //* ��ħ����
    fscanf( fp, "%d\n", &dim );

    //* ����ʬ���Υޥȥ�å���
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	fscanf( fp, "%lf ", &( axis( j, i ) ) );
    
    //* ��ͭ�ͤ��¤٤��٥��ȥ�
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      fscanf( fp, "%lf\n", &( variance( i ) ) );

    //* �ǡ�����ʿ�ѥ٥��ȥ�
    double tmpVal;
    if( fscanf( fp, "%lf\n", &tmpVal ) == EOF ){
      mean_flg = false;
    }
    else{
      mean_flg = true;
      mean.resize( dim );
      mean( 0 ) = tmpVal;
      for( int i=1; i<dim; i++ )
	fscanf( fp, "%lf\n", &( mean( i ) ) );
    }
  }else{
    //* ��ħ����
    fread( &dim, sizeof(int), 1, fp );

    //* ����ʬ���Υޥȥ�å���
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	fread( &( axis( j, i ) ), sizeof(double), 1, fp );

    //* ��ͭ�ͤ��¤٤��٥��ȥ�
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      fread( &( variance( i ) ), sizeof(double), 1, fp );

    //* �ǡ�����ʿ�ѥ٥��ȥ�
    double tmpVal;
    if( fread( &tmpVal, sizeof(double), 1, fp ) != 1 ){
      mean_flg = false;
    }
    else{
      mean_flg = true;
      mean.resize( dim );
      mean( 0 ) = tmpVal;
      for( int i=1; i<dim; i++ )
	fread( &( mean( i ) ), sizeof(double), 1, fp );
    }
  }
  
  fclose( fp );
}


//*****************
//* �ե�����ؽ񤭽Ф�
void PCA::write( const char *filename, bool ascii )
{ 
  const int dim = variance.length();
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "w" );
  else
    fp = fopen( filename, "wb" );

  if( ascii ){
    //* ��ħ����
    fprintf( fp, "%d\n", dim );

    //* ����ʬ���Υޥȥ�å���
    for( int i=0; i<dim; i++ ){
      for( int j=0; j<dim; j++ )
	fprintf( fp, "%f ", axis( j, i ) );
      fprintf( fp, "\n" );
    }

    //* ��ͭ�ͤ��¤٤��٥��ȥ�
    for( int i=0; i<dim; i++ )
      fprintf( fp, "%f\n", variance( i ) );

    if( mean_flg )
      //* �ǡ�����ʿ�ѥ٥��ȥ�
      for( int i=0; i<dim; i++ )
	fprintf( fp, "%f\n", mean( i ) );

  }else{
    //* ��ħ����
    fwrite( &dim, sizeof(int), 1, fp );

    //* ����ʬ���Υޥȥ�å���
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	fwrite( &( axis( j, i ) ), sizeof(double), 1, fp );

    //* ��ͭ�ͤ��¤٤��٥��ȥ�
    for( int i=0; i<dim; i++ )
      fwrite( &( variance( i ) ), sizeof(double), 1, fp );

    if( mean_flg )
      //* �ǡ�����ʿ�ѥ٥��ȥ�
      for( int i=0; i<dim; i++ )
	fwrite( &( mean( i ) ), sizeof(double), 1, fp );
  }
  
  fclose( fp );
}

//**************//
//* private�ؿ� *//
//**************//

//*******************************************
//* ��ͭ�ͤ��礭����˸�ͭ�ͤȸ�ͭ�٥��ȥ���¤��ؤ���
void PCA::sortVecAndVal( Matrix &vecs, ColumnVector &vals ){
  int *index = new int[ dim ];
  for( int i = 0; i < dim; i++ )
    index[ i ] = i;

  //* �¤��ؤ�
  int tmpIndex;
  for( int i = 0; i < dim; i++ ){
    for( int j = 1; j < dim - i; j++ ){
      if( vals( index[ j - 1 ] ) < vals( index[ j ] ) ){
	tmpIndex       = index[ j ];
	index[ j ]     = index[ j - 1 ];
	index[ j - 1 ] = tmpIndex;
      }
    }
  }
  
  //* ����ʬ���μ���
  axis.resize( dim, dim );
  variance.resize( dim );
  for( int i = 0; i < dim; i++ ){
    variance( i ) = vals( index[ i ] );
    for( int j = 0; j < dim; j++ )
      axis( j, i ) = vecs( j, index[ i ] );
  }

  //* �������
  delete[] index;  
}
