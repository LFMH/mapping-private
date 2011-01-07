#include <cstdio>
#include <iostream>
#include <fstream>
#include "color_feature_classification/libPCA.hpp"

using namespace std;

PCA::PCA( bool _mean_flg ) :
  dim(-1),
  mean_flg(_mean_flg),
  nsample(0) {
}

//***********************************************************
//* add feature vectors to the correlation matrix one by one
void PCA::addData( std::vector<float> &feature ){
  if( dim == -1 ){ // initial value
    dim = feature.size();
    correlation = MatrixXf::Zero( dim, dim );
    mean = VectorXf::Zero( dim );
  }
  else if( feature.size() != (std::vector<float>::size_type)dim ){
    cerr << "ERR (in PCA::addData): vector size differs" << endl;
    exit( EXIT_FAILURE );
  }

  if(mean_flg)
    for( int i=0; i<dim; i++ )
      mean(i) += feature[i];
  for( int i = 0; i < dim; i++ ){
    const float val = feature[ i ];
    int idx = i + i * dim;
    for(int j = i; j < dim; j++ )
      correlation( idx++ ) += val * feature[ j ];
  }
  nsample++;
}

//************
//* solve PCA
void PCA::solve( bool regularization_flg, float regularization_nolm ){
  if( dim==-1 ){
    cerr << "ERR (in PCA::solve): there is no data" << endl;
    exit( EXIT_FAILURE );
  }
  printf("sample: %lld\n",nsample);

  const double inv_nsample = 1.0 / (double)nsample;
  for(int i = 0; i < dim; i++ ){
    int idx = i + i * dim;
    for( int j = i; j < dim; j++ )
      correlation( idx++ ) *= inv_nsample;
  }
  for( int i = 0; i < dim; i++ )
    for( int j = i + 1; j < dim; j++ )
      correlation( i, j ) = correlation( j, i );

  if( mean_flg ){
    for(int i = 0; i < dim; i++ )
      mean(i) *= inv_nsample;    
    correlation -= mean * mean.transpose();
  }

  if( regularization_flg )
    for(int i = 0; i < dim; i++ )
      correlation( i, i ) += regularization_nolm;

  //* solve eigen problem
  SelfAdjointEigenSolver< MatrixXf > pca ( correlation );
  MatrixXf tmp_axis = pca.eigenvectors();
  VectorXf tmp_variance = pca.eigenvalues();
  sortVecAndVal( tmp_axis, tmp_variance );  
}

//*****************************************
//* get the mean vector of feature vectors
const VectorXf& PCA::Mean() const {
  if( !mean_flg ){
    cerr << "ERR (in PCA::Mean): There is no mean vector (mean_flg=false)." << endl;
    exit( EXIT_FAILURE );
  }
  return mean;
}

//****************
//* read PCA file
void PCA::read( const char *filename, bool ascii )
{
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "r" );
  else
    fp = fopen( filename, "rb" );

  if( ascii ){
    //* dimension of feature vectors
    if( fscanf( fp, "%d\n", &dim ) != 1 ) cerr << "fscanf err." << endl;

    //* eigen vectors
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	if( fscanf( fp, "%f ", &( axis( j, i ) ) ) != 1 ) cerr << "fscanf err." << endl;
    
    //* eigen values
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      if( fscanf( fp, "%f\n", &( variance( i ) ) ) != 1 ) cerr << "fscanf err." << endl;

    //* mean vector of feature vectors
    float tmpVal;
    if( fscanf( fp, "%f\n", &tmpVal ) != 1 ){
      mean_flg = false;
    }
    else{
      mean_flg = true;
      mean.resize( dim );
      mean( 0 ) = tmpVal;
      for( int i=1; i<dim; i++ )
	if( fscanf( fp, "%f\n", &( mean( i ) ) ) != 1 ) cerr << "fscanf err." << endl;
    }
  }else{
    //* dimension of feature vectors
    if( fread( &dim, sizeof(int), 1, fp ) != 1 ) cerr << "fread err." << endl;

    //* eigen vectors
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	if( fread( &( axis( j, i ) ), sizeof(float), 1, fp ) != 1 ) cerr << "fread err." << endl;

    //* eigen values
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      if( fread( &( variance( i ) ), sizeof(float), 1, fp ) != 1 ) cerr << "fread err." << endl;

    //* mean vector of feature vectors
    float tmpVal;
    if( fread( &tmpVal, sizeof(float), 1, fp ) != 1 ){
      mean_flg = false;
    }
    else{
      mean_flg = true;
      mean.resize( dim );
      mean( 0 ) = tmpVal;
      for( int i=1; i<dim; i++ )
	if( fread( &( mean( i ) ), sizeof(float), 1, fp ) != 1 ) cerr << "fread err." << endl;
    }
  }
  
  fclose( fp );
}


//*****************
//* write PCA file
void PCA::write( const char *filename, bool ascii )
{ 
  const int dim = variance.size();
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "w" );
  else
    fp = fopen( filename, "wb" );

  if( ascii ){
    //* dimension of feature vectors
    fprintf( fp, "%d\n", dim );

    //* eigen vectors
    for( int i=0; i<dim; i++ ){
      for( int j=0; j<dim; j++ )
	fprintf( fp, "%f ", axis( j, i ) );
      fprintf( fp, "\n" );
    }

    //* eigen values
    for( int i=0; i<dim; i++ )
      fprintf( fp, "%f\n", variance( i ) );

    if( mean_flg )
      //* mean vector of feature vectors
      for( int i=0; i<dim; i++ )
	fprintf( fp, "%f\n", mean( i ) );

  }else{
    //* dimension of feature vectors
    fwrite( &dim, sizeof(int), 1, fp );

    //* eigen vectors
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	fwrite( &( axis( j, i ) ), sizeof(float), 1, fp );

    //* eigen values
    for( int i=0; i<dim; i++ )
      fwrite( &( variance( i ) ), sizeof(float), 1, fp );

    if( mean_flg )
      //* mean vector of feature vectors
      for( int i=0; i<dim; i++ )
	fwrite( &( mean( i ) ), sizeof(float), 1, fp );
  }
  
  fclose( fp );
}

//* private function
void PCA::sortVecAndVal( MatrixXf &vecs, VectorXf &vals ){
  int *index = new int[ dim ];
  for( int i = 0; i < dim; i++ )
    index[ i ] = i;

  //* sort
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
  
  //* copy
  axis.resize( dim, dim );
  variance.resize( dim );
  for( int i = 0; i < dim; i++ ){
    variance( i ) = vals( index[ i ] );
    for( int j = 0; j < dim; j++ )
      axis( j, i ) = vecs( j, index[ i ] );
  }

  delete[] index;  
}
