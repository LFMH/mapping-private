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

//*********************************************
//* 特徴ベクトルを読み込みながら自己相関行列を計算していく
void PCA::addData( std::vector<float> &feature ){
  if( dim == -1 ){ // 初期
    dim = feature.size();
    correlation = Eigen3::MatrixXf::Zero( dim, dim );
    mean = Eigen3::VectorXf::Zero( dim );
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

//***************
//* 主成分分析を行う
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
  
  //* 固有値問題を解く
  Eigen3::SelfAdjointEigenSolver< Eigen3::MatrixXf > pca ( correlation );
  axis = pca.eigenvectors();
  variance = pca.eigenvalues();

//   //* 結果を受けとる
//   Matrix tmp_vectors = real( eig.eigenvectors() );
//   std::vector<float> tmp_values = real( eig.eigenvalues() );
  
//   //* 固有値を大きい順に
//   sortVecAndVal( tmp_vectors, tmp_values );  
}

//************************
//* データの平均ベクトルの取得
inline const Eigen3::VectorXf& PCA::Mean() const {
  if( !mean_flg ){
    cerr << "ERR (in PCA::Mean): There is no mean vector (mean_flg=false)." << endl;
    exit( EXIT_FAILURE );
  }
  return mean;
}

//*******************
//* ファイルから読み込み
void PCA::read( const char *filename, bool ascii )
{
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "r" );
  else
    fp = fopen( filename, "rb" );

  if( ascii ){
    //* 特徴次元
    if( fscanf( fp, "%d\n", &dim ) != 1 ) cerr << "fscanf err." << endl;

    //* 主成分軸のマトリックス
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	if( fscanf( fp, "%f ", &( axis( j, i ) ) ) != 1 ) cerr << "fscanf err." << endl;
    
    //* 固有値を並べたベクトル
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      if( fscanf( fp, "%f\n", &( variance( i ) ) ) != 1 ) cerr << "fscanf err." << endl;

    //* データの平均ベクトル
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
    //* 特徴次元
    if( fread( &dim, sizeof(int), 1, fp ) != 1 ) cerr << "fread err." << endl;

    //* 主成分軸のマトリックス
    axis.resize( dim, dim );
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	if( fread( &( axis( j, i ) ), sizeof(float), 1, fp ) != 1 ) cerr << "fread err." << endl;

    //* 固有値を並べたベクトル
    variance.resize( dim );
    for( int i=0; i<dim; i++ )
      if( fread( &( variance( i ) ), sizeof(float), 1, fp ) != 1 ) cerr << "fread err." << endl;

    //* データの平均ベクトル
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
//* ファイルへ書き出し
void PCA::write( const char *filename, bool ascii )
{ 
  const int dim = variance.size();
  FILE *fp;
  
  if( ascii )
    fp = fopen( filename, "w" );
  else
    fp = fopen( filename, "wb" );

  if( ascii ){
    //* 特徴次元
    fprintf( fp, "%d\n", dim );

    //* 主成分軸のマトリックス
    for( int i=0; i<dim; i++ ){
      for( int j=0; j<dim; j++ )
	fprintf( fp, "%f ", axis( j, i ) );
      fprintf( fp, "\n" );
    }

    //* 固有値を並べたベクトル
    for( int i=0; i<dim; i++ )
      fprintf( fp, "%f\n", variance( i ) );

    if( mean_flg )
      //* データの平均ベクトル
      for( int i=0; i<dim; i++ )
	fprintf( fp, "%f\n", mean( i ) );

  }else{
    //* 特徴次元
    fwrite( &dim, sizeof(int), 1, fp );

    //* 主成分軸のマトリックス
    for( int i=0; i<dim; i++ )
      for( int j=0; j<dim; j++ )
	fwrite( &( axis( j, i ) ), sizeof(float), 1, fp );

    //* 固有値を並べたベクトル
    for( int i=0; i<dim; i++ )
      fwrite( &( variance( i ) ), sizeof(float), 1, fp );

    if( mean_flg )
      //* データの平均ベクトル
      for( int i=0; i<dim; i++ )
	fwrite( &( mean( i ) ), sizeof(float), 1, fp );
  }
  
  fclose( fp );
}

//**************//
//* private関数 *//
//**************//

// //*******************************************
// //* 固有値の大きい順に固有値と固有ベクトルを並び替える
// void PCA::sortVecAndVal( Matrix &vecs, std::vector<float> &vals ){
//   int *index = new int[ dim ];
//   for( int i = 0; i < dim; i++ )
//     index[ i ] = i;

//   //* 並び替え
//   int tmpIndex;
//   for( int i = 0; i < dim; i++ ){
//     for( int j = 1; j < dim - i; j++ ){
//       if( vals( index[ j - 1 ] ) < vals( index[ j ] ) ){
// 	tmpIndex       = index[ j ];
// 	index[ j ]     = index[ j - 1 ];
// 	index[ j - 1 ] = tmpIndex;
//       }
//     }
//   }
  
//   //* 軸と分散の取得
//   axis.resize( dim, dim );
//   variance.resize( dim );
//   for( int i = 0; i < dim; i++ ){
//     variance( i ) = vals( index[ i ] );
//     for( int j = 0; j < dim; j++ )
//       axis( j, i ) = vecs( j, index[ i ] );
//   }

//   //* メモリ解放
//   delete[] index;  
// }
