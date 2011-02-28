#include <iostream>
#include <stdio.h>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/libPCA.hpp>
#include "./FILE_MODE"

using namespace std;

/*************************************************/
/* 学習物体の主成分の固有値の寄与率を確認する            */
/* -d <次元数> とすれば、その次元数の寄与率を表示する    */
/* -c <寄与率> とすれば、その寄与率になる次元数を表示する */
/*************************************************/

int main(int argc, char** argv)
{
  if((argc!=2)&&(argc!=4)){
    cerr << "usage: " << argv[0] << " [model_name]" << endl;
    cerr << " or" << endl;
    cerr << "usage: " << argv[0] << " [model_name] -d(or -c) <val>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 100 ];
  sprintf( tmpname, "models/%s/pca_result", argv[1] );

  PCA pca_each;
  pca_each.read( tmpname ,ASCII_MODE_P );
  VectorXf variance = pca_each.Variance();

  //* 圧縮したCCHLAC特徴ベクトルの次元数の読み込み
  const int dim = Param::readDim();

  if( argc==2 ){
    for( int i=0; i<dim; i++ )
      printf("%f\n",variance(i));
  }
  else{
    double c_all = 0;
    for( int i=0; i<dim; i++ )
      c_all += variance( i );

    if( argv[2][1]=='d' ){
      const int d = atoi(argv[3]);
      double c_ = 0;
      for( int i=0; i<d; i++ )
	c_ += variance( i );
      printf("%f\n",c_/c_all);
    }
    else if( argv[2][1]=='c' ){
      const double c = atof(argv[3]);
      double c_ = 0;
      for( int i=0; i<dim; i++ ){
	c_ += variance( i );
	if( c_/c_all >= c ){
	  printf("%d\n",i);
	  return 1;
	}
      }
      printf("%d\n",dim);
    }
    else
      cerr << "ERR: option command must be \"-d\" or \"-c\"" << endl;
  }
}
