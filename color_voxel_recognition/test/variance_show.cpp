#include <iostream>
#include <stdio.h>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/libPCA.hpp>
#include "./FILE_MODE"

using namespace std;

/*************************************************/
/* �ؽ�ʪ�Τμ���ʬ�θ�ͭ�ͤδ�ͿΨ���ǧ����            */
/* -d <������> �Ȥ���С����μ������δ�ͿΨ��ɽ������    */
/* -c <��ͿΨ> �Ȥ���С����δ�ͿΨ�ˤʤ뼡������ɽ������ */
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

  //* ���̤���CCHLAC��ħ�٥��ȥ�μ��������ɤ߹���
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
