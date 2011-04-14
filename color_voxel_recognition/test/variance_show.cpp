#include <iostream>
#include <stdio.h>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/libPCA.hpp>
#include "color_voxel_recognition/FILE_MODE"

using namespace std;

/*******************************************************************************************************************/
/* show eigenvalues of a target object's subspace                                                                  */
/*  options:                                                                                                       */
/* -d <dim> : show the accumulative contribution rate of the dimension                                             */
/* -c <accumulative contribution rate> : show the dimension of the accumulative contribution rate of the dimension */
/*******************************************************************************************************************/

int main(int argc, char** argv)
{
  if((argc!=3)&&(argc!=5)){
    cerr << "usage: " << argv[0] << " [path] [model_name]" << endl;
    cerr << " or" << endl;
    cerr << "usage: " << argv[0] << " [path] [model_name] -d(or -c) <val>" << endl;
    exit( EXIT_FAILURE );
  }
  char tmpname[ 1000 ];
  sprintf( tmpname, "%s/models/%s/pca_result", argv[1], argv[2] );

  PCA pca_each;
  pca_each.read( tmpname ,ASCII_MODE_P );
  VectorXf variance = pca_each.Variance();

  // read the dimension of compressed feature vectors
  sprintf( tmpname, "%s/param/parameters.txt", argv[1] );
  const int dim = Param::readDim( tmpname );

  if( argc==3 ){
    for( int i=0; i<dim; i++ )
      printf("%f\n",variance(i));
  }
  else{
    double c_all = 0;
    for( int i=0; i<dim; i++ )
      c_all += variance( i );

    if( argv[3][1]=='d' ){
      const int d = atoi(argv[4]);
      double c_ = 0;
      for( int i=0; i<d; i++ )
	c_ += variance( i );
      printf("%f\n",c_/c_all);
    }
    else if( argv[3][1]=='c' ){
      const double c = atof(argv[4]);
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
