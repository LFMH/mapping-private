#include <pcl/common/eigen.h>
#include <iostream>
#include "color_voxel_recognition/libPCA.hpp"

using namespace Eigen;
using namespace std;

int main( int argc, char** argv ){

  if( argc < 2 ){
    cerr << "Need at least one parameter! Syntax is: " << argv[0] << " {pca_file_name} [something if you want to see valid dim.]" << endl;
    return(-1);
  }
  PCA pca;
  pca.read( argv[1], false );
  VectorXf var = pca.Variance();
  cout << var << endl;

  if( argc > 2 ){
    cout << endl;
    int dim = var.size();
    for( int i=0; i<dim; i++ ){
      if( var( i ) <= 0 ){
	cout << i << endl;
	break;
      }
    }
  }
}
