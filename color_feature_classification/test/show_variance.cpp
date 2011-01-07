#include <pcl/common/eigen.h>
#include <iostream>
#include "color_feature_classification/libPCA.hpp"

using namespace Eigen3;
using namespace std;

int main( int argc, char** argv ){

  if( argc != 2 ){
    cerr << "Need one parameter! Syntax is: " << argv[0] << " {pca_file_name}" << endl;
    return(-1);
  }
  PCA pca;
  pca.read( argv[1], false );
  cout << pca.Variance() << endl;
  // VectorXf var = pca.Variance();
  // cout << var << endl;

}
