#ifndef MY_PCA_EIGEN_HPP
#define MY_PCA_EIGEN_HPP

#include <vector>
#include <Eigen/Eigenvalues>

using namespace Eigen;

/*****************/
/* class for PCA */
/*****************/

class PCA{
public:
  bool mean_flg;        // "true" when you substract the mean vector from the correlation matrix

  PCA( bool _mean_flg = true ); // _mean_flg should be "true" when you substract the mean vector from the correlation matrix
  
  ~PCA(){}
  
  // add feature vectors to the correlation matrix one by one
  void addData( const std::vector<float> feature );

  // solve PCA
  void solve( bool regularization_flg = false, float regularization_nolm = 0.0001 );
    
  // get eigen vectors
  const MatrixXf &Axis() const { return axis; }

  // get eigen values
  const VectorXf &Variance() const { return variance; }

  // get the mean vector of feature vectors
  const VectorXf &Mean() const;
    
  // read PCA file
  void read( const char *filename, bool ascii = false );

  // write PCA file
  void write( const char *filename, bool ascii = false );
    
private:
  int dim;              // dimension of feature vectors
  long long nsample;    // number of feature vectors
  VectorXf mean;        // mean vector of feature vectors
  MatrixXf correlation; // self correlation matrix
  MatrixXf axis;        // eigen vectors
  VectorXf variance;    // eigen values

  void sortVecAndVal( MatrixXf &vecs, VectorXf &vals );
};

#endif
