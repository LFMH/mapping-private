#ifndef MY_PCA_HPP
#define MY_PCA_HPP

#include <vector>
#include <Eigen3/Eigenvalues>

using namespace Eigen3;

/*****************/
/* class for PCA */
/*****************/

class PCA{
public:
  PCA( bool _mean_flg = true ); // _mean_flg should be "true" when you substract the mean vector from the correlation matrix
  
  ~PCA(){}
  
  //* add feature vectors to the correlation matrix one by one
  void addData( std::vector<float> &feature );

  //* solve PCA
  void solve();
    
  //* get eigen vectors
  const MatrixXf &Axis() const { return axis; }

  //* get eigen values
  const VectorXf &Variance() const { return variance; }

  //* get the mean vector of feature vectors
  const VectorXf &Mean() const;
    
  //* read PCA file
  void read( const char *filename, bool ascii = false );

  //* write PCA file
  void write( const char *filename, bool ascii = false );
    
private:
  int dim;              // dimension of feature vectors
  bool mean_flg;        // "true" when you substract the mean vector from the correlation matrix
  long long nsample;    // number of feature vectors
  VectorXf mean;        // mean vector of feature vectors
  MatrixXf correlation; // self correlation matrix
  MatrixXf axis;        // eigen vectors
  VectorXf variance;    // eigen values
};

#endif
