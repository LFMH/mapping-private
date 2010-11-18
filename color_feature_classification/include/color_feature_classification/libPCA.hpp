#ifndef MY_PCA_HPP
#define MY_PCA_HPP

#include <vector>
#include <Eigen3/Eigenvalues>

/**********************/
/* ����ʬʬ�Ϥ򤹤륯�饹 */
/**********************/

class PCA{
public:
  PCA( bool _mean_flg = true ); // ������ع��󤫤饵��ץ�ʿ���ͤ�Ҥ��ʤ�����false��
  ~PCA(){}

  //* ��ħ�٥��ȥ���ɤ߹��ߤʤ��鼫����ع����׻����Ƥ���
  void addData( std::vector<float> &feature );

  //* ����ʬʬ�Ϥ�Ԥ�
  void solve();
    
  //* ����ʬ���Υޥȥ�å����μ���
  const Eigen3::MatrixXf &Axis() const { return axis; }

  //* ��ͭ�ͤ��¤٤��٥��ȥ�μ���
  const Eigen3::VectorXf &Variance() const { return variance; }

  //* �ǡ�����ʿ�ѥ٥��ȥ�μ���
  const Eigen3::VectorXf &Mean() const;
    
  //* PCA�Υǡ�����ե����뤫���ɤ߹���
  void read( const char *filename, bool ascii = false );

  //* PCA�Υǡ�����ե�����˽���
  void write( const char *filename, bool ascii = false );
    
private:
  int dim;               // ��ħ����
  bool mean_flg;         // ������ع��󤫤�ʿ���ͤ�Ҥ����ݤ��Υե饰
  long long nsample;     // ����ץ��
  Eigen3::VectorXf mean;     // ��ħ�̤�ʿ��
  Eigen3::MatrixXf correlation;    // ������ع���
  Eigen3::MatrixXf axis;           // ����ʬ��
  Eigen3::VectorXf variance; // ��ͭ�ͤ��¤٤��٥��ȥ�

//   //* ��ͭ�ͤ��礭����˸�ͭ�ͤȸ�ͭ�٥��ȥ���¤��ؤ���
//   void sortVecAndVal( Eigen3::MatrixXf &vecs, Eigen3::VectorXf &vals );  
};

#endif
