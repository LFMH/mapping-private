#ifndef MY_SVM_HPP
#define MY_SVM_HPP

#include <iostream>

#include <octave/config.h>
#include <octave/Matrix.h>

#include "../util/DataMatrix.hpp"

namespace coolVisi
{
  /*!
   SVM��Ԥ����饹
   ��libSVM�򤽤Τޤްܿ���
   */
  class SVM
  {
  public:
    /*************
     *  methods  *
     *************/
  public:
    /* ���󥹥ȥ饯�����ǥ��ȥ饯�� */
    SVM( bool print_flg = true );
    ~SVM();

    void setParam( char c, int val );
    void setParam( char c, double val );
    void set_svm_type( int val );
    void set_kernel_type( int val );
    void set_degree( int val );
    void set_gamma( double val );
    void set_coef0( double val );
    void set_cost( double val );
    void set_nu( double val );
    void set_p( double val );
    void set_cache_size( double val );
    void set_epsilon( double val );
    void set_shrinking( int val );
    void set_probability( int val );
    void set_lower( double val );
    void set_upper( double val );
    void set_weight( int class_num, double w );
    void set_y( double yl, double yu );

    void crossValidation( DataMatrix *data, int nc, int nr_fold = 10, bool scaling_flg = true );
    void crossValidation( const char *filename, int nr_fold = 10, bool scaling_flg = true );
    void solve( DataMatrix *data, int nc, bool scaling_flg = true );
    void solve( const char *filename, bool scaling_flg = true );
    double recognize( const ColumnVector &v, bool check_flg = true ) const ;
    const int nClass() const { return nclass; }
    double *get_prob_estimates() const { return prob_estimates; }

    /*�ե����������*/
    void read( const char *projname );
    void write( const char *projname ) const ;

  private:
    void do_cross_validation( int nr_fold );
    void setData( DataMatrix *data, int nc );
    void setData( const char *filename );
    const char* readline( FILE *input, char *line );
    double y_scaling( double value ) const ;
    double scaling( int index, double value ) const ;
    void minmax( DataMatrix *data, int nc );
    void minmax( const char *filename );
    void save_scale( const char *filename ) const ;
    void load_scale( const char *filename );
    void exit_with_help();
    /* ���顼(�إ��)ɽ�� */
    void print_help( char **err_message );

    /*******
     * �ѿ�
     *******/
  protected:
    // libSVM�μ������ä��뤿��ι�¤��
    struct _libsvm_impl;
    _libsvm_impl *libsvm;

    int nclass;          // ���饹��
    int dim;             // ��ħ����
    double lower;
    double upper;
    double y_lower;
    double y_upper;
    double y_min;
    double y_max;
    bool is_scaling;
    bool is_y_scaling;
    double *feature_min;
    double *feature_max;
    double *prob_estimates;

  };
};

#endif
