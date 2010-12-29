#ifndef MY_LINEAR_HPP
#define MY_LINEAR_HPP

#include <iostream>

#include <octave/config.h>
#include <octave/Matrix.h>

#include "../util/DataMatrix.hpp"

namespace coolVisi
{
  /*!
   ����SVM, LINEAR��Ԥ����饹
   ��libLINEAR�򤽤Τޤްܿ���
   */
  class LINEAR
  {
  public:
    /*************
     *  methods  *
     *************/
  public:
    /* ���󥹥ȥ饯�����ǥ��ȥ饯�� */
    LINEAR( bool print_flg = true );
    ~LINEAR();

    void setParam( char c, int val );
    void setParam( char c, double val );
    void set_solver_type( int val );
    void set_cost( double val );
    void set_epsilon( double val );
    void set_bias( double val );
    void set_probability( int val );
    void set_lower( double val );
    void set_upper( double val );
    void set_weight( int class_num, double w );
    void set_y( double yl, double yu );

    void crossValidation( DataMatrix *data, int nc, int nr_fold = 10, bool scaling_flg = true );
    void crossValidation( const char *filename, int nr_fold = 10, bool scaling_flg = true );
    void solve( DataMatrix *data, int nc, bool scaling_flg = true );
    void solve( const char *filename, bool scaling_flg = true );
    int recognize( const ColumnVector &v ) const ;
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
    // libLINEAR����
    struct _liblinear_impl;
    _liblinear_impl *liblinear;

    int nclass;          // ���饹��
    //int dim; // �ѻߡ������� prob.n ����ħ������
    bool eps_default_flg; // true �ξ��param.eps��default���ͤ�
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
    int probability;

  };
};

#endif
