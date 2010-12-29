#include "../coolVisi/stat/SVM.hpp"
#include "libSVM.hpp"
#include "ErrorPrint.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include <stdarg.h>
#include <errno.h>
#include <float.h>

using namespace std;
using namespace coolVisi;

void svm_print_null(const char *s) {}
void svm_print_string_stdout(const char *s) {
  fputs(s,stdout);
  fflush(stdout);
}

#define max(x,y) (((x)>(y))?(x):(y))
#define min(x,y) (((x)<(y))?(x):(y))
#define SKIP_TARGET\
	while(isspace(*p)) ++p;\
	while(!isspace(*p)) ++p;

#define SKIP_ELEMENT\
	while(*p!=':') ++p;\
	++p;\
	while(isspace(*p)) ++p;\
	while(*p && !isspace(*p)) ++p;

// libSVMの実装を隠蔽するための構造体
struct SVM::_libsvm_impl {
  struct svm_parameter param;
  struct svm_problem prob;
  struct svm_model *model;
};

/*--------------------------------------------------------------------------
 * コンストラクタ・デストラクタ
 ---------------------------------------------------------------------------*/
SVM::SVM( bool print_flg ) : libsvm(new _libsvm_impl) {
  libsvm->param.svm_type = C_SVC;
  libsvm->param.kernel_type = RBF;
  libsvm->param.degree = 3;
  libsvm->param.gamma = 0;	// 1/num_features
  libsvm->param.coef0 = 0;
  libsvm->param.nu = 0.5;
  libsvm->param.cache_size = 100;
  libsvm->param.C = 1;
  libsvm->param.eps = 1e-3;
  libsvm->param.p = 0.1;
  libsvm->param.shrinking = 1;
  libsvm->param.probability = 0;
  libsvm->param.nr_weight = 0;
  libsvm->param.weight_label = NULL;
  libsvm->param.weight = NULL;
  libsvm->prob.l = 0;
  libsvm->prob.y = NULL;
  libsvm->prob.x = NULL;
  libsvm->model = NULL;
  prob_estimates = NULL;
  lower = -1.0;
  upper = 1.0;
  is_scaling = true;
  is_y_scaling = false;
  y_max = -DBL_MAX;
  y_min = DBL_MAX;
  feature_min = NULL;
  feature_max = NULL;
  dim = 0;
  if( print_flg )
    svm_print_string = &svm_print_string_stdout;
  else
    svm_print_string = &svm_print_null;
}

SVM::~SVM(){
  svm_destroy_param( &libsvm->param );
  if( libsvm->model != NULL )
    svm_destroy_model( libsvm->model );
  if( libsvm->prob.y != NULL ) delete libsvm->prob.y;
  if( libsvm->prob.x != NULL ) delete libsvm->prob.x;
  if( prob_estimates != NULL ) delete prob_estimates;
  if( feature_min != NULL ) delete feature_min;
  if( feature_max != NULL ) delete feature_max;
  delete libsvm;
}

/*--------------------------------------------------------------------------
 * public methods
 ---------------------------------------------------------------------------*/
void SVM::setParam( char c, int val ){
  setParam( c, (double)val );
}

void SVM::setParam( char c, double val ){
  switch( c )
    {
    case 's':
      libsvm->param.svm_type = (int)val;
      break;
    case 't':
      libsvm->param.kernel_type = (int)val;
      break;
    case 'd':
      libsvm->param.degree = (int)val;
      break;
    case 'g':
      libsvm->param.gamma = val;
      break;
    case 'r':
      libsvm->param.coef0 = val;
      break;
    case 'n':
      libsvm->param.nu = val;
      break;
    case 'm':
      libsvm->param.cache_size = val;
      break;
    case 'c':
      libsvm->param.C = val;
      break;
    case 'e':
      libsvm->param.eps = val;
      break;
    case 'p':
      libsvm->param.p = val;
      break;
    case 'h':
      libsvm->param.shrinking = (int)val;
      break;
    case 'b':
      libsvm->param.probability = (int)val;
      break;
    case 'l':
      lower = val;
      break;
    case 'u':
      upper = val;
      break;
    default:
      char *error_message = new char[ 2300 ];
      print_help( &error_message );
      ERR_PRINT( "%s",error_message );
      exit(1);
      //exit_with_help();
    }  
}

void SVM::set_svm_type( int val )      { libsvm->param.svm_type = val; }
void SVM::set_kernel_type( int val )   { libsvm->param.kernel_type = val; }
void SVM::set_degree( int val )        { libsvm->param.degree = val; }
void SVM::set_gamma( double val )      { libsvm->param.gamma = val; }
void SVM::set_coef0( double val )      { libsvm->param.coef0 = val; }
void SVM::set_cost( double val )       { libsvm->param.C = val; }
void SVM::set_nu( double val )         { libsvm->param.nu = val; }
void SVM::set_p( double val )          { libsvm->param.p = val; }
void SVM::set_cache_size( double val ) { libsvm->param.cache_size = val; }
void SVM::set_epsilon( double val )    { libsvm->param.eps = val; }
void SVM::set_shrinking( int val )     { libsvm->param.shrinking = val; }
void SVM::set_probability( int val )   { libsvm->param.probability = val; }
void SVM::set_lower( double val )      { lower = val; }
void SVM::set_upper( double val )      { upper = val; }

void SVM::set_weight( int class_num, double w ){
  ++libsvm->param.nr_weight;
  libsvm->param.weight_label = (int *)realloc(libsvm->param.weight_label,sizeof(int)*libsvm->param.nr_weight);
  libsvm->param.weight = (double *)realloc(libsvm->param.weight,sizeof(double)*libsvm->param.nr_weight);
  libsvm->param.weight_label[libsvm->param.nr_weight-1] = class_num;
  libsvm->param.weight[libsvm->param.nr_weight-1] = w;
}

void SVM::set_y( double yl, double yu ){
  if( yu > yl ){
    ERR_PRINT("inconsistent lower/upper specification\n");
    exit(1);
  }
  is_y_scaling = true;
  y_lower = yl;
  y_upper = yu;
}

void SVM::crossValidation( DataMatrix *data, int nc, int nr_fold, bool scaling_flg ){
  is_scaling = scaling_flg;
  if( !is_scaling ) is_y_scaling = false; // yもスケーリングしないよ
  if( nr_fold < 2 ){
    ERR_PRINT("n-fold cross validation: n must >= 2\n");
    exit(1);
  }
  if( is_scaling )
    minmax( data, nc );
  setData( data, nc );
  do_cross_validation( nr_fold );
  // 注 nclassには値が入らない
}

void SVM::crossValidation( const char *filename, int nr_fold, bool scaling_flg ){
  is_scaling = scaling_flg;
  if( !is_scaling ) is_y_scaling = false; // yもスケーリングしないよ
  if( nr_fold < 2 ){
    ERR_PRINT("n-fold cross validation: n must >= 2\n");
    exit(1);
  }
  if( is_scaling )
    minmax( filename );
  setData( filename );
  do_cross_validation( nr_fold );
  // 注 nclassには値が入らない
}

void SVM::solve( DataMatrix *data, int nc, bool scaling_flg ){
  is_scaling = scaling_flg;
  if( !is_scaling ) is_y_scaling = false; // yもスケーリングしないよ
  else
    minmax( data, nc );
  setData( data, nc );
  libsvm->model = svm_train( &libsvm->prob, &libsvm->param );
  nclass = nc;
  if( libsvm->param.probability && ( ( libsvm->param.svm_type == C_SVC ) || ( libsvm->param.svm_type == NU_SVC ) ) ){
    if( prob_estimates != NULL ) delete prob_estimates;
    prob_estimates = new double[ nclass ];
  }
}

void SVM::solve( const char *filename, bool scaling_flg ){
  is_scaling = scaling_flg;
  if( !is_scaling ) is_y_scaling = false; // yもスケーリングしないよ
  else
    minmax( filename );
  setData( filename );
  libsvm->model = svm_train( &libsvm->prob, &libsvm->param );
  nclass = libsvm->model->nr_class;
  if( libsvm->param.probability && ( ( libsvm->param.svm_type == C_SVC ) || ( libsvm->param.svm_type == NU_SVC ) ) ){
    if( prob_estimates != NULL ) delete prob_estimates;
    prob_estimates = new double[ nclass ];
  }
}

double SVM::recognize( const ColumnVector &v, bool check_flg ) const {
  double label;
  if( v.length() < dim ){
    ERR_PRINT("Dimension of feature vector is too small. %d < %d\n",v.length(),dim );
    exit(1);
  }
  else if( v.length() > dim )
    ERR_PRINT("Warning: Dimension of feature vector is too large. %d > %d\n",v.length(),dim );

  // 特徴ベクトルをセット
  struct svm_node *x = new struct svm_node[ dim + 1 ];
  int idx = 0;
  for( int i = 0; i < dim; i++ ){
    x[ idx ].index = i;
    x[ idx ].value = v( i );

    if( is_scaling ){
      // 下記の条件を満たさないときはスキップ（idxを更新しない）	
      if( ( feature_max[ x[ idx ].index ] != feature_min[ x[ idx ].index ] ) && ( x[ idx ].value != 0 ) ){
	x[ idx ].value = scaling( x[ idx ].index, x[ idx ].value );
	if( libsvm->param.kernel_type != PRECOMPUTED ) x[ idx ].index++; // indexが１から始まる
	++idx;
      }
    }
    else{
      // 下記の条件を満たさないときはスキップ（idxを更新しない）	
      if( x[ idx ].value != 0 ){
	if( libsvm->param.kernel_type != PRECOMPUTED ) x[ idx ].index++; // indexが１から始まる
	++idx;
      }
    }
  }
  x[ idx++ ].index = -1;

  // probabilityパラメタのチェック
  if( check_flg ){
    if( libsvm->param.probability ){
      if( svm_check_probability_model( libsvm->model ) == 0 ){
	ERR_PRINT("Model does not support probabiliy estimates\n");
	exit(1);
      }
    }
    else{
      if( svm_check_probability_model( libsvm->model ) != 0 )
	ERR_PRINT("Model supports probability estimates, but disabled in prediction.\n");
    }
  }

  // predict
  if( libsvm->param.probability && ( ( libsvm->param.svm_type == C_SVC ) || ( libsvm->param.svm_type == NU_SVC ) ) ){
    label = svm_predict_probability(libsvm->model,x,prob_estimates);
    //     for(j=0;j<nclass;j++)
    //       printf(" %g",prob_estimates[j]);
    //     printf("\n");
  }
  else
    label = svm_predict(libsvm->model,x);
  
  if( is_y_scaling )
    label = y_scaling( label );

  delete x;  
  return label;
}

/*--------------------------------------------------------------------------
 * モデルの読み込み・書き出し
 ---------------------------------------------------------------------------*/
void SVM::read( const char *projname ){
  char filename[ 256 ];
  sprintf( filename, "%s.model", projname );
  libsvm->model = svm_load_model( filename );
  sprintf( filename, "%s.scale", projname );
  load_scale( filename ); // ここで次元dimの値もとってくる

  //svm_destroy_param( &libsvm->param );
  //libsvm->param = libsvm->model->libsvm->param;
  nclass = libsvm->model->nr_class;
  if( libsvm->param.probability && ( ( libsvm->param.svm_type == C_SVC ) || ( libsvm->param.svm_type == NU_SVC ) ) ){
    if( prob_estimates != NULL ) delete prob_estimates;
    prob_estimates = new double[ nclass ];
  }
}

void SVM::write( const char *projname ) const {
  char filename[ 256 ];
  sprintf( filename, "%s.model", projname );
  svm_save_model( filename, libsvm->model );
  sprintf( filename, "%s.scale", projname );
  save_scale( filename );
}

/*--------------------------------------------------------------------------
 * private methods
 ---------------------------------------------------------------------------*/
void SVM::do_cross_validation( int nr_fold ){
  int i;
  int total_correct = 0;
  double total_error = 0;
  double sumv = 0, sumy = 0, sumvv = 0, sumyy = 0, sumvy = 0;
  double *target = new double[ libsvm->prob.l ];
  
  svm_cross_validation(&libsvm->prob,&libsvm->param,nr_fold,target);
  if(libsvm->param.svm_type == EPSILON_SVR ||
     libsvm->param.svm_type == NU_SVR)
    {
      for(i=0;i<libsvm->prob.l;i++)
	{
	  double y = libsvm->prob.y[i];
	  double v = target[i];
	  total_error += (v-y)*(v-y);
	  sumv += v;
	  sumy += y;
	  sumvv += v*v;
	  sumyy += y*y;
	  sumvy += v*y;
	}
      printf("Cross Validation Mean squared error = %g\n",total_error/libsvm->prob.l);
      printf("Cross Validation Squared correlation coefficient = %g\n",
	     ((libsvm->prob.l*sumvy-sumv*sumy)*(libsvm->prob.l*sumvy-sumv*sumy))/
	     ((libsvm->prob.l*sumvv-sumv*sumv)*(libsvm->prob.l*sumyy-sumy*sumy))
	     );
    }
  else
    {
      for(i=0;i<libsvm->prob.l;i++)
	if(target[i] == libsvm->prob.y[i])
	  ++total_correct;
      printf("Cross Validation Accuracy = %g%%\n",100.0*total_correct/libsvm->prob.l);
    }
  delete target;
}

void SVM::setData( DataMatrix *data, int nc ){
  dim = data[0].rows();

  for( int c = 0; c < nc; c++ )
    libsvm->prob.l += data[c].cols();  

  libsvm->prob.y = new double[ libsvm->prob.l ];
  libsvm->prob.x = new struct svm_node*[ libsvm->prob.l ];
  struct svm_node *x_space = new struct svm_node[ ( dim + 1 ) * libsvm->prob.l ];
  
  int l = 0;
  int idx = 0;
  int num;
  double y;
  for( int c = 0; c < nc; c++ ){
    num = data[c].cols();
    if( nc==2 ){ // 2クラスなとき
      if( c==0 ) y = 1;
      else y = -1;
    }
    else // 多クラスなとき
      y = c + 1;

    for( int i = 0; i < num; i++ ){
      if( is_y_scaling )
	libsvm->prob.y[ l ] = y_scaling( y );
      else
	libsvm->prob.y[ l ] = y;

      libsvm->prob.x[ l ] = &x_space[ idx ];

      for( int j = 0; j < dim; j++ ){
	x_space[ idx ].index = j;
	x_space[ idx ].value = data[ c ]( j, i );
	
	if( is_scaling ){
	  // 下記の条件を満たさないときはスキップ（idxを更新しない）	
	  if( ( feature_max[ x_space[ idx ].index ] != feature_min[ x_space[ idx ].index ] ) && ( x_space[ idx ].value != 0 ) ){
	    x_space[ idx ].value = scaling( x_space[ idx ].index, x_space[ idx ].value );
	    if( libsvm->param.kernel_type != PRECOMPUTED ) x_space[ idx ].index++; // indexが１から始まる
	    ++idx;
	  }
	}
	else{
	  // 下記の条件を満たさないときはスキップ（idxを更新しない）	
	  if(  x_space[ idx ].value != 0 ){
	    if( libsvm->param.kernel_type != PRECOMPUTED ) x_space[ idx ].index++; // indexが１から始まる
	    ++idx;
	  }
	}
      }
      l++;
      x_space[ idx++ ].index = -1;
    }
  }

  if(libsvm->param.gamma == 0 && dim > 0)
    libsvm->param.gamma = 1.0 / dim;
}

void SVM::setData( const char *filename ){
  struct svm_node *x_space;
  static char *line = NULL;
  static int max_line_len;

  int elements, inst_max_index, i, j;
  FILE *fp = fopen(filename,"r");
  if(fp==NULL){
    ERR_PRINT("can't open file %s\n", filename);
    exit(1);
  }  
  char *endptr;
  char *idx, *val, *label;
  
  if(fp == NULL)
    {
      ERR_PRINT("can't open input file %s\n",filename);
      exit(1);
    }
  
  libsvm->prob.l = 0;
  elements = 0;
  
  max_line_len = 1024;
  line = new char[ max_line_len ];
  while(readline(fp,line)!=NULL)
    {
      char *p = strtok(line," \t"); // label
      
      // features
      while(1)
	{
	  p = strtok(NULL," \t");
	  if(p == NULL || *p == '\n') // check '\n' as ' ' may be after the last feature
	    break;
	  ++elements;
	}
      ++elements;
      ++libsvm->prob.l;
    }
  rewind(fp);
  
  libsvm->prob.y = new double[ libsvm->prob.l ];
  libsvm->prob.x = new struct svm_node *[ libsvm->prob.l ];
  x_space = new struct svm_node[ elements ];

  int shift1 = 1;
  if( libsvm->param.kernel_type == PRECOMPUTED ) shift1 = 0;
  
  dim = 0;
  j=0;
  for(i=0;i<libsvm->prob.l;i++)
    {
      inst_max_index = -1; // strtol gives 0 if wrong format, and precomputed kernel has <index> start from 0
      readline(fp,line);
      libsvm->prob.x[i] = &x_space[j];
      label = strtok(line," \t");

      libsvm->prob.y[i] = 1;
      //libsvm->prob.y[i] = strtod(label,&endptr);
      if( is_y_scaling )
	libsvm->prob.y[i] = y_scaling( strtod(label,&endptr) );
      else
	libsvm->prob.y[i] = strtod(label,&endptr);

      if(endptr == label){
	ERR_PRINT( "Wrong input format at line %d\n", i+1 );
	exit(1);
      }
      
      while(1)
	{
	  idx = strtok(NULL,":");
	  val = strtok(NULL," \t");
	  
	  if(val == NULL)
	    break;
	  
	  errno = 0;
	  x_space[j].index = (int) strtol(idx,&endptr,10);
	  if(endptr == idx || errno != 0 || *endptr != '\0' || x_space[j].index <= inst_max_index){
	    ERR_PRINT( "Wrong input format at line %d\n", i+1 );
	    exit(1);
	  }
	  else
	    inst_max_index = x_space[j].index;
	  
	  errno = 0;
	  x_space[j].value = strtod(val,&endptr);
	  if(endptr == val || errno != 0 || (*endptr != '\0' && !isspace(*endptr))){
	    ERR_PRINT( "Wrong input format at line %d\n", i+1 );
	    exit(1);
	  }

	  if( is_scaling ){
	    // 下記の条件を満たさないときはスキップ（jを更新しない）	
	    if( ( feature_max[ x_space[j].index - shift1 ] != feature_min[ x_space[j].index - shift1 ] ) && ( x_space[j].value != 0 ) ){
	      x_space[j].value = scaling( x_space[j].index - shift1, x_space[j].value );
	      ++j;
	    }
	  }
	  else{
	    // 下記の条件を満たさないときはスキップ（jを更新しない）	
	    if( x_space[j].value != 0 ){
	      ++j;
	    }
	  }
	}
      
      if(inst_max_index > dim)
	dim = inst_max_index;
      x_space[j++].index = -1;
    }
  
  if( libsvm->param.kernel_type == PRECOMPUTED ) dim++; 
 
  if(libsvm->param.gamma == 0 && dim > 0)
    libsvm->param.gamma = 1.0 / dim;
  
  if(libsvm->param.kernel_type == PRECOMPUTED)
    for(i=0;i<libsvm->prob.l;i++)
      {
	if (libsvm->prob.x[i][0].index != 0)
	  {
	    ERR_PRINT("Wrong input format: first column must be 0:sample_serial_number\n");
	    exit(1);
	  }
	if ((int)libsvm->prob.x[i][0].value <= 0 || (int)libsvm->prob.x[i][0].value >= dim)
	  {
	    ERR_PRINT("Wrong input format: sample_serial_number out of range\n");
	    exit(1);
	  }
      }  
  fclose(fp);

}

const char* SVM::readline(FILE *input,char *line){
  int len;
  int max_line_len = 1024;
  
  if(fgets(line,max_line_len,input) == NULL)
    return NULL;
  
  while(strrchr(line,'\n') == NULL)
    {
      max_line_len *= 2;
      line = (char *) realloc(line,max_line_len);
      len = (int) strlen(line);
      if(fgets(line+len,max_line_len-len,input) == NULL)
	break;
    }
  return line;
}

double SVM::y_scaling( double value ) const {
  double s_value;
  if(value == y_min)     s_value = y_lower;
  else if(value == y_max)     s_value = y_upper;
  else     s_value = y_lower + (y_upper-y_lower) *
	     (value - y_min)/(y_max-y_min);
  return s_value;
}

double SVM::scaling( int index, double value ) const {
  double s_value;
  if(value == feature_min[index])     s_value = lower;
  else if(value == feature_max[index])     s_value = upper;
  else     s_value = lower + (upper-lower) * 
	     (value-feature_min[index])/
	     (feature_max[index]-feature_min[index]);
  return s_value;
}

void SVM::minmax( DataMatrix *data, int nc ){
  // まず y
  if( nc == 2 ){
    y_min = -1;
    y_max = 1;
  }
  else{
    y_min = 1;
    y_max = nc;
  }

  dim = data[0].rows();
  feature_max = new double[ dim ];
  feature_min = new double[ dim ];
  for( int i = 0; i < dim; i++ ){
    feature_max[i]=-DBL_MAX;
    feature_min[i]=DBL_MAX;
  }

  for( int c = 0; c < nc; c++ ){
    int num = data[c].cols();
    for( int i = 0; i < num; i++ ){
      for( int j = 0; j < dim; j++ ){
	feature_max[ j ] = max( feature_max[ j ], data[ c ]( j, i ) );
	feature_min[ j ] = min( feature_min[ j ], data[ c ]( j, i ) );
      }
    }
  }
}

void SVM::minmax( const char *filename ){
  static int max_line_len = 1024;
  static char *line = new char[ max_line_len ];
  int index;

  // まずは次元を数えるだけ
  FILE *fp = fopen( filename, "r" );
  if(fp==NULL){
    ERR_PRINT("can't open file %s\n", filename);
    exit(1);
  }  
 
  while(readline(fp,line)!=NULL){
    char *p=line;
    
    SKIP_TARGET
    
    while(sscanf(p,"%d:%*f",&index)==1){
      dim = max(dim, index);
      SKIP_ELEMENT
	//num_nonzeros++;
    }		
  }
  rewind(fp);
  // ここまで

  int shift1 = 1;
  if( libsvm->param.kernel_type == PRECOMPUTED ){
    shift1 = 0;
    dim++;
  }

  feature_max = new double[ dim ];
  feature_min = new double[ dim ];
  for( int i = 0; i < dim; i++ ){
    feature_max[i]=-DBL_MAX;
    feature_min[i]=DBL_MAX;
  }

  // ここから読み込み  
  while(readline(fp,line)!=NULL){
    char *p=line;
    int next_index=1;
    double target;
    double value;
    
    sscanf(p,"%lf",&target);
    y_max = max(y_max,target);
    y_min = min(y_min,target);
    
    SKIP_TARGET
      
      while(sscanf(p,"%d:%lf",&index,&value)==2){
	for( int i = next_index; i < index; i++ ){
	  feature_max[ i - shift1 ]=max(feature_max[ i - shift1 ],0);
	  feature_min[ i - shift1 ]=min(feature_min[ i - shift1 ],0);
	}
	  
	feature_max[ index - shift1 ]=max(feature_max[ index - shift1 ],value);
	feature_min[ index - shift1 ]=min(feature_min[ index - shift1 ],value);
	
	SKIP_ELEMENT
	  next_index=index+1;
      }		
    
    for( int i = next_index; i < dim; i++ ){
      feature_max[ i - shift1 ]=max(feature_max[ i - shift1 ],0);
      feature_min[ i - shift1 ]=min(feature_min[ i - shift1 ],0);
    }	
  }  
  fclose(fp);  

}

void SVM::save_scale( const char *filename ) const {
  FILE *fp_save = fopen(filename,"w");
  if(fp_save==NULL){
    ERR_PRINT("can't open file %s\n", filename);
    exit(1);
  }
  if(is_scaling){
    if(is_y_scaling){
      fprintf(fp_save, "y\n");
      fprintf(fp_save, "%.16g %.16g\n", y_lower, y_upper);
      fprintf(fp_save, "%.16g %.16g\n", y_min, y_max);
    }
    fprintf(fp_save, "x\n");
    fprintf(fp_save, "%.16g %.16g\n", lower, upper);
    for( int i = 0; i < dim; i++ ){
      if(feature_min[i]!=feature_max[i])
	fprintf(fp_save,"%d %.16g %.16g\n",i,feature_min[i],feature_max[i]);
    }
  }
  else
    fprintf(fp_save, "dim: %d\n", dim);
  fclose(fp_save);
}

void SVM::load_scale( const char *filename ){
  int idx, c;
  double fmin, fmax;
  static int max_line_len = 1024;
  static char *line = new char[ max_line_len ];

  // まずは次元を数えるだけ
  FILE *fp_restore = fopen(filename,"r");
  if(fp_restore==NULL){
    ERR_PRINT("can't open file %s\n", filename);
    exit(1);
  }  
  c = fgetc(fp_restore);
  if( c == 'd' ){
    fscanf(fp_restore,"%s %d\n",line,&dim);
    is_scaling = false;
    is_y_scaling = false;
    return;
  }

  if(c == 'y'){
    readline(fp_restore,line);
    readline(fp_restore,line);
    readline(fp_restore,line);
  }
  readline(fp_restore,line);
  readline(fp_restore,line);  
  while(fscanf(fp_restore,"%d %*f %*f\n",&idx) == 1)
    dim = max(idx,dim);
  rewind(fp_restore);
  // ここまで
  dim++;

  feature_max = new double[ dim ];
  feature_min = new double[ dim ];

  // ここから読み込み  
  if((c = fgetc(fp_restore)) == 'y'){
    fscanf(fp_restore, "%lf %lf\n", &y_lower, &y_upper);
    fscanf(fp_restore, "%lf %lf\n", &y_min, &y_max);
    is_y_scaling = true;
  }
  else
    ungetc(c, fp_restore);
  
  if (fgetc(fp_restore) == 'x') {
    is_scaling = true;
    fscanf(fp_restore, "%lf %lf\n", &lower, &upper);
    while(fscanf(fp_restore,"%d %lf %lf\n",&idx,&fmin,&fmax)==3){
      if( idx<dim ){ // 必要ないんだけど、ダブるチェック？？
	feature_min[idx] = fmin;
	feature_max[idx] = fmax;
      }
    }
  }
  fclose(fp_restore);
  // ここまで
}

void SVM::print_help( char **err_message ){
  strcpy( *err_message, "options:\n" );
  strcat( *err_message, "-s svm_type : set type of SVM (default 0)\n" );
  strcat( *err_message, "	0 -- C-SVC\n" );
  strcat( *err_message, "	1 -- nu-SVC\n" );
  strcat( *err_message, "	2 -- one-class SVM\n" );
  strcat( *err_message, "	3 -- epsilon-SVR\n" );
  strcat( *err_message, "	4 -- nu-SVR\n" );
  strcat( *err_message, "-t kernel_type : set type of kernel function (default 2)\n" );
  strcat( *err_message, "	0 -- linear: u'*v\n" );
  strcat( *err_message, "	1 -- polynomial: (gamma*u'*v + coef0)^degree\n" );
  strcat( *err_message, "	2 -- radial basis function: exp(-gamma*|u-v|^2)\n" );
  strcat( *err_message, "	3 -- sigmoid: tanh(gamma*u'*v + coef0)\n" );
  strcat( *err_message, "	4 -- precomputed kernel (kernel values in training_set_file)\n" );
  strcat( *err_message, "-d degree : set degree in kernel function (default 3)\n" );
  strcat( *err_message, "-g gamma : set gamma in kernel function (default 1/num_features)\n" );
  strcat( *err_message, "-r coef0 : set coef0 in kernel function (default 0)\n" );
  strcat( *err_message, "-c cost : set the parameter C of C-SVC, epsilon-SVR, and nu-SVR (default 1)\n" );
  strcat( *err_message, "-n nu : set the parameter nu of nu-SVC, one-class SVM, and nu-SVR (default 0.5)\n" );
  strcat( *err_message, "-p epsilon : set the epsilon in loss function of epsilon-SVR (default 0.1)\n" );
  strcat( *err_message, "-m cachesize : set cache memory size in MB (default 100)\n" );
  strcat( *err_message, "-e epsilon : set tolerance of termination criterion (default 0.001)\n" );
  strcat( *err_message, "-h shrinking : whether to use the shrinking heuristics, 0 or 1 (default 1)\n" );
  strcat( *err_message, "-b probability_estimates : whether to train a SVC or SVR model for probability estimates, 0 or 1 (default 0)\n" );
  strcat( *err_message, "-l lower : x scaling lower limit (default -1)\n" );
  strcat( *err_message, "-u upper : x scaling upper limit (default +1)\n" );
  strcat( *err_message, "to set weight, use set_weight()\n" );
  strcat( *err_message, "to set y_lower and y_upper, use set_y()\n" );
}
