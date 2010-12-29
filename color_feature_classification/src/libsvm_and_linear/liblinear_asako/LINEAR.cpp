#include "../coolVisi/stat/LINEAR.hpp"
#include "libLINEAR.hpp"
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

void print_null(const char *s) {}
void print_string_stdout(const char *s) {
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

// libLINEAR隠蔽
struct LINEAR::_liblinear_impl {
  struct parameter param;
  struct problem prob;
  struct model *model;
};

/*--------------------------------------------------------------------------
 * コンストラクタ・デストラクタ
 ---------------------------------------------------------------------------*/
LINEAR::LINEAR( bool print_flg ) : liblinear(new _liblinear_impl) {
  liblinear->param.solver_type = L2R_L2LOSS_SVC_DUAL;
  liblinear->param.C = 1;
  //liblinear->param.eps = INF; // see setting below
  liblinear->param.eps = 0.1;
  liblinear->param.nr_weight = 0;
  liblinear->param.weight_label = NULL;
  liblinear->param.weight = NULL;
  liblinear->prob.bias = -1;
  probability = 0;
  liblinear->prob.l = 0;
  liblinear->prob.y = NULL;
  liblinear->prob.x = NULL;
  liblinear->model = NULL;
  prob_estimates = NULL;
  eps_default_flg = true;
  lower = -1.0;
  upper = 1.0;
  is_scaling = true;
  is_y_scaling = false;
  y_max = -DBL_MAX;
  y_min = DBL_MAX;
  feature_min = NULL;
  feature_max = NULL;
  liblinear->prob.n = 0;
  if( print_flg )
    liblinear_print_string = &print_string_stdout;
  else
    liblinear_print_string = &print_null;
}

LINEAR::~LINEAR(){
  destroy_param( &liblinear->param );
  if( liblinear->model != NULL )
    destroy_model( liblinear->model );
  if( liblinear->prob.y != NULL ) delete liblinear->prob.y;
  if( liblinear->prob.x != NULL ) delete liblinear->prob.x;
  if( prob_estimates != NULL ) delete prob_estimates;
  if( feature_min != NULL ) delete feature_min;
  if( feature_max != NULL ) delete feature_max;
  delete liblinear;
}

/*--------------------------------------------------------------------------
 * public methods
 ---------------------------------------------------------------------------*/
void LINEAR::setParam( char c, int val ){
  setParam( c, (double)val );
}

void LINEAR::setParam( char c, double val ){
  switch( c )
    {
    case 's':
      liblinear->param.solver_type = (int)val;
      if( eps_default_flg ){
	switch( liblinear->param.solver_type ){
	case L2R_LR:
	case L2R_L2LOSS_SVC:
	  liblinear->param.eps = 0.01;
	  break;
	case L2R_L2LOSS_SVC_DUAL:
	case L2R_L1LOSS_SVC_DUAL:
	case MCSVM_CS:
	  liblinear->param.eps = 0.1;
	  break;
	case L1R_L2LOSS_SVC:
	case L1R_LR:
	  liblinear->param.eps = 0.01;
	  break;
	default:
	  break;
	}
      }
      break;
    case 'c':
      liblinear->param.C = val;
      break;
    case 'e':
      liblinear->param.eps = val;
      eps_default_flg = false;
      break;
    case 'B':
      liblinear->prob.bias = val;
      break;
    case 'b':
      probability = (int)val;
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
    }  
}

void LINEAR::set_solver_type( int val ){
  liblinear->param.solver_type = val;
  if( eps_default_flg ){
    switch( liblinear->param.solver_type ){
    case L2R_LR:
    case L2R_L2LOSS_SVC:
      liblinear->param.eps = 0.01;
      break;
    case L2R_L2LOSS_SVC_DUAL:
    case L2R_L1LOSS_SVC_DUAL:
    case MCSVM_CS:
      liblinear->param.eps = 0.1;
      break;
    case L1R_L2LOSS_SVC:
    case L1R_LR:
      liblinear->param.eps = 0.01;
      break;
    default:
      break;
    }
  }
}

void LINEAR::set_cost( double val )    { liblinear->param.C = val; }
void LINEAR::set_epsilon( double val ) { liblinear->param.eps = val; eps_default_flg = false;  }
void LINEAR::set_bias( double val )    { liblinear->prob.bias = val; }
void LINEAR::set_probability( int val ){ probability = val; }
void LINEAR::set_lower( double val )   { lower = val; }
void LINEAR::set_upper( double val )   { upper = val; }

void LINEAR::set_weight( int class_num, double w ){
  ++liblinear->param.nr_weight;
  liblinear->param.weight_label = (int *)realloc(liblinear->param.weight_label,sizeof(int)*liblinear->param.nr_weight);
  liblinear->param.weight = (double *)realloc(liblinear->param.weight,sizeof(double)*liblinear->param.nr_weight);
  liblinear->param.weight_label[liblinear->param.nr_weight-1] = class_num;
  liblinear->param.weight[liblinear->param.nr_weight-1] = w;
}

void LINEAR::set_y( double yl, double yu ){
  if( yu > yl ){
    ERR_PRINT("inconsistent lower/upper specification\n");
    exit(1);
  }
  is_y_scaling = true;
  y_lower = yl;
  y_upper = yu;
}

void LINEAR::crossValidation( DataMatrix *data, int nc, int nr_fold, bool scaling_flg ){
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

void LINEAR::crossValidation( const char *filename, int nr_fold, bool scaling_flg ){
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

void LINEAR::solve( DataMatrix *data, int nc, bool scaling_flg ){
  is_scaling = scaling_flg;
  if( !is_scaling ) is_y_scaling = false; // yもスケーリングしないよ
  else
    minmax( data, nc );
  setData( data, nc );
  liblinear->model = train( &liblinear->prob, &liblinear->param );
  nclass = nc;
  if( probability ){
    if( prob_estimates != NULL ) delete prob_estimates;
    prob_estimates = new double[ nclass ];
  }
}

void LINEAR::solve( const char *filename, bool scaling_flg ){
  is_scaling = scaling_flg;
  if( !is_scaling ) is_y_scaling = false; // yもスケーリングしないよ
  else
    minmax( filename );
  setData( filename );
  liblinear->model = train( &liblinear->prob, &liblinear->param );
  nclass = liblinear->model->nr_class;
  if( probability ){
    if( prob_estimates != NULL ) delete prob_estimates;
    prob_estimates = new double[ nclass ];
  }
}

int LINEAR::recognize( const ColumnVector &v ) const {
  int label;
  if( v.length() < liblinear->prob.n ){
    ERR_PRINT("Dimension of feature vector is too small. %d < %d\n",v.length(),liblinear->prob.n );
    exit(1);
  }
  else if( v.length() > liblinear->prob.n )
    ERR_PRINT("Warning: Dimension of feature vector is too large. %d > %d\n",v.length(),liblinear->prob.n );

  // 特徴ベクトルをセット
  struct feature_node *x = new struct feature_node[ liblinear->prob.n + 2 ];
  int idx = 0;
  for( int i = 0; i < liblinear->prob.n; i++ ){
    x[ idx ].index = i;
    x[ idx ].value = v( i );

    if( is_scaling ){
      // 下記の条件を満たさないときはスキップ（idxを更新しない）	
      if( ( feature_max[ x[ idx ].index ] != feature_min[ x[ idx ].index ] ) && ( x[ idx ].value != 0 ) ){
	x[ idx ].value = scaling( x[ idx ].index, x[ idx ].value );
	if( liblinear->model->bias < 0 ) x[ idx ].index++; // indexが１から始まる
	++idx;
      }
    }
    else{
      // 下記の条件を満たさないときはスキップ（idxを更新しない）	
      if( x[ idx ].value != 0 ){
	if( liblinear->model->bias < 0 ) x[ idx ].index++; // indexが１から始まる
	++idx;
      }
    }
  }

  if(liblinear->model->bias>=0){
    x[ idx ].index = liblinear->prob.n;
    x[ idx ].value = liblinear->model->bias;
    idx++;
  }
  x[ idx ].index = -1;

  // predict
  if( probability ){
    if( liblinear->model->param.solver_type != L2R_LR ){
      ERR_PRINT( "probability output is only supported for logistic regression\n" );
      exit(1);
    }
    label = predict_probability(liblinear->model,x,prob_estimates);
    //     for(j=0;j<nclass;j++)
    //       printf(" %g",prob_estimates[j]);
    //     printf("\n");
  }
  else
    label = predict(liblinear->model,x);

  if( is_y_scaling )
    label = y_scaling( label );

  delete x;  
  return label;
}

/*--------------------------------------------------------------------------
 * モデルの読み込み・書き出し
 ---------------------------------------------------------------------------*/
void LINEAR::read( const char *projname ){
  char filename[ 256 ];
  sprintf( filename, "%s.model", projname );
  liblinear->model = load_model( filename );
  sprintf( filename, "%s.scale", projname );
  load_scale( filename ); // ここで次元dimの値もとってくる

  //destroy_param( &liblinear->param );
  //liblinear->param = liblinear->model->liblinear->param;
  nclass = liblinear->model->nr_class;
  if( probability ){
    if( prob_estimates != NULL ) delete prob_estimates;
    prob_estimates = new double[ nclass ];
  }
}

void LINEAR::write( const char *projname ) const {
  char filename[ 256 ];
  sprintf( filename, "%s.model", projname );
  save_model( filename, liblinear->model );
  sprintf( filename, "%s.scale", projname );
  save_scale( filename );
}

/*--------------------------------------------------------------------------
 * private methods
 ---------------------------------------------------------------------------*/
void LINEAR::do_cross_validation( int nr_fold ){
  int i;
  int total_correct = 0;
  int *target = new int[ liblinear->prob.l ];
  
  cross_validation(&liblinear->prob,&liblinear->param,nr_fold,target);

  for(i=0;i<liblinear->prob.l;i++)
    if(target[i] == liblinear->prob.y[i])
      ++total_correct;
  printf("Cross Validation Accuracy = %g%%\n",100.0*total_correct/liblinear->prob.l);
  delete target;
}

void LINEAR::setData( DataMatrix *data, int nc ){
  liblinear->prob.n = data[0].rows();

  for( int c = 0; c < nc; c++ )
    liblinear->prob.l += data[c].cols();  

  liblinear->prob.y = new int[ liblinear->prob.l ];
  liblinear->prob.x = new struct feature_node*[ liblinear->prob.l ];
  struct feature_node *x_space = new struct feature_node[ ( liblinear->prob.n + 2 ) * liblinear->prob.l ];
  
  int l = 0;
  int idx = 0;
  int num;
  int y;
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
	liblinear->prob.y[ l ] = (int)y_scaling( (double)y );
      else
	liblinear->prob.y[ l ] = y;

      liblinear->prob.x[ l ] = &x_space[ idx ];

      for( int j = 0; j < liblinear->prob.n; j++ ){
	x_space[ idx ].index = j;
	x_space[ idx ].value = data[ c ]( j, i );

	if( is_scaling ){	
	  // 下記の条件を満たさないときはスキップ（idxを更新しない）	
	  if( ( feature_max[ x_space[ idx ].index ] != feature_min[ x_space[ idx ].index ] ) && ( x_space[ idx ].value != 0 ) ){
	    x_space[ idx ].value = scaling( x_space[ idx ].index, x_space[ idx ].value );
	    if( liblinear->prob.bias < 0 ) x_space[ idx ].index++; // indexが１から始まる
	    ++idx;
	  }
	}
	else{
	  // 下記の条件を満たさないときはスキップ（idxを更新しない）	
	  if( x_space[ idx ].value != 0 ){
	    if( liblinear->prob.bias < 0 ) x_space[ idx ].index++; // indexが１から始まる
	    ++idx;
	  }
	}
      }
      l++;

      if(liblinear->prob.bias >= 0)
	x_space[ idx++ ].value = liblinear->prob.bias;
      x_space[ idx++ ].index = -1;
    }
  }
  if(liblinear->prob.bias >= 0)
    {
      for(int i=1;i<liblinear->prob.l;i++)
	(liblinear->prob.x[i]-2)->index = liblinear->prob.n; 
      x_space[idx-2].index = liblinear->prob.n;
    }
}

void LINEAR::setData( const char *filename ){
  struct feature_node *x_space;
  static char *line = NULL;
  static int max_line_len;

  int elements, max_index, inst_max_index, i, j;
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
  
  liblinear->prob.l = 0;
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
      ++liblinear->prob.l;
    }
  rewind(fp);
  
  liblinear->prob.y = new int[ liblinear->prob.l ];
  liblinear->prob.x = new struct feature_node *[ liblinear->prob.l ];
  x_space = new struct feature_node[ elements + liblinear->prob.l ];
  
  int shift1 = 1;
  if( liblinear->prob.bias >= 0 ) shift1 = 0;

  max_index = 0;
  j=0;
  for(i=0;i<liblinear->prob.l;i++)
    {
      inst_max_index = 0; // strtol gives 0 if wrong format
      readline(fp,line);
      liblinear->prob.x[i] = &x_space[j];
      label = strtok(line," \t");

      if( is_y_scaling )
	liblinear->prob.y[i] = (int)y_scaling( (double)strtol(label,&endptr,10) );
      else
	liblinear->prob.y[i] = (int)strtol(label,&endptr,10);

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
      
      if(inst_max_index > max_index)
	max_index = inst_max_index;
      if(liblinear->prob.bias >= 0)
	x_space[j++].value = liblinear->prob.bias;

      x_space[j++].index = -1;
    }
  
  if(liblinear->prob.bias >= 0)
    {
      liblinear->prob.n=max_index+1;
      for(i=1;i<liblinear->prob.l;i++)
	(liblinear->prob.x[i]-2)->index = liblinear->prob.n; 
      x_space[j-2].index = liblinear->prob.n;
    }
  else
    liblinear->prob.n=max_index;

  fclose(fp);

}

const char* LINEAR::readline(FILE *input,char *line){
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

double LINEAR::y_scaling( double value ) const {
  double s_value;
  if(value == y_min)     s_value = y_lower;
  else if(value == y_max)     s_value = y_upper;
  else     s_value = y_lower + (y_upper-y_lower) *
	     (value - y_min)/(y_max-y_min);
  return s_value;
}

double LINEAR::scaling( int index, double value ) const {
  double s_value;
  if(value == feature_min[index])     s_value = lower;
  else if(value == feature_max[index])     s_value = upper;
  else     s_value = lower + (upper-lower) * 
	     (value-feature_min[index])/
	     (feature_max[index]-feature_min[index]);
  return s_value;
}

void LINEAR::minmax( DataMatrix *data, int nc ){
  // まず y
  if( nc == 2 ){
    y_min = -1;
    y_max = 1;
  }
  else{
    y_min = 1;
    y_max = nc;
  }

  liblinear->prob.n = data[0].rows();
  feature_max = new double[ liblinear->prob.n ];
  feature_min = new double[ liblinear->prob.n ];
  for( int i = 0; i < liblinear->prob.n; i++ ){
    feature_max[i]=-DBL_MAX;
    feature_min[i]=DBL_MAX;
  }

  for( int c = 0; c < nc; c++ ){
    int num = data[c].cols();
    for( int i = 0; i < num; i++ ){
      for( int j = 0; j < liblinear->prob.n; j++ ){
	feature_max[ j ] = max( feature_max[ j ], data[ c ]( j, i ) );
	feature_min[ j ] = min( feature_min[ j ], data[ c ]( j, i ) );
      }
    }
  }
}

void LINEAR::minmax( const char *filename ){
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
      liblinear->prob.n = max(liblinear->prob.n, index);
      SKIP_ELEMENT
	//num_nonzeros++;
    }		
  }
  rewind(fp);
  // ここまで

  int shift1 = 1;
  if( liblinear->prob.bias >= 0 ){
    shift1 = 0;
    liblinear->prob.n++;
  }

  feature_max = new double[ liblinear->prob.n ];
  feature_min = new double[ liblinear->prob.n ];
  for( int i = 0; i < liblinear->prob.n; i++ ){
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
    
    for( int i = next_index; i < liblinear->prob.n; i++ ){
      feature_max[ i - shift1 ]=max(feature_max[ i - shift1 ],0);
      feature_min[ i - shift1 ]=min(feature_min[ i - shift1 ],0);
    }	
  }  
  fclose(fp);  

}

void LINEAR::save_scale( const char *filename ) const {
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
    for( int i = 0; i < liblinear->prob.n; i++ ){
      if(feature_min[i]!=feature_max[i])
	fprintf(fp_save,"%d %.16g %.16g\n",i,feature_min[i],feature_max[i]);
    }
  }
  else
    fprintf(fp_save, "dim: %d\n", liblinear->prob.n);
  fclose(fp_save);
}

void LINEAR::load_scale( const char *filename ){
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
    fscanf(fp_restore,"%s %d\n",line,&(liblinear->prob.n));
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
    liblinear->prob.n = max(idx,liblinear->prob.n);
  rewind(fp_restore);
  // ここまで
  liblinear->prob.n++;

  feature_max = new double[ liblinear->prob.n ];
  feature_min = new double[ liblinear->prob.n ];

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
      if( idx<liblinear->prob.n ){ // 必要ないんだけど、ダブるチェック？？
	feature_min[idx] = fmin;
	feature_max[idx] = fmax;
      }
    }
  }
  fclose(fp_restore);
  // ここまで
}

void LINEAR::print_help( char **err_message ){
  strcpy( *err_message, "options:\n" );
  strcat( *err_message, "-s type : set type of solver (default 1)\n" );
  strcat( *err_message, "	0 -- L2-regularized logistic regression\n" );
  strcat( *err_message, "	1 -- L2-regularized L2-loss support vector classification (dual)\n" );
  strcat( *err_message, "	2 -- L2-regularized L2-loss support vector classification (primal)\n" );
  strcat( *err_message, "	3 -- L2-regularized L1-loss support vector classification (dual)\n" );
  strcat( *err_message, "	4 -- multi-class support vector classification by Crammer and Singer\n" );
  strcat( *err_message, "	5 -- L1-regularized L2-loss support vector classification\n" );
  strcat( *err_message, "	6 -- L1-regularized logistic regression\n" );
  strcat( *err_message, "-c cost : set the parameter C (default 1)\n" );
  strcat( *err_message, "-e epsilon : set tolerance of termination criterion\n" );
  strcat( *err_message, "	-s 0 and 2\n" );
  strcat( *err_message, "		|f'(w)|_2 <= eps*min(pos,neg)/l*|f'(w0)|_2,\n" );
  strcat( *err_message, "		where f is the primal function and pos/neg are # of\n" );
  strcat( *err_message, "		positive/negative data (default 0.01)\n" );
  strcat( *err_message, "	-s 1, 3, and 4\n" );
  strcat( *err_message, "		Dual maximal violation <= eps; similar to libsvm (default 0.1)\n" );
  strcat( *err_message, "	-s 5 and 6\n" );
  strcat( *err_message, "		|f'(w)|_inf <= eps*min(pos,neg)/l*|f'(w0)|_inf,\n" );
  strcat( *err_message, "		where f is the primal function (default 0.01)\n" );
  strcat( *err_message, "-B bias : if bias >= 0, instance x becomes [x; bias]; if < 0, no bias term added (default -1)\n" );
  strcat( *err_message, "-b probability_estimates : whether to train a SVC or SVR model for probability estimates, 0 or 1 (default 0)\n" );
  strcat( *err_message, "-l lower : x scaling lower limit (default -1)\n" );
  strcat( *err_message, "-u upper : x scaling upper limit (default +1)\n" );
  strcat( *err_message, "to set weight, use set_weight()\n" );
  strcat( *err_message, "to set y_lower and y_upper, use set_y()\n" );
}
