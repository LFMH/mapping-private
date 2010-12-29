#include <iostream>

#include <coolVisi/util/DataMatrix.hpp>
#include <coolVisi/stat/SVM.hpp>

using namespace std;
using namespace coolVisi;

//#define PROBABILITY

void print_result( const SVM &svm, DataMatrix *data );
void print_result_predict( const SVM &svm, DataMatrix *data );

const char* readline(FILE *input,char *line){
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

int main(int argc, char **argv){
  const int nclass = 2;
  const int dim    = 3;
  const int ns     = 1000;
  DataMatrix data[ nclass ];
  for( int i = 0; i < nclass; ++i )
    data[ i ].resize( dim, ns );
  const double center[ nclass ][ dim ] = 
    { { 1.0, 1.0, 1.0 }, { -1.0, -1.0, -1.0 } };

  // create data
  srand( time( NULL ) );
  double rmax = (double)RAND_MAX;
  for( int i = 0; i < nclass; ++i )
    for( int j = 0; j < ns; ++j )
      for( int k = 0; k < dim; ++k )
	data[ i ]( k, j ) = 
	  ( k + 1 ) * (double)rand() / rmax + center[ i ][ k ];

  //****** 1. test with the above data *********//  

  // SVM cross validation
  cout << "-- cross validation ------------------------" << endl;
  SVM svm( false );
  svm.crossValidation( data, nclass );

  // SVM training
  cout << "-- analyzing test --------------------------" << endl;
  SVM svm2;
#ifdef PROBABILITY
  svm2.set_probability( 1 );
#endif
  svm2.solve( data, nclass );
  print_result( svm2, data );

  // write and read the results of SVM
  cout << "-- read and write test ---------------------" << endl;
  SVM svm3;
  svm2.write( "data/SVM-test" );
#ifdef PROBABILITY
  svm3.set_probability( 1 );
#endif
  svm3.read( "data/SVM-test" );
#ifdef PROBABILITY
  print_result_predict( svm3, data );
#else
  print_result( svm3, data );
#endif

  //****** 2. test with the data from a file *************//  

  int nclass2 = 2;
  int dim2    = 13;
  DataMatrix data2[ nclass2 ];
  ColumnVector vec( dim2 );
  char line[1100];
  char *endptr;
  char *idx, *val, *label;
  int index;
  int y;
  FILE *fp = fopen( "data/heart_scale", "r" );
  for(int i=0;i<270;i++){
    readline(fp,line);
    label = strtok(line," \t");
    for( int j=0; j<dim2; j++)
      vec( j ) = 0;
    while(1){
      idx = strtok(NULL,":");
      val = strtok(NULL," \t");      
      if(val == NULL)
	break;
      index = (int) strtol(idx,&endptr,10) - 1;
      vec( index ) = strtod(val,&endptr);      
    }
    y = (int)strtod(label,&endptr);
    if( y > 0 )
      data2[ 0 ].addData( vec );
    else
      data2[ 1 ].addData( vec );    
  }
  fclose( fp );

  // SVM cross validation
  cout << "-- cross validation ( from file ) ----------" << endl;
  SVM svm4( false );
  svm4.crossValidation( "data/heart_scale", 10, false );

  // SVM training
  cout << "-- analyzing test ( from file ) ------------" << endl;
  SVM svm5;
#ifdef PROBABILITY
  svm5.set_probability( 1 );
#endif
  svm5.solve( "data/heart_scale", false );
  print_result( svm5, data2 );

  // write and read the results of SVM
  cout << "-- read and write test ---------------------" << endl;
  SVM svm6;
  svm5.write( "data/SVM-test" );
#ifdef PROBABILITY
  svm6.set_probability( 1 );
#endif
  svm6.read( "data/SVM-test" );
#ifdef PROBABILITY
  print_result_predict( svm6, data2 );
#else
  print_result( svm6, data2 );
#endif

  return 0;
}


void print_result( const SVM &svm, DataMatrix *data ){
  int total_ns = data[ 0 ].nSamples();
  for( int i = 1; i < svm.nClass(); ++i )
    total_ns += data[ i ].nSamples();

  int correct = 0;
  for( int j = 0; j < data[ 0 ].nSamples(); ++j )
    if( svm.recognize( data[ 0 ]( j ) ) > 0 ) // because there are only 2 classes
      ++correct;
  for( int j = 0; j < data[ 1 ].nSamples(); ++j )
    if( svm.recognize( data[ 1 ]( j ) ) < 0 ) // because there are only 2 classes
      ++correct;
  cout << "recognition rate = " 
       << (double)correct / total_ns << endl;
}

void print_result_predict( const SVM &svm, DataMatrix *data ){
  int total_ns = data[ 0 ].nSamples();
  for( int i = 1; i < svm.nClass(); ++i )
    total_ns += data[ i ].nSamples();

  int correct = 0;
  double *probability_estimates;
  for( int j = 0; j < data[ 0 ].nSamples(); ++j ){
    svm.recognize( data[ 0 ]( j ) );
    probability_estimates = svm.get_prob_estimates();
    if( probability_estimates[ 0 ] > probability_estimates[ 1 ] )
      ++correct;
    cout << "class 0: " << probability_estimates[ 0 ] << " " << probability_estimates[ 1 ] << endl;
  }
  for( int j = 0; j < data[ 1 ].nSamples(); ++j ){
    svm.recognize( data[ 1 ]( j ) );
    probability_estimates = svm.get_prob_estimates();
    if( probability_estimates[ 0 ] < probability_estimates[ 1 ] )
      ++correct;
    cout << "class 1: " << probability_estimates[ 0 ] << " " << probability_estimates[ 1 ] << endl;
  }
  cout << "recognition rate = " 
       << (double)correct / total_ns << endl;
}
