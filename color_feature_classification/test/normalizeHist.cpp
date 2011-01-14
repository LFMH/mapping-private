#define QUIET

#include <color_feature_classification/points_tools.hpp>
#include "color_chlac/grsd_colorCHLAC_tools.h"
#include <iostream>
#include "FILE_MODE"

using namespace pcl;
using namespace std;

const float lower = 0;//-1;
const float upper = 1;

void scaling( const int index, std::vector<float> &feature, const std::vector<float> feature_min, const std::vector<float> feature_max ) {
  if( feature_min[ index ] == feature_max[ index ] ) feature[ index ] = 0;
  else if( feature[ index ] == feature_min[ index ] ) feature[ index ] = lower;
  else if( feature[ index ] == feature_max[ index ] ) feature[ index ] = upper;
  else feature[ index ] = lower + (upper-lower) * ( feature[ index ] - feature_min[ index ] ) / ( feature_max[ index ] - feature_min[ index ] );
}

//-------
//* main
int main( int argc, char** argv ){
  if( argc != 4 ){
    ROS_ERROR ("Need three parameters! Syntax is: %s {input_pointcloud_filename.pcd} {bin_normalize_file} {output_histogram_filename.pcd}\n", argv[0]);
    return(-1);
  }
  std::vector<float> feature;
  std::vector<float> feature_min;
  std::vector<float> feature_max;

  // bin normalization parameters
  string filename;
  FILE *fp = fopen( argv[2], "r" );
  float val1, val2;
  while( fscanf( fp, "%f %f\n", &val1, &val2 ) != EOF ){
    feature_min.push_back( val1 );
    feature_max.push_back( val2 );
  }
  fclose( fp );

  //* read
  char line[ 100 ];
  string tmpname;
  int dim, sample_num;
  fp = fopen( argv[1], "r" );
  while( 1 ){
    fgets(line,sizeof(line),fp);
    tmpname = string (line);
    if( tmpname.compare (0, 5, "COUNT") == 0 )
      sscanf( line, "COUNT %d", &dim );
    else if( tmpname.compare (0, 6, "POINTS") == 0 )
      sscanf( line, "POINTS %d", &sample_num );
    else if( tmpname.compare (0, 4, "DATA") == 0 )
      break;
  }
  if( sample_num != 1 ){
    ROS_ERROR ("Input feature file is not global.\n");
    return(-1);
  }
  feature.resize( dim );
  for(int t=0;t<dim;t++){
    fscanf(fp,"%f ",&(feature[ t ]) );
    scaling( t, feature, feature_min, feature_max );
  }
  fclose(fp);

  //* write
  writeFeature( argv[ argc - 1 ], feature );
  
  return(0);
}
