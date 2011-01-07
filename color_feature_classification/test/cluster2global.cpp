#include <color_chlac/grsd_colorCHLAC_tools.h>
#include <iostream>

using namespace std;

int main( int argc, char** argv ){
  if( argc != 3 ){
    ROS_ERROR ("Need two parameters! Syntax is: %s [input_histogram.pcd] [options] [output_histogram.pcd]\n", argv[0]);
    return(-1);
  }
  int dim, sample_num;
  char line[ 100 ];
  string tmpname;
  std::vector<float> feature;
  float val;

  FILE *fpR = fopen( argv[1], "r" );
  FILE *fpW = fopen( argv[2], "w" );
  while( 1 ){
    fgets(line,sizeof(line),fpR);
    tmpname = string (line);
    if( tmpname.compare (0, 5, "COUNT") == 0 )
      sscanf( line, "COUNT %d", &dim );
    else if( tmpname.compare (0, 5, "WIDTH") == 0 ){
      fprintf( fpW, "WIDTH 1\n" );
      continue;
    }
    else if( tmpname.compare (0, 6, "POINTS") == 0 ){
      sscanf( line, "POINTS %d", &sample_num );
      fprintf( fpW, "POINTS 1\n" );
      continue;
    }
    else if( tmpname.compare (0, 4, "DATA") == 0 ){
      fprintf( fpW, "%s", line );
      break;
    }
    fprintf( fpW, "%s", line );
  }
  feature.resize( dim );
  for(int t=0;t<dim;t++)
    feature[ t ] = 0;

  for(int n=0;n<sample_num;n++){
    for(int t=0;t<dim;t++){
      fscanf(fpR,"%f ",&val );
      feature[ t ] += val;
    }
  }
  fclose(fpR);

  for(int t=0;t<dim;t++)
    fprintf( fpW, "%f ", feature[ t ] );

  fclose(fpW);
  
}
