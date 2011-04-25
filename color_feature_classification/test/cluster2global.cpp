#include <iostream>
#include <vector>
#include <stdio.h>

int main( int argc, char** argv ){
  if( argc != 3 ){
    std::cerr<< "Need two parameters! Syntax is: " << argv[0] << " [input_histogram.pcd] [output_histogram.pcd]" << std::endl;
    return(-1);
  }
  int dim, sample_num;
  char line[ 100 ];
  std::string tmpname;
  std::vector<float> feature;
  float val;

  FILE *fpR = fopen( argv[1], "r" );
  FILE *fpW = fopen( argv[2], "w" );
  while( 1 ){
    fgets(line,sizeof(line),fpR);
    tmpname = std::string (line);
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
