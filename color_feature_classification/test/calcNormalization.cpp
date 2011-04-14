#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <float.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>

using namespace std;
using namespace terminal_tools;

const int max_dim_size = 1500;

void
readHist_saveMinMax ( int argc, char **argv, const std::string &extension, const char* filename )
{  
  char line[ 100 ];
  string tmpname;
  int dim, sample_num;
  float val;
  std::vector<float> feature_max;

  feature_max.resize( max_dim_size );
  for( int i = 0; i < max_dim_size; i++ )
    feature_max[i]=-FLT_MAX;

  for (int i = 1; i < argc; i++){
    string fname = string (argv[i]);
    // Needs to have the right size
    if (fname.size () <= extension.size ())
      continue;
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0){
      FILE *fp = fopen( argv[i], "r" );
      cout<< argv[i] << endl;

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

      for(int n=0;n<sample_num;n++){
	for(int t=0;t<dim;t++){
	  fscanf(fp,"%f ",&val );
	  if( feature_max[ t ] < val ) feature_max[ t ] = val;
	}
      }
      fclose(fp);
    }
  }
  FILE *fp = fopen( filename, "w" );
  for( int i = 0; i < dim; i++ )
    fprintf( fp, "%f\n", feature_max[ i ] );
  fclose(fp);
}

int main(int argc, char** argv)
{
  if( argc < 3 ){
    std::cerr<< "Need at least two parameters! Syntax is: " << argv[0] << " {input_pointcloud_filename.pcd} {output(bin_normalization/max_X).txt}"<<std::endl;
    return(-1);
  }

  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  readHist_saveMinMax( argc, argv, extension, argv[ argc - 1 ] );

  return 0;
}
