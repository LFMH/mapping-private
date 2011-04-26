#define QUIET

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>
#include "color_voxel_recognition/pca.h"
#include "FILE_MODE"

using namespace pcl;
using namespace std;
using namespace terminal_tools;

//const float lower = 0;//-1;
const float upper = 1;

void scaling( const int index, std::vector<float> &feature, const std::vector<float> feature_max ) {
  if( feature_max[ index ] == 0 ) feature[ index ] = 0;
  else if( feature[ index ] == feature_max[ index ] ) feature[ index ] = upper;
  else feature[ index ] = upper * feature[ index ] / feature_max[ index ];
}

void
readFeatureModels ( int argc, char **argv, const std::string &extension, 
		    std::vector< std::vector<float> > &models, const string max_filename = "" )
{  
  char line[ 100 ];
  string tmpname;
  int dim, sample_num;
  std::vector<float> feature;
  std::vector<float> feature_max;
  bool is_normalize = false;

  //* bin normalization parameters
  if( max_filename.size() != 0 ){
    is_normalize = true;
    FILE *fp = fopen( max_filename.c_str(), "r" );
    float val;
    while( fscanf( fp, "%f\n", &val ) != EOF )
      feature_max.push_back( val );
    fclose( fp );
  }
  
  for (int i = 1; i < argc; i++){
    string fname = string (argv[i]);
    // Needs to have the right size
    if (fname.size () <= extension.size ())
      continue;
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0){
      FILE *fp = fopen( argv[i], "r" );
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
      feature.resize( dim );
      if( is_normalize ){
	for(int n=0;n<sample_num;n++){
	  for(int t=0;t<dim;t++){
	    fscanf(fp,"%f ",&(feature[ t ]) );
	    scaling( t, feature, feature_max );
	  }
	  models.push_back ( feature );
	}
      }
      else{
	for(int n=0;n<sample_num;n++){
	  for(int t=0;t<dim;t++)
	    fscanf(fp,"%f ",&(feature[ t ]) );
	  models.push_back ( feature );
	}
      }
      fclose(fp);
    }
  }
}

void compressFeature( string filename, std::vector< std::vector<float> > &models, const int dim, bool ascii ){
  PCA pca;
  pca.read( filename.c_str(), ascii );
  VectorXf variance = pca.getVariance();
  MatrixXf tmpMat = pca.getAxis();
  MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim);
  const int num = (int)models.size();
  for( int i=0; i<num; i++ ){
    Map<VectorXf> vec( &(models[i][0]), models[i].size() );
    //vec = tmpMat2.transpose() * vec;
    VectorXf tmpvec = tmpMat2.transpose() * vec;
    models[i].resize( dim );
    if( WHITENING ){
      for( int t=0; t<dim; t++ )
	models[i][t] = tmpvec[t] / sqrt( variance( t ) );
    }
    else{
      for( int t=0; t<dim; t++ )
	models[i][t] = tmpvec[t];
    }
  }
}

int main( int argc, char** argv ){
  if( argc < 3 ){
    ROS_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options] [output_plot_filename]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = size of compressed feature vectors\n");
    ROS_INFO ("                          -comp  filename = name of compress_axis file\n");
    ROS_INFO ("                          -comp2 filename = name of compress_axis file\n");
    ROS_INFO ("                          -norm filename = name of bin_normalize file\n");
    return(-1);
  }

  // compute features
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  std::vector< std::vector<float> > models;
  string filename;
  if( parse_argument (argc, argv, "-norm", filename) > 0 )
    readFeatureModels (argc, argv, extension, models, filename);
  else
    readFeatureModels (argc, argv, extension, models);

  // compress the dimension of the vector (if needed)
  int dim;
  if( parse_argument (argc, argv, "-dim", dim) > 0 ){
    if ((dim < 0)||(dim >= (int)models[0].size())){
      print_error ("Invalid dimension (%d)!\n", dim);
      return (-1);
    }
    parse_argument (argc, argv, "-comp", filename);
    compressFeature( filename, models, dim, false );
  }

  parse_argument (argc, argv, "-comp2", filename);
  compressFeature( filename, models, 3, false ); // projection to subspace

  FILE *fp = fopen( argv[ argc - 1 ], "w" );
  int num = (int)models.size();
  for( int i=0; i<num; i++ )
    fprintf(fp, "%f %f %f\n", models[ i ][ 0 ], models[ i ][ 1 ], models[ i ][ 2 ] );
  fclose( fp );

  return(0);
}
