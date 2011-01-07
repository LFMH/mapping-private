#define QUIET

#include <pcl/features/vfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>
#include <color_chlac/grsd_colorCHLAC_tools.h>
#include "color_feature_classification/libPCA.hpp"

using namespace pcl;
using namespace std;
using namespace terminal_tools;

void
readFeatureModels ( int argc, char **argv, const std::string &extension, 
		    std::vector< std::vector<float> > &models)
{  
  char line[ 100 ];
  string tmpname;
  int dim, sample_num;
  std::vector<float> feature;

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
      for(int n=0;n<sample_num;n++){
	for(int t=0;t<dim;t++)
	  fscanf(fp,"%f ",&(feature[ t ]) );
	models.push_back ( feature );
      }
      fclose(fp);
    }
  }
}

void compressFeature( string filename, std::vector< std::vector<float> > &models, const int dim, bool ascii ){
  PCA pca;
  pca.read( filename.c_str(), ascii );
  MatrixXf tmpMat = pca.Axis();
  MatrixXf tmpMat2 = tmpMat.block(0,0,tmpMat.rows(),dim);
  const int num = (int)models.size();
  for( int i=0; i<num; i++ ){
    Map<VectorXf> vec( &(models[i][0]), models[i].size() );
    //vec = tmpMat2.transpose() * vec;
    VectorXf tmpvec = tmpMat2.transpose() * vec;
    models[i].resize( dim );
    for( int t=0; t<dim; t++ )
      models[i][t] = tmpvec[t];
  }
}

// bool if_zero_vec( const std::vector<float> vec ){
//   const int vec_size = vec.size();
//   for( int i=0; i<vec_size; i++ )
//     if( vec[ i ] != 0 ) return false;
//   return true;
// }

void computeSubspace( std::vector< std::vector<float> > models, const char* filename, bool ascii ){
  cout << models[0].size() << endl;
  PCA pca( false );
  const int num = (int)models.size();
  for( int i=0; i<num; i++ )
    if( !if_zero_vec( models[ i ] ) )
      pca.addData( models[ i ] );
  pca.solve();
  pca.write( filename, ascii );
}

int main( int argc, char** argv ){
  if( argc < 3 ){
    ROS_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [options] [output_pca_name]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = size of compressed feature vectors\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    return(-1);
  }

  // compute features
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  std::vector< std::vector<float> > models;
  readFeatureModels (argc, argv, extension, models);

  // compress the dimension of the vector (if needed)
  int dim;
  if( parse_argument (argc, argv, "-dim", dim) > 0 ){
    if ((dim < 0)||(dim >= (int)models[0].size())){
      print_error ("Invalid dimension (%d)!\n", dim);
      return (-1);
    }
    string filename;
    parse_argument (argc, argv, "-comp", filename);
    compressFeature( filename, models, dim, false );
  }

  // compute subspace
  computeSubspace( models, argv[ argc - 1 ], false );

  return(0);
}
