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

// color threshold
int thR, thG, thB;

void
computeFeatureModels ( const char feature_type, int argc, char **argv, const std::string &extension, 
		       std::vector< std::vector<float> > &models)
{  
  for (int i = 1; i < argc; i++){
    string fname = string (argv[i]);
    // Needs to have the right size
    if (fname.size () <= extension.size ())
      continue;
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0){
      
      pcl::PointCloud<PointXYZRGB> cloud_object_cluster;
      readPoints( argv[i], cloud_object_cluster );

      //* compute normals
      pcl::PointCloud<PointXYZRGBNormal> cloud;
      computeNormal( cloud_object_cluster, cloud );
      
      //* voxelize
      pcl::VoxelGrid<PointXYZRGBNormal> grid;
      pcl::PointCloud<PointXYZRGBNormal> cloud_downsampled;
      getVoxelGrid( grid, cloud, cloud_downsampled );
      
      //* compute - GRSD -
      std::vector<float> grsd;
      computeGRSD( grid, cloud, cloud_downsampled, grsd );

      if( feature_type == 'g' ) models.push_back (grsd);
      else{
	//* compute - ColorCHLAC -
	std::vector< float > colorCHLAC;
	computeColorCHLAC( grid, cloud_downsampled, colorCHLAC, thR, thG, thB );
	models.push_back ( conc_vector( grsd, colorCHLAC ) );
      }
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

void computeSubspace( std::vector< std::vector<float> > models, const char* filename, bool ascii ){
  cout << models[0].size() << endl;
  PCA pca( false );
  const int num = (int)models.size();
  for( int i=0; i<num; i++ )
    pca.addData( models[ i ] );
  pca.solve();
  pca.write( filename, ascii );
}

int main( int argc, char** argv ){
  if( argc < 4 ){
    ROS_ERROR ("Need at least three parameters! Syntax is: %s {feature_initial(g or r)} [model_directory] [options] [output_pca_name]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = size of compressed feature vectors\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    return(-1);
  }

  // check feature_type
  const char feature_type = argv[1][0];
  if( (feature_type != 'g') && (feature_type != 'r') ){
    ROS_ERROR ("Unknown feature type.\n");
    return(-1);
  }

  // color threshold
  FILE *fp = fopen( "color_threshold.txt", "r" );
  fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
  fclose(fp);

  // compute features
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  std::vector< std::vector<float> > models;
  computeFeatureModels (feature_type, argc, argv, extension, models);

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
