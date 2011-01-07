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

// color threshold
int thR, thG, thB;
float voxel_size;
const int offset_step = 5;//2; // the step size of offset voxel number for subdivisions

void
computeFeatureModels ( const char feature_type, int argc, char **argv, const std::string &extension, 
		       std::vector< std::vector<float> > &models, const int subdivision_size)
{  
  const int repeat_num_offset = ceil(subdivision_size / offset_step);

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
      getVoxelGrid( grid, cloud, cloud_downsampled, voxel_size );

      // repeat with changing offset values for subdivisions
      for( int ox = 0; ox < repeat_num_offset; ox++ ){
	for( int oy = 0; oy < repeat_num_offset; oy++ ){
	  for( int oz = 0; oz < repeat_num_offset; oz++ ){
	    //* compute - GRSD -
	    std::vector< std::vector<float> > grsd;
	    computeGRSD( grid, cloud, cloud_downsampled, grsd, voxel_size, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step );
	    const int hist_num = grsd.size(); // number of subdivisions
	    
	    if( feature_type == 'g' ){
	      for( int h=0; h<hist_num; h++ )
		models.push_back ( grsd[ h ] );
	    }
	    else{
	      //* compute - ColorCHLAC - rotation-invariant
	      std::vector< std::vector<float> > colorCHLAC;
	      computeColorCHLAC_RI( grid, cloud_downsampled, colorCHLAC, thR, thG, thB, subdivision_size, ox*offset_step, oy*offset_step, oz*offset_step );
	      for( int h=0; h<hist_num; h++ )
		models.push_back ( conc_vector( grsd[ h ], colorCHLAC[ h ] ) );
	    }
	  }
	}
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
  if( argc < 4 ){
    ROS_ERROR ("Need at least three parameters! Syntax is: %s {feature_initial(g or r)} [model_directory] [options] [output_pca_name]\n", argv[0]);
    ROS_INFO ("    where [options] are:  -dim D = size of compressed feature vectors\n");
    ROS_INFO ("                          -comp filename = name of compress_axis file\n");
    ROS_INFO ("                          -subdiv N = subdivision size (e.g. 10 voxels)\n");
    return(-1);
  }

  // check feature_type
  const char feature_type = argv[1][0];
  if( (feature_type != 'g') && (feature_type != 'r') ){
    ROS_ERROR ("Unknown feature type. %c \n",feature_type);
    return(-1);
  }

  // color threshold
  FILE *fp = fopen( "color_threshold.txt", "r" );
  fscanf( fp, "%d %d %d\n", &thR, &thG, &thB );
  fclose(fp);

  // voxel size
  fp = fopen( "voxel_size.txt", "r" );
  fscanf( fp, "%f\n", &voxel_size );
  fclose(fp);

  // subdivision size
  int subdivision_size = 0;
  if( parse_argument (argc, argv, "-subdiv", subdivision_size) > 0 ){
    if ( subdivision_size < 0 ){
      print_error ("Invalid subdivision size (%d)! \n", subdivision_size);
      return (-1);
    }
  }

  // compute features
  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  std::vector< std::vector<float> > models;
  computeFeatureModels (feature_type, argc, argv, extension, models, subdivision_size);

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
