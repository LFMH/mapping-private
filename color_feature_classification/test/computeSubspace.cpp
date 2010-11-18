#include <pcl/features/vfh.h>
#include <vfh_cluster_classifier/common_io.h>
#include <terminal_tools/parse.h>
#include <terminal_tools/print.h>
#include "color_feature_classification/libPCA.hpp"

using namespace pcl;
using namespace terminal_tools;
using vfh_cluster_classifier::vfh_model;

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of VFH features that will act as the model (training data)
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param extension the file extension containing the VFH features
  * \param models the resultant vector of histogram models
  */
void
  loadFeatureModels (int argc, char **argv, const std::string &extension, 
                     std::vector<vfh_model> &models)
{
  // Suppose the first argument is the actual test model
  for (int i = 1; i < argc; i++)
  {
    string fname = string (argv[i]);
    // Needs to have the right size
    if (fname.size () <= extension.size ())
      continue;
 
    transform (fname.begin (), fname.end (), fname.begin (), (int(*)(int))tolower);
    if (fname.compare (fname.size () - extension.size (), extension.size (), extension) == 0)
    {
      // Load the histogram model and saves it into the global list of models
      vfh_model m;
      if (vfh_cluster_classifier::loadHist (argv[i], m))
      {
        print_highlight ("Loading %s (%zu models loaded so far).\n", argv[i], models.size ());
        models.push_back (m);
      }
    }
  }
}

void computeSubspace( std::vector<vfh_model> models, const char* filename, bool ascii ){
  PCA pca( false );
  int num = (int)models.size();
  for( int i=0; i<num; i++ )
    pca.addData( models[ i ].second );
  pca.solve();  
  pca.write( filename, ascii );
}

int main( int argc, char** argv ){
  if( argc < 3 ){
    ROS_ERROR ("Need at least two parameters! Syntax is: %s [model_directory] [output_pca_name]\n", argv[0]);
    return(-1);
  }
  //argc--; // because argv[ argc-1 ] is not a model file name.

  std::string extension (".pcd");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);
  std::vector<vfh_model> models;

  loadFeatureModels (argc, argv, extension, models);
  computeSubspace( models, argv[ argc - 1 ], false );

  return(0);
}
