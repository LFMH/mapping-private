/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: build_tree.cpp 43313 2010-08-18 16:30:38Z rusu $
 *
 */

/**
  * \author Radu Bogdan Rusu 
  *
  * @b build_tree creates a FLANN kd-tree from a list of VFH signatures
  * (training data), and saves the tree to disk.
  */

#include <vfh_cluster_classifier/vfh_nearest_neighbors.h>
#include <color_chlac/ColorCHLAC.hpp>

////////////////////////////////////////////////////////////////////////////////
/** \brief Load a set of VFH features that will act as the model (training data)
  * \param argc the number of arguments (pass from main ())
  * \param argv the actual command line arguments (pass from main ())
  * \param extension the file extension containing the VFH features
  * \param models the resultant vector of histogram models
  */
/*void
  loadFeatureModels (const boost::filesystem::path &base_dir, const std::string &extension, 
                     std::vector<vfh_model> &models)
{
  if (!boost::filesystem::exists (base_dir) && !boost::filesystem::is_directory (base_dir))
    return;

  for (boost::filesystem::directory_iterator it (base_dir); it != boost::filesystem::directory_iterator (); ++it)
  {
    if (boost::filesystem::is_directory (it->status ()))
    {
      stringstream ss;
      ss << it->path ();
      print_highlight ("Loading %s (%zu models loaded so far).\n", ss.str ().c_str (), models.size ());
      loadFeatureModels (it->path (), extension, models);
    }
    if (boost::filesystem::is_regular_file (it->status ()) && boost::filesystem::extension (it->path ()) == extension)
    {
      vfh_model m;
      if (vfh_cluster_classifier::loadHist (base_dir / it->path ().filename (), m))
        models.push_back (m);
    }
  }
}*/

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
      if (!vfh_cluster_classifier::loadHist (argv[i], m)){
	m.first = argv[i];
	m.second.resize( DIM_COLOR_BIN_1_3+DIM_COLOR_1_3 );
	// read file normaly
	char line[ 100 ];
	FILE *fp = fopen( argv[i], "r" );
	for( int t=0; t<9; t++ )
	  fgets(line,sizeof(line),fp);
	for(int t=0;t<DIM_COLOR_BIN_1_3+DIM_COLOR_1_3;t++)
	  fscanf(fp,"%f ",&(m.second[ t ]) );
	fclose(fp);
      }
      print_highlight ("Loading %s (%zu models loaded so far).\n", argv[i], models.size ());
      models.push_back (m);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Convert data into FLANN format
  * \param models a list of histogram models to convert
  */
void
  convertToFLANN (const vector<vfh_model> &models, flann::Matrix<float> &data)
{
  data.rows = models.size ();
  data.cols = models[0].second.size (); // number of histogram bins
  data.data = (float*)malloc (data.rows * data.cols * sizeof (float)); 

  for (size_t i = 0; i < data.rows; ++i)
    for (size_t j = 0; j < data.cols; ++j)
      data.data[i * data.cols  + j] = models[i].second[j];
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Save the list of file models. 
  * \param models the list models
  * \param filename the output file name
  */
void
  saveFileList (const std::vector<vfh_model> &models, const std::string &filename)
{
  std::ofstream fs;
  fs.open (filename.c_str ());
  for (size_t i = 0; i < models.size (); ++i)
    fs << models[i].first << "\n";
  fs.close ();
}

/* ---[ */
int
  main (int argc, char** argv)
{
  int linear = 0;
  int metric = 7;
  if (argc < 3)
  {
    print_error ("Need at least two parameters! Syntax is: %s [model_directory] [options] {kdtree.idx} {training_data.h5} {training_data.list}\n", argv[0]);

    print_info ("    where [options] are:  -metric X = metric/distance type (1 = Euclidean, 2 = Manhattan, 3 = Minkowski, 4 = Max, 5 = HIK, 6 = JM, 7 = Chi-Square, 8 = KL). Default: "); print_value ("%d\n", metric);
    print_info ("                          -linear X = approximate nearest neighbor search (1) versus brute force (0). Default: "); print_value ("%d\n\n", linear);
    //
    print_info ("     * note:\n");
    print_info ("             - the metric_type has to match the metric that was used when the tree was created.\n");
    print_info ("             - the last three parameters are optional and represent: the kdtree index file (default: "); print_value ("kdtree.idx"); print_info (")\n"
                "                                                                     the training data used to create the tree (default: "); print_value ("training_data.h5"); print_info (")\n"
                "                                                                     the list of models used in the training data (default: "); print_value ("training_data.list"); print_info (")\n");
     return (-1);
  }

  std::string extension (".pcd");
  //std::string extension (".vpfh");
  transform (extension.begin (), extension.end (), extension.begin (), (int(*)(int))tolower);

  // Set the tree metric type
  parse_argument (argc, argv, "-metric", metric);
  if (metric < 0 || metric > 8)
  {
    print_error ("Invalid metric specified (%d)!\n", metric);
    return (-1);
  }
  flann_set_distance_type ((flann_distance_t)metric, 0);
  print_highlight ("Using distance metric = "); print_value ("%d\n", metric); 

  parse_argument (argc, argv, "-linear", linear);
  print_highlight ("Linear = "); print_value ("%d\n", linear); 

  // --[ Read the kdtree index file
  string kdtree_idx_file_name = "kdtree.idx";
  vector<int> idx_indices = parse_file_extension_argument (argc, argv, ".idx");
  if (idx_indices.size () > 1)
  {
    print_error ("Need a single kdtree index file!\n");
    return (-1);
  }
  if (idx_indices.size () == 1)
    kdtree_idx_file_name = argv[idx_indices.at (0)];
  if (boost::filesystem::exists (kdtree_idx_file_name))
  {
    print_error ("Kd-tree index found in %s. Cannot overwrite.\n", kdtree_idx_file_name.c_str ());
    return (-1);
  }
  else
  {
    print_highlight ("Using "); print_value ("%s", kdtree_idx_file_name.c_str ()); print_info (" as the kdtree index file.\n");
  }

  // --[ Read the training data h5 file
  string training_data_h5_file_name = "training_data.h5";
  vector<int> train_h5_indices = parse_file_extension_argument (argc, argv, ".h5");
  if (train_h5_indices.size () > 1)
  {
    print_error ("Need a single h5 training data file!\n");
    return (-1);
  }
  if (train_h5_indices.size () == 1)
    training_data_h5_file_name = argv[train_h5_indices.at (0)];

  // --[ Read the training data list file
  string training_data_list_file_name = "training_data.list";
  vector<int> train_list_indices = parse_file_extension_argument (argc, argv, ".list");
  if (train_list_indices.size () > 1)
  {
    print_error ("Need a single list training data file!\n");
    return (-1);
  }
  if (train_list_indices.size () == 1)
    training_data_list_file_name = argv[train_list_indices.at (0)];

  std::vector<vfh_model> models;
  flann::Matrix<float> data;

  // Check if the data has already been saved to disk
  if (!boost::filesystem::exists (training_data_h5_file_name) && !boost::filesystem::exists (training_data_list_file_name))
  {
    // Load the model histograms
    //loadFeatureModels (argv[1], extension, models);
    loadFeatureModels (argc, argv, extension, models);
    print_highlight ("Loaded %d VFH models. Creating training data %s/%s.\n", (int)models.size (), training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    convertToFLANN (models, data);
    flann::save_to_file (data, training_data_h5_file_name, "training_data");
    saveFileList (models, training_data_list_file_name);
  }
  else
  {
    print_error ("Training data found in %s/%s. Cannot overwrite.\n", training_data_h5_file_name.c_str (), training_data_list_file_name.c_str ());
    return (-1);
  }

  // Build the tree index and save it to disk
  print_error ("Building the kdtree index (%s) for %d elements...\n", kdtree_idx_file_name.c_str (), (int)data.rows);
  if (linear == 0)
  {
    flann::Index< flann::L2<float> > index (data, flann::LinearIndexParams ());
    index.buildIndex ();
    index.save (kdtree_idx_file_name);
  }
  else
  {
    flann::Index< flann::L2<float> > index (data, flann::KDTreeIndexParams (4));
    index.buildIndex ();
    index.save (kdtree_idx_file_name);
  }

  return (0);
}
/* ]--- */
