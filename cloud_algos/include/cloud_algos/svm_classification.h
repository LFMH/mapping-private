#ifndef CLOUD_ALGOS_SVM_CLASSIFICATION_H
#define CLOUD_ALGOS_SVM_CLASSIFICATION_H
#include <cloud_algos/cloud_algos.h>

#include <float.h>
// TODO: keep only needed ones
#include <iostream>
#include <sstream>
#include <fstream>
#include <string>
#include <string.h>
#include <algorithm>
#include <vector>
#include <set>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>

// TODO: check if the license is fine, and include it somewhere else, if it is not in ROS already...
#include <cloud_algos/StringTokenizer.h>

#include <libsvm/svm.h>

namespace cloud_algos
{

class SVMClassification : public CloudAlgo
{
 public:

  // Input/output type
  typedef sensor_msgs::PointCloud OutputType;
  typedef sensor_msgs::PointCloud InputType;

  // Options
  std::string model_file_name_; // filename where the model should be loaded from
  std::string scale_file_name_; // filename where the model should be loaded from
  bool scale_self_;             // should data be scaled first or not (see: scale_file)
  bool scale_file_;             // if scale_self_ disabled, get scale parameters from scale_file_name_

  // Default names
  static std::string default_input_topic ()
    {return std::string ("cloud_pcd");}
  static std::string default_output_topic ()
    {return std::string ("cloud_svm");};
  static std::string default_node_name () 
    {return std::string ("svm_classification_node");};

  // Algorithm methods
  void init (ros::NodeHandle&);
  void pre ();
  void post ();
  std::vector<std::string> requires  ();
  std::vector<std::string> provides ();
  std::string process (const boost::shared_ptr<const InputType>&);
  OutputType output ();

  // Constructor-Destructor
  SVMClassification () : CloudAlgo ()
  {
    model_file_name_ = std::string ("svm/fpfh.model");
    //scale_file_name_ = std::string ("svm/fpfh.scp");
    scale_file_name_ = std::string ("svm/teapot_smooth_fpfh.scp");
    scale_self_ = false;
    scale_file_ = true; // gets considered only if scale_self is false
  }

  static inline double
    scaleFeature (int index, double value, double **min_max_values, double lower, double upper)
  {
    // Skip single-valued attributes
    if(min_max_values[0][index] == min_max_values[1][index])
      return (0);

    // TODO: do the first two cases help?
    if (value == min_max_values[0][index])
      value = lower;
    else if (value == min_max_values[1][index])
      value = upper;
    else
      // Linear interpolation
      value = lower + (upper - lower) * (value - min_max_values[0][index]) / (min_max_values[1][index] - min_max_values[0][index]);

    return value;
  }

  // TODO make this a general function which gets a list of channels / all channels... point_cloud_mapping::statistics doesn't have it
  static inline double**
    computeScaleParameters (const boost::shared_ptr<const InputType>& cloud, int startIdx, int nr_values)
  {
    // Allocate result vector for min and max values
    double*  p   = new double[2*nr_values];
    for (int i = 0; i < nr_values; i++)
    {
      p[i] = DBL_MAX;
      p[nr_values + i] = -DBL_MAX;
    }
    double** res = new double*[2];
    for (int i = 0; i < 2; i++)
      res[i] = & ( p[i*nr_values] );

    for (int i = 0; i < nr_values; i++)
      std::cerr << i << ": " << res[0][i] << " " << res[1][i] << std::endl;

    // Go through all the channels and compute min&max
    for (size_t cp = 0; cp < cloud->points.size (); cp++)
    {
      // TODO: parallelize, maybe put this as outer for
      for (int i = 0; i < nr_values; i++)
      {
        if (res[0][i] > cloud->channels[startIdx+i].values[cp])
          res[0][i] = cloud->channels[startIdx+i].values[cp];
        else if (res[1][i] < cloud->channels[startIdx+i].values[cp])
          res[1][i] = cloud->channels[startIdx+i].values[cp];
      }
    }

    for (int i = 0; i < nr_values; i++)
      std::cerr << i << ": " << res[0][i] << " " << res[1][i] << std::endl;

    // Return result
    return res;
  }

  static inline double**
    parseScaleParameterFile (const char *fileName, double &lower, double &upper, int nr_values, bool verbose = true)
  {
    // Allocate result vector for min and max values
    double*  p   = new double[2*nr_values];
    for (int i = 0; i < 2*nr_values; i++) p[i] = 0.0;
    double** res = new double*[2];
    for (int i = 0; i < 2; i++)
      res[i] = & ( p[i*nr_values] );
    //ANNpointArray res = annAllocPts (2, nr_values, 0);

    // For reading from files
    std::ifstream fs;
    std::string line;

    // Open file
    fs.open (fileName);
    if (!fs.is_open ())
    {
      if (verbose)
        ROS_ERROR ("Couldn't open %s for reading!", fileName);
      return NULL;
    }

    // Get the type
    getline (fs, line);
    if (line.substr (0, 1) != "x")
    {
      if (verbose)
        ROS_WARN ("X scaling not found in %s or unknown!", fileName);
      return NULL;
    }

    // Get the bounds
    getline (fs, line);
    StringTokenizer st = StringTokenizer (line, " ");
    lower = st.nextFloatToken ();
    upper = st.nextFloatToken ();

    // Get the min and max values
    int i = 0;
    while (!fs.eof ())
    {
      i++;
      getline (fs, line);
      if (line == "")
        continue;

      StringTokenizer st = StringTokenizer (line, " ");

      int idx = st.nextIntToken ();
      float fmin = st.nextFloatToken ();
      float fmax = st.nextFloatToken ();
      if (idx <= nr_values)
      {
        res[0][idx-1] = fmin;
        res[1][idx-1] = fmax;
      }
    }

    // Close file
    fs.close ();

    // Return result
    return res;
  }

 private:

  // ROS stuff
  ros::NodeHandle nh_;
  ros::Publisher pub_;

  // ROS messages
  boost::shared_ptr<sensor_msgs::PointCloud> cloud_svm_;
};

}
#endif
