#ifndef MY_PARAM_HPP
#define MY_PARAM_HPP

/*****************************/
/* read and write parameters */
/*****************************/

class Param{
public:

  // read the length of voxel side
  static float readVoxelSize( const char* filename = "param/parameters.txt" );

  // read the dimension of compressed feature vectors
  static int readDim( const char* filename = "param/parameters.txt" );

  // read the number of voxels in each subdivision's side of scene
  static int readBoxSize_scene( const char* filename = "param/parameters.txt" );

  // read the number of voxels in each subdivision's side of a target object
  static int readBoxSize_model( const char* filename = "param/parameters.txt" );

  // // read the max number of voxels created once
  // static int readMaxVoxelNum( const char* filename = "param/parameters.txt" );

  // read the number for synthetic rotation in learning objects
  static int readRotateNum( const char* filename = "param/parameters.txt" );

  // read flag for c3_hlac extraction
  static int readC3HLAC_flag( const char* filename = "param/parameters.txt" );

  // read the threshold for RGB binalize
  static void readColorThreshold( int &r, int &g, int &b, const char* filename = "param/color_threshold.txt" );

  // // read parameter of SR confidence for auto mesh construction
  // static int readConfTh( const char* filename );

  // // read parameters for auto mesh construction
  // static void readParamAuto( float& length_max_rate, float& length_th, float& distance_th, int& confidence_th, bool &relative_mode, const char* filename );

  // // write parameters for auto mesh construction
  // static void writeParamAuto( float length_max_rate, float length_th, float distance_th, int confidence_th, bool relative_mode, const char* filename );

private:
  static bool readParam( const char* filename, const char *param_string, int &val );
  static bool readParam( const char* filename, const char *param_string, float &val );

  // no constructor
  Param();
};

#endif
