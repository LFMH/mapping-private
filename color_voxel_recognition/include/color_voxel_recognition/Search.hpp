#ifndef MY_SEARCH_HPP
#define MY_SEARCH_HPP

#include <pcl/point_types.h>
#include "pcl/filters/voxel_grid.h"

/*****************************************************************/
/* Class for Sliding-Box Object Detection                        */
/*  the most similar box regions to the target object are shown. */
/*****************************************************************/

// Detection Mode
// 6 patterns in giving out range1, range2 and range3 to x, y and z.
enum SearchMode{ S_MODE_1, S_MODE_2, S_MODE_3, S_MODE_4, S_MODE_5, S_MODE_6 };

//-------------------------//
// single object detection //
//-------------------------//
class SearchObj{
public:
  double search_time; // time for search
  SearchObj();
  ~SearchObj();

  // set size of sliding box
  void setRange( int _range1, int _range2, int _range3 );

  // set number of output regions which are $rank_num most similar to the target object
  virtual void setRank( int _rank_num );

  // set threshold of number of occupied voxels in a box region
  void setThreshold( int _exist_voxel_num_threshold );

  // read projection axis of the target object's subspace
  virtual void readAxis( const char *filename, int dim, int dim_model, bool ascii, bool multiple_similarity );

  // read integral feature table (and voxel numbers) of scene
  void readData( const char *filenameF, const char *filenameN, int dim, bool ascii );

  // return xrange, yrange and zrange of sliding box
  void getRange( int &xrange, int &yrange, int &zrange, SearchMode mode );

  // object detection with rotation
  void search();

  // object detection without rotation
  void search_withoutRotation();

  // output the results in a file
  void writeResult( const char *filename, int box_size );

  // delete the results
  virtual void cleanMax();

  // set projection axis for feature compression (without whitening)
  void setSceneAxis( Eigen::MatrixXf _axis );

  // set projection axis for feature compression (with whitening)
  void setSceneAxis( Eigen::MatrixXf _axis, Eigen::VectorXf var, int dim );

  // delete the results and integral feature table
  virtual void cleanData();

  // return xy_num (= x_num * y_num)
  int XYnum() const { return xy_num; }

  // return z_num
  int Znum() const { return z_num; }

  // return coordinate x of ($num-th) detected region
  virtual int maxX( int num ) const { return max_x[ num ]; }

  // return coordinate y of ($num-th) detected region
  virtual int maxY( int num ) const { return max_y[ num ]; }

  // return coordinate z of ($num-th) detected region
  virtual int maxZ( int num ) const { return max_z[ num ]; }

  // return SearchMode of ($num-th) detected region
  virtual SearchMode maxMode( int num ) const { return max_mode[ num ]; }

  // return similarity of ($num-th) detected region
  virtual double maxDot( int num ) const { return max_dot[ num ]; }

  // return x length of ($num-th) detected region
  virtual int maxXrange( int num );

  // return y length of ($num-th) detected region
  virtual int maxYrange( int num );

  // return z length of ($num-th) detected region
  virtual int maxZrange( int num );

  // set values for feature normalization
  void setNormalizeVal( const char* filename );

protected:
  int range1; // side length (1) of sliding box
  int range2; // side length (2) of sliding box
  int range3; // side length (3) of sliding box
  int x_num;  // number of subdivisions in scene on x-axis
  int y_num;  // number of subdivisions in scene on y-axis
  int z_num;  // number of subdivisions in scene on z-axis
  int xy_num; // x_num * y_num
  int rank_num; // number of output regions which are $rank_num most similar to the target object
  int exist_voxel_num_threshold; // threshold of number of occupied voxels in a box region
  int *max_x; // coordinate x of ($rank_num best) detected region
  int *max_y; // coordinate y of ($rank_num best) detected region
  int *max_z; // coordinate z of ($rank_num best) detected region
  SearchMode *max_mode;   // SearchMode of ($rank_num best) detected region
  double *max_dot;        // similarity of ($rank_num best) detected region
  Eigen::MatrixXf axis_q; // projection axis of the target object's subspace
  int *exist_voxel_num;   // number of occupied voxels in a box region of scene
  Eigen::VectorXf *nFeatures; // integral feature table of scene
  Eigen::MatrixXf axis_p;     // projection axis for feature compression
  std::vector<float> feature_max; // values for histogram normalization
  bool compress_flg; // flag to compress features or not

  // return xrange of sliding box
  int xRange( SearchMode mode );

  // return yrange of sliding box
  int yRange( SearchMode mode );

 // return zrange of sliding box
  int zRange( SearchMode mode );

  // check if the reference region is overlapped by some region already detected
  virtual int checkOverlap( int x, int y, int z, SearchMode mode );

  // copy the info of $src_num-th detected region to $dest_num-th detected region
  virtual void max_cpy( int src_num, int dest_num );

  // replace the info of $dest_num-th detected region
  virtual void max_assign( int dest_num, double dot, int x, int y, int z, SearchMode mode );

  // object detection
  virtual void search_part( SearchMode mode );

  // set integral feature table (and voxel numbers) of scene
  void setData( const Eigen::Vector3i subdiv_b_, std::vector< std::vector<float> > feature );

  // extract features and voxel numbers of reference region from integral feature table
  template <typename T>
  T clipValue( T* ptr, const int x, const int y, const int z, const int xrange, const int yrange, const int zrange );
};

//----------------------------//
// multiple objects detection //
//----------------------------//
class SearchObj_multi : public SearchObj {
public:
  using SearchObj::search_time; // time for search
  SearchObj_multi();
  ~SearchObj_multi();

  // set number of detection-target objects
  void setModelNum( int model_num_ ){ model_num = model_num_; }

  // set number of output regions which are $rank_num most similar to the target object
  void setRank( int _rank_num );

  // read projection axis of the target object's subspace
  void readAxis( char **filename, int dim, int dim_model, bool ascii, bool multiple_similarity ); // multi

  //* TODO implement the following:
  //void writeResult( const char *filename, int box_size );

  // delete the results
  void cleanMax();

  // delete the results and integral feature table
  void cleanData();

  // return coordinate x of ($num-th) detected region
  int maxX( int m_num, int num ) const { return max_x_multi[ m_num ][ num ]; }

  // return coordinate y of ($num-th) detected region
  int maxY( int m_num, int num ) const { return max_y_multi[ m_num ][ num ]; }

  // return coordinate z of ($num-th) detected region
  int maxZ( int m_num, int num ) const { return max_z_multi[ m_num ][ num ]; }

  // return SearchMode of ($num-th) detected region
  SearchMode maxMode( int m_num, int num ) const { return max_mode_multi[ m_num ][ num ]; }

  // return similarity of ($num-th) detected region
  double maxDot( int m_num, int num ) const { return max_dot_multi[ m_num ][ num ]; }

  // return x length of ($num-th) detected region
  int maxXrange( int m_num, int num );

  // return y length of ($num-th) detected region
  int maxYrange( int m_num, int num );

  // return z length of ($num-th) detected region
  int maxZrange( int m_num, int num );

  // solve and remove overlap between detected regions for object1, object2, ...
  void removeOverlap();

protected:
  int model_num;           // number of detection-target objects
  using SearchObj::range1; // side length (1) of sliding box
  using SearchObj::range2; // side length (2) of sliding box
  using SearchObj::range3; // side length (3) of sliding box
  using SearchObj::x_num;  // number of subdivisions in scene on x-axis
  using SearchObj::y_num;  // number of subdivisions in scene on y-axis
  using SearchObj::z_num;  // number of subdivisions in scene on z-axis
  using SearchObj::xy_num; // x_num * _num
  using SearchObj::rank_num; // number of output regions which are $rank_num most similar to the target object
  using SearchObj::exist_voxel_num_threshold; // threshold of number of occupied voxels in a box region
  int **max_x_multi; // coordinate x of ($rank_num best) detected region
  int **max_y_multi; // coordinate y of ($rank_num best) detected region
  int **max_z_multi; // coordinate z of ($rank_num best) detected region
  SearchMode **max_mode_multi;   // SearchMode of ($rank_num best) detected region
  double **max_dot_multi;        // similarity of ($rank_num best) detected region
  Eigen::MatrixXf *axis_q_multi; // projection axis of the target object's subspace
  using SearchObj::exist_voxel_num; // number of occupied voxels in a box region of scene
  using SearchObj::nFeatures; // integral feature table of scene
  using SearchObj::axis_p;    // projection axis for feature compression
  using SearchObj::feature_max; // values for histogram normalization
  using SearchObj::compress_flg; // flag to compress features or not

  // check if the reference region is overlapped by some region already detected
  int checkOverlap( int m_num, int x, int y, int z, SearchMode mode );

  // copy the info of $src_num-th detected region to $dest_num-th detected region
  void max_cpy( int m_num, int src_num, int dest_num );

  // replace the info of $dest_num-th detected region
  void max_assign( int m_num, int dest_num, double dot, int x, int y, int z, SearchMode mode );

  // object detection
  void search_part( SearchMode mode );
};

#endif
