#ifndef MY_COLOR_VOXEL_HPP
#define MY_COLOR_VOXEL_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//* definition of inverse-RGB values (two types)
//* ! Note that TRIGONOMETRIC mode is not available before my paper is published.
enum ReverseMode{ SIMPLE_REVERSE, TRIGONOMETRIC };

class ColorVoxel{
public:
  ColorVoxel();
  ColorVoxel( const ColorVoxel& another );
  ~ColorVoxel();

  //* set the length of the side of each voxel
  void setVoxelSize( float val );

  //* set the minimum and maximum xyz valuse of 3D data
  void setMinMax();

  //* transform point cloud into voxel data
  void points2voxel( pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster, ReverseMode mode );

  //* binarize RGB values
  void binarize( unsigned char thR, unsigned char thG, unsigned char thB );

  //* allocate voxel memory
  void createVoxelData();

  //* initialize voxel values by 0
  void cleanVoxelData();

  int Xsize() { return xsize; }
  int Ysize() { return ysize; }
  int Zsize() { return zsize; }
  const unsigned char* Vr() const { return vr; }
  const unsigned char* Vg() const { return vg; }
  const unsigned char* Vb() const { return vb; }
  const unsigned char* _Vr() const { return _vr; }
  const unsigned char* _Vg() const { return _vg; }
  const unsigned char* _Vb() const { return _vb; }
  const ColorVoxel& operator=(const ColorVoxel &another);

private:
  int *exist_num;   // Number of points in a voxel
  int xsize;   // Number of voxels on x axis
  int ysize;   // Number of voxels on y axis
  int zsize;   // Number of voxels on z axis
  int xysize;  // xsize * ysize
  int xyzsize; // xsize * ysize * zsize
  float x_min; // Minimum x value of points
  float x_max; // Maximum x value of points
  float y_min; // Minimum y value of points
  float y_max; // Maximum y value of points
  float z_min; // Minimum z value of points
  float z_max; // Maximum z value of points
  unsigned char *vr;  // R value
  unsigned char *vg;  // G value
  unsigned char *vb;  // B value
  unsigned char *_vr; // inverse-R value
  unsigned char *_vg; // inverse-G value
  unsigned char *_vb; // inverse-B value
  float voxel_size;   // length of the side of each voxel

  //* set RGB values and inverse-RGB values (without binarize)
  void setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, ReverseMode mode );

  //* set RGB values and inverse-RGB values (with binarize)
  void setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );
};

#endif
