#ifndef MY_VOXEL_HPP
#define MY_VOXEL_HPP

#ifdef USE_OPENCV
#include <cv.h>
#endif

#ifdef USE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#endif

#include <stdio.h>
#include "objFile.hpp"

/********************************************************/
/* Voxel�κ���/�ɤ߹��ߤ�Ԥ����饹                          */
/* �ä��ɤ߹��߻��ˤ�Color-CHLAC��ħ��и�����6�����ο�ɽ���ˤ��� */
/*  OpenCV�������Ȥ��Ȥ��� #define USE_OPENCV ����        */
/*   �ե�����ե����ޥåȡ�                                 */
/*      xsize ysize zsize                               */
/*      x y z r g b                                     */
/*      x y z r g b                                     */
/*           .                                          */
/*           .                                          */
/*                                                      */
/********************************************************/

//* RGB�ͤ��������ʬ�����
enum ReverseMode{ SIMPLE_REVERSE, TRIGONOMETRIC };

class Voxel{
public:
  Voxel();
  Voxel( bool _ascii ); // �ܥ������ɤ߽��ѤΥե����뤬�Х��ʥ꤫�ɤ��������
  Voxel( const char *filename, const char *mode ); // �ܥ������ɤ߽��ѤΥե�����Ȥ��γ����⡼�ɤ����
  Voxel( const Voxel &another );
  ~Voxel();

  //***********************//
  //* �ܥ�����������˻Ȥ��ؿ� *//
  //***********************//

  //* �ܥ�����ΰ��դ�Ĺ�������
  void setVoxelSize( float val );

  //* �ܥ����벽�оݥǡ����λ�������ɽ�κ����͡��Ǿ��ͤ����
  void setMinMax( float _x_min, float _x_max, float _y_min, float _y_max, float _z_min, float _z_max );

  //* ��å���ǡ����Υܥ����벽 RGB2�Ͳ�����
  void obj2voxel( Obj &object, const char *image_filename, ReverseMode mode );

  //* ��å���ǡ����Υܥ����벽 RGB2�Ͳ�����
  void obj2voxel( Obj &object, const char *image_filename, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );

  //* ��å���ǡ����Υܥ����벽 �ե�����˽���
  void obj2voxel_file( Obj &object, const char *image_filename );

#ifdef USE_PCL
  //* binarize RGB values
  void binarize( unsigned char thR, unsigned char thG, unsigned char thB );

  //* set the minimum and maximum xyz valuse of 3D data
  void setMinMax();

  //* transform point cloud into voxel data
  void points2voxel( pcl::PointCloud<pcl::PointXYZRGB> cloud_object_cluster, ReverseMode mode );
  void points2voxel( pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_object_cluster, ReverseMode mode );

  //* �ե�����˽���
  void writeVoxel();
#endif

#ifdef USE_OPENCV
  //* ��å���ǡ����Υܥ����벽 ���������OpenCV�Τ�� RGB2�Ͳ�����
  void obj2voxel( Obj &object, const cv::Mat& color_img, ReverseMode mode );

  //* ��å���ǡ����Υܥ����벽 ���������OpenCV�Τ�� RGB2�Ͳ�����
  void obj2voxel( Obj &object, const cv::Mat& color_img, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );
#endif

  //**************************//
  //* �ܥ������ɤ߹��߻��˻Ȥ��ؿ� *//
  //**************************//

  //* �ܥ������x����y����z����θĿ������
  void setXYZsize( int _xsize, int _ysize, int _zsize );

  //* �ܥ�����Υ������
  void createVoxelData();

  //* �ܥ�������ͤ�0������
  void cleanVoxelData();

  //* ����Ǿ��ͤ�Ͽ (������exist_flag���ͤ����������äƤ��뤳�Ȥ���)
  void getMinMax( int &x_min_i, int &x_max_i, int &y_min_i, int &y_max_i, int &z_min_i, int &z_max_i );

  //* �����ʬ�ͤ�׻����ʤ��ܥ�������ɤ߹���
  void readVoxel_normal( const char *filename );

  //* RGB���Ͳ����ʤ��ܥ�������ɤ߹��� �ʥ��󥹥���������˥ե�����̾����ꤹ�����
  void readVoxel( ReverseMode mode );

  //* RGB���Ͳ�����ܥ�������ɤ߹���  �ʥ��󥹥���������˥ե�����̾����ꤹ�����
  void readVoxel( unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );

  //* RGB���Ͳ����ʤ��ܥ�������ɤ߹��� �ʥ��󥹥���������˥ե�����̾����ꤷ�ʤ�����
  //* �� xsize, ysize, zsize ���ư�ǻ��ꤹ����� size_read_flg=false �Ȥ��� setXYZsize()�ؿ���ȤäƤ�������
  void readVoxel( const char *filename, ReverseMode mode, bool size_read_flg = true );

  //* RGB���Ͳ�����ܥ�������ɤ߹���  �ʥ��󥹥���������˥ե�����̾����ꤷ�ʤ�����
  //* �� xsize, ysize, zsize ���ư�ǻ��ꤹ����� size_read_flg=false �Ȥ��� setXYZsize()�ؿ���ȤäƤ�������
  void readVoxel( const char *filename, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b, bool size_read_flg = true );

  //* RGB���Ͳ����ʤ��ܥ�������ɤ߹��� ��z�����˥��ե��å�ʬ���餷��zsize�ʾ�Υܥ������ͤ�̵���
  //* �� xsize, ysize, zsize ���ư�ǻ��ꤹ����� size_read_flg=false �Ȥ��� setXYZsize()�ؿ���ȤäƤ�������
  void readVoxelZoffset( const char *filename, int zoffset, ReverseMode mode, bool size_read_flg = true );

  //* RGB���Ͳ�����ܥ�������ɤ߹���  ��z�����˥��ե��å�ʬ���餷��zsize�ʾ�Υܥ������ͤ�̵���
  //* �� xsize, ysize, zsize ���ư�ǻ��ꤹ����� size_read_flg=false �Ȥ��� setXYZsize()�ؿ���ȤäƤ�������
  void readVoxelZoffset( const char *filename, int zoffset, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b, bool size_read_flg = true );


  //***************//
  //* �Ƽ��ѿ��μ��� *//
  //***************//

  const int Xsize() const { return xsize; }
  const int Ysize() const { return ysize; }
  const int Zsize() const { return zsize; }
  const unsigned char* Vr() const { return vr; }
  const unsigned char* Vg() const { return vg; }
  const unsigned char* Vb() const { return vb; }
  const unsigned char* _Vr() const { return _vr; }
  const unsigned char* _Vg() const { return _vg; }
  const unsigned char* _Vb() const { return _vb; }
  const bool* Exist() const { return exist_flag; }
  const bool exist( int x, int y, int z ) const { return exist_flag[ x + y*xsize + z*xysize ]; }
  const unsigned char red( int x, int y, int z ) const { return vr[ x + y*xsize + z*xysize ]; }
  const unsigned char green( int x, int y, int z ) const { return vg[ x + y*xsize + z*xysize ]; }
  const unsigned char blue( int x, int y, int z ) const { return vb[ x + y*xsize + z*xysize ]; }

  const Voxel& operator=(const Voxel &another);

private:
  bool ascii; // �ܥ������ɤ߽��ѤΥե����뤬�Х��ʥ꤫�ɤ���
  FILE *fp_r;  // �ܥ������ɤ߹����ѤΥե�������
  FILE *fp_w;  // �ܥ�����񤭹����ѤΥե�������
  bool *exist_flag;   // �ܥ����뤬ʪ�Τ���ͭ����Ƥ��뤫�ݤ��ξ���
  int *exist_num;   // Number of points in a voxel
  int xsize;   // �ܥ������x����θĿ�
  int ysize;   // �ܥ������y����θĿ�
  int zsize;   // �ܥ������z����θĿ�
  int xysize;  // xsize * ysize
  int xyzsize; // xsize * ysize * zsize
  float x_min; // �ܥ����벽�оݥǡ����� x��ɸ�κǾ���
  float x_max; // �ܥ����벽�оݥǡ����� x��ɸ�κ�����
  float y_min; // �ܥ����벽�оݥǡ����� y��ɸ�κǾ���
  float y_max; // �ܥ����벽�оݥǡ����� y��ɸ�κ�����
  float z_min; // �ܥ����벽�оݥǡ����� z��ɸ�κǾ���
  float z_max; // �ܥ����벽�оݥǡ����� z��ɸ�κ�����
  unsigned char *vr;  // R �μ���ʬ��
  unsigned char *vg;  // G �μ���ʬ��
  unsigned char *vb;  // B �μ���ʬ��
  unsigned char *_vr; // R �������ʬ��
  unsigned char *_vg; // G �������ʬ��
  unsigned char *_vb; // B �������ʬ��
  float voxel_size; // �ܥ�����ΰ��դ�Ĺ��

  //* RGB �μ���ʬ�ͤ������ʬ�ͤ򥻥åȤ��� �����Ͳ����ʤ���
  //* �� RreverseMode �ǽ������Ƥ��Ѥ��
  void setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, ReverseMode mode );

  //* RGB �μ���ʬ�ͤ������ʬ�ͤ򥻥åȤ��� �����Ͳ������
  void setRGB( int idx, unsigned char r, unsigned char g, unsigned char b, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );

  //* �ƥ�å���Υܥ����벽 �ե�����˽���
  void face2voxel_file( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step );

  //* �ƥ�å���Υܥ����벽 RGB2�Ͳ�����
  void face2voxel( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step, ReverseMode mode );

  //* �ƥ�å���Υܥ����벽 RGB2�Ͳ�����
  void face2voxel( double x1, double x2, double x3, double y1, double y2, double y3, double z1, double z2, double z3, double u1, double u2, double u3, double v1, double v2, double v3, const unsigned char *image, int width, int height, int width_step, unsigned char color_threshold_r, unsigned char color_threshold_g, unsigned char color_threshold_b );
};

#endif
