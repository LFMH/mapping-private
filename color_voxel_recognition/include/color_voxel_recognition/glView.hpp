#ifndef MY_GLVIEW_HPP
#define MY_GLVIEW_HPP

#include <GL/glut.h>
#include "objFile.hpp"
#include "ppmFile.hpp"
#include "Voxel.hpp"

/****************************************************/
/* OpenGL�ǥܥ�������å����ɽ������                   */
/* �Ȥ��Ȥ��� ���Υ��饹��Ѿ����ƿ����˥��饹���ꡢ        */
/*   display()��ɬ�ܡ�ȡ�keyboard()��Ǥ�ա��������뤳�� */
/****************************************************/

class GlView{
public:
  GlView();
  ~GlView(){}
  void setScale( float val ){ scale = val; }
  void setWinSize( int w, int h ){ win_width = w; win_height = h; }
  const float getCenterX() const { return center_x; }
  const float getCenterY() const { return center_y; }
  const float getCenterZ() const { return center_z; }

  //* ���ߤβ��̤β�������¸
  void capture( const char* filename );

  //* ����Τ���˥ܥ�������ɤ߹��� 
  void readVoxel( Voxel& voxel, const char* filename );

  //* ����Τ���˥ܥ�������ɤ߹��� �����ϰϤΤ�
  void readVoxel( Voxel& voxel, const char* filename, int sx, int rx, int sy, int ry, int sz, int rz );

  //* ����Τ���˥�å�����ɤ߹���
  void readMesh( Obj** object, Ppm** ppm, char **obj_names, char **tex_names, int num, bool reverse_flg = false, bool bmp_texmap_flg = false );

  //* ������ɥ������
  int createWindow( int width, int height, int x, int y, const char* windowname );

  //* �����롼�פ򳫻�
  static void start();

  //* ��å���˥ƥ�������򥻥å�
  static void setTexture( int width, int height, const unsigned char* image, int num_texture );

  static GlView* me;

protected:
  GLint mouse_button;
  GLint mouse_x;
  GLint mouse_y;
  float pitch;
  float yaw; 
  float translation_x;
  float translation_y;
  float scale;
  int win_width;
  int win_height;
  float center_x;
  float center_y;
  float center_z;

  //* ľ�����ΰ������
  static void dispBox( float sx, float gx, float sy, float gy, float sz, float gz, int line_width = 3, unsigned char r = 255, unsigned char g = 0, unsigned char b = 0 );

  //* ����������
  static void dispPoints( Obj& obj );

  //* ��å��������
  static void dispMesh( Obj& obj, bool mesh_flg = false );

  //* ��å�������� �����ϰϤΤ�
  static void dispMesh( Obj& obj, float sx, float gx, float sy, float gy, float sz, float gz, bool mesh_flg = false, bool reverse_flg = false );
  
  //* �ܥ����������
  static void dispVoxel( Voxel& voxel, bool cube_flg = false );
  
  //* �ܥ���������� �����ϰϤΤ�
  static void dispVoxel( Voxel& voxel, int sx, int rx, int sy, int ry, int sz, int rz, bool cube_flg = false );
  
  //* 3D�ǡ������礭�����������������褹��Ȥ�������˸Ƥ֤��
  void viewpoint_init( float x_min, float x_max, float y_min, float y_max, float z_min, float z_max );
  
  //* 3D�ǡ������礭�����������������褹��Ȥ���display�ؿ��κǽ�˸Ƥ֤��
  void display_init( float dis, bool local_view = false );

private:
  static void init( void );
  static void display_s( void ){ me->display(); }
  static void reshape_s ( int w, int h ){ me->reshape( w, h ); }
  static void keyboard_s( unsigned char key, int x, int y ){ me->keyboard( key, x, y ); }
  static void mouse_s( int button, int state, int x, int y ){ me->mouse( button, state, x, y ); }
  static void motion_s( int x, int y ){ me->motion( x, y ); }
  static void idle();

  virtual void display( void ) = 0;
  virtual void keyboard( unsigned char key, int x, int y ){}
  void mouse( int button, int state, int x, int y );
  void reshape ( int w, int h );
  void motion( int x, int y );
};

#endif
