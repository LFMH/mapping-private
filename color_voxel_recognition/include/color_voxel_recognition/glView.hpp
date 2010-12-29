#ifndef MY_GLVIEW_HPP
#define MY_GLVIEW_HPP

#include <GL/glut.h>
#include "objFile.hpp"
#include "ppmFile.hpp"
#include "Voxel.hpp"

/****************************************************/
/* OpenGLでボクセルやメッシュを表示する                   */
/* 使うときは このクラスを継承して新たにクラスを作り、        */
/*   display()＜必須＞と、keyboard()＜任意＞を定義すること */
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

  //* 現在の画面の画像を保存
  void capture( const char* filename );

  //* 描画のためにボクセルを読み込み 
  void readVoxel( Voxel& voxel, const char* filename );

  //* 描画のためにボクセルを読み込み ある範囲のみ
  void readVoxel( Voxel& voxel, const char* filename, int sx, int rx, int sy, int ry, int sz, int rz );

  //* 描画のためにメッシュを読み込み
  void readMesh( Obj** object, Ppm** ppm, char **obj_names, char **tex_names, int num, bool reverse_flg = false, bool bmp_texmap_flg = false );

  //* ウィンドウを作成
  int createWindow( int width, int height, int x, int y, const char* windowname );

  //* 処理ループを開始
  static void start();

  //* メッシュにテクスチャをセット
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

  //* 直方体領域の描画
  static void dispBox( float sx, float gx, float sy, float gy, float sz, float gz, int line_width = 3, unsigned char r = 255, unsigned char g = 0, unsigned char b = 0 );

  //* 点群の描画
  static void dispPoints( Obj& obj );

  //* メッシュの描画
  static void dispMesh( Obj& obj, bool mesh_flg = false );

  //* メッシュの描画 ある範囲のみ
  static void dispMesh( Obj& obj, float sx, float gx, float sy, float gy, float sz, float gz, bool mesh_flg = false, bool reverse_flg = false );
  
  //* ボクセルの描画
  static void dispVoxel( Voxel& voxel, bool cube_flg = false );
  
  //* ボクセルの描画 ある範囲のみ
  static void dispVoxel( Voxel& voxel, int sx, int rx, int sy, int ry, int sz, int rz, bool cube_flg = false );
  
  //* 3Dデータの大きさを正規化して描画するとき、初期に呼ぶもの
  void viewpoint_init( float x_min, float x_max, float y_min, float y_max, float z_min, float z_max );
  
  //* 3Dデータの大きさを正規化して描画するとき、display関数の最初に呼ぶもの
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
