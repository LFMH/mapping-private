#include "color_voxel_recognition/glView.hpp"
#include <math.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>

using namespace std;

GlView* GlView::me = 0;

GlView::GlView() : 
  mouse_button(-1),
  mouse_x(0),
  mouse_y(0),
  pitch(0),
  yaw(0),
  translation_x(0),
  translation_y(0),
  scale(1),
  win_width(0),
  win_height(0),
  center_x(0),
  center_y(0),
  center_z(0) { me = this; }

//*********************
//* 現在の画面の画像を保存
void GlView::capture( const char* filename ){
  FILE *fp_w = NULL;
  if (( fp_w = fopen( filename, "w" ) ) == NULL) {
    cerr << "cannot open the file: " << filename << endl;
    return;
  }
  
  GLubyte *buf = ( GLubyte * )malloc( sizeof( unsigned char ) * win_width * win_height * 3 );
  glReadPixels( 0, 0, win_width, win_height, GL_RGB, GL_UNSIGNED_BYTE, buf );
  fprintf( fp_w, "P6\n" );
  fprintf( fp_w, "%d %d\n", win_width, win_height );
  fprintf( fp_w, "255\n" );
  for ( int i = 0; i < win_height; i++ ) {
    for( int j = 0; j < win_width; j++ ) {
      fprintf( fp_w, "%c", buf[ ( (win_height-1-i) * win_width + j ) * 3 ] );
      fprintf( fp_w, "%c", buf[ ( (win_height-1-i) * win_width + j ) * 3 + 1] );
      fprintf( fp_w, "%c", buf[ ( (win_height-1-i) * win_width + j ) * 3 + 2] );
    }
  }
  fclose( fp_w );
  free( buf );
  cout << "image captured." << endl;
}    

//***************************
//* 描画のためにボクセルを読み込み 
void GlView::readVoxel( Voxel& voxel, const char* filename ){
  voxel.readVoxel_normal( filename );
  int x_min,x_max,y_min,y_max,z_min,z_max;
  voxel.getMinMax( x_min, x_max, y_min, y_max, z_min, z_max );
  viewpoint_init( x_min, x_max, y_min, y_max, z_min, z_max );

  cout << "x_min: "<< x_min << "  x_max: " << x_max << endl;
  cout << "y_min: "<< y_min << "  y_max: " << y_max << endl;
  cout << "z_min: "<< z_min << "  z_max: " << z_max << endl;
}

//**************************************
//* 描画のためにボクセルを読み込み ある範囲のみ
void GlView::readVoxel( Voxel& voxel, const char* filename, int sx, int rx, int sy, int ry, int sz, int rz ){
  voxel.readVoxel_normal( filename );
  viewpoint_init( sx, sx+rx, sy, sy+ry, sz, sz+rz );
}

//***************************
//* 描画のためにメッシュを読み込み
void GlView::readMesh( Obj** obj, Ppm** ppm, char **obj_names, char **tex_names, int num, bool reverse_flg, bool bmp_texmap_flg ){
  float x_min = FLT_MAX;
  float y_min = FLT_MAX;
  float z_min = FLT_MAX;
  float x_max = -FLT_MAX;
  float y_max = -FLT_MAX;
  float z_max = -FLT_MAX;

  int num_texture = -1;
  for( int i=0; i<num; i++ ){
    (*obj)[ i ].readMesh( obj_names[ i ], bmp_texmap_flg );

    if( reverse_flg ){
      //* xとyの符号を反転させる
      for( int v = 0; v < (*obj)[ i ].v_num; v++ ){
	(*obj)[ i ].vertices[ v ].x = - (*obj)[ i ].vertices[ v ].x;
	(*obj)[ i ].vertices[ v ].z = - (*obj)[ i ].vertices[ v ].z;
      }
    }
    if( (*obj)[i].x_min < x_min ) x_min = (*obj)[i].x_min;
    if( (*obj)[i].y_min < y_min ) y_min = (*obj)[i].y_min;
    if( (*obj)[i].z_min < z_min ) z_min = (*obj)[i].z_min;
    if( (*obj)[i].x_max > x_max ) x_max = (*obj)[i].x_max;
    if( (*obj)[i].y_max > y_max ) y_max = (*obj)[i].y_max;
    if( (*obj)[i].z_max > z_max ) z_max = (*obj)[i].z_max;
    (*ppm)[ i ].read( tex_names[ i ] );
    (*obj)[i].setTextureID( ++num_texture );
    setTexture( (*ppm)[ i ].Width(), (*ppm)[ i ].Height(), (*ppm)[ i ].ImageData(), num_texture );    
  }

  viewpoint_init( x_min, x_max, y_min, y_max, z_min, z_max );

  cout<<"x_min: "<<x_min<<"  x_max: "<<x_max<<endl;
  cout<<"y_min: "<<y_min<<"  y_max: "<<y_max<<endl;
  cout<<"z_min: "<<z_min<<"  z_max: "<<z_max<<endl;
}

//*****************
//* ウィンドウを作成
int GlView::createWindow( int width, int height, int x, int y, const char* windowname ){
  if( ( win_width == 0 ) || ( win_height == 0 ) ){
    win_width = width;
    win_height = height;
  }
  else if( ( win_width != width ) || ( win_height != height ) )
    cerr << "Warning (in GlView::createWindow): win_width or win_height is different from this window's size(" << width << "," << height << ")" << endl;
  glutInitDisplayMode ( GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH );
  glutInitWindowSize (width, height); 
  glutInitWindowPosition (x, y);
  return glutCreateWindow ( windowname );
}

//****************
//* 処理ループを開始
void GlView::start(){
  init();
  glutDisplayFunc(display_s);
  glutReshapeFunc(reshape_s);
  glutKeyboardFunc(keyboard_s);
  glutIdleFunc(idle);
  glutMouseFunc(mouse_s);
  glutMotionFunc(motion_s);
  glutMainLoop();
}

//*************************
//* メッシュにテクスチャをセット
void GlView::setTexture( int width, int height, const unsigned char* image, int num_texture ){
  glBindTexture(GL_TEXTURE_2D, num_texture);
  
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_NEAREST);
  
  glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
  
  glTexImage2D(GL_TEXTURE_2D, 0, 3, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, image);
  gluBuild2DMipmaps(GL_TEXTURE_2D, 3, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);
}

//****************//
//* protected関数 *//
//****************//

//****************
//* 直方体領域の描画
void GlView::dispBox( float sx, float gx, float sy, float gy, float sz, float gz, int line_width, unsigned char r, unsigned char g, unsigned char b ){
  glDisable(GL_TEXTURE_2D);
  glColor3ub( r, g, b );
  glLineWidth( line_width );
  glBegin( GL_LINE_LOOP );
  glVertex3f( sx,sy,sz );
  glVertex3f( gx,sy,sz );
  glVertex3f( gx,gy,sz );
  glVertex3f( sx,gy,sz );
  glEnd();
  glBegin( GL_LINE_LOOP );
  glVertex3f( sx,sy,gz );
  glVertex3f( gx,sy,gz );
  glVertex3f( gx,gy,gz );
  glVertex3f( sx,gy,gz );
  glEnd();
  glBegin( GL_LINE_LOOP );
  glVertex3f( sx,sy,sz );
  glVertex3f( sx,sy,gz );
  glVertex3f( sx,gy,gz );
  glVertex3f( sx,gy,sz );
  glEnd();
  glBegin( GL_LINE_LOOP );
  glVertex3f( gx,sy,sz );
  glVertex3f( gx,sy,gz );
  glVertex3f( gx,gy,gz );
  glVertex3f( gx,gy,sz );
  glEnd();
  glBegin( GL_LINE_LOOP );
  glVertex3f( sx,sy,sz );
  glVertex3f( gx,sy,sz );
  glVertex3f( gx,sy,gz );
  glVertex3f( sx,sy,gz );
  glEnd();
  glBegin( GL_LINE_LOOP );
  glVertex3f( sx,gy,sz );
  glVertex3f( gx,gy,sz );
  glVertex3f( gx,gy,gz );
  glVertex3f( sx,gy,gz );
  glEnd();
  glEnable(GL_TEXTURE_2D);
}

//***********
//* 点群の描画
void GlView::dispPoints( Obj& obj ){
  glBegin(GL_POINTS); 
  for( int v=0; v<obj.v_num; v++)
    glVertex3f( obj.vertices[ v ].x, obj.vertices[ v ].y, obj.vertices[ v ].z);
  glEnd();
}

//**************
//* メッシュの描画
void GlView::dispMesh( Obj& obj, bool mesh_flg ){
  if( mesh_flg ){
    for( int f=0; f<obj.f_num; f++){
      glBegin(GL_LINE_LOOP); 
      glVertex3f( obj.vertices[ obj.faces[ f ].v1 ].x, obj.vertices[ obj.faces[ f ].v1 ].y, obj.vertices[ obj.faces[ f ].v1 ].z);
      glVertex3f( obj.vertices[ obj.faces[ f ].v2 ].x, obj.vertices[ obj.faces[ f ].v2 ].y, obj.vertices[ obj.faces[ f ].v2 ].z);
      glVertex3f( obj.vertices[ obj.faces[ f ].v3 ].x, obj.vertices[ obj.faces[ f ].v3 ].y, obj.vertices[ obj.faces[ f ].v3 ].z);
      glEnd();
    }
  }
  else{
    glBindTexture(GL_TEXTURE_2D, obj.texture_id );
    glBegin(GL_TRIANGLES);
    for( int f=0; f<obj.f_num; f++){
      glTexCoord2f( obj.verticesT[ obj.faces[ f ].vt1 ].x, obj.verticesT[ obj.faces[ f ].vt1 ].y);
      glVertex3f(   obj.vertices[  obj.faces[ f ].v1  ].x, obj.vertices[ obj.faces[ f ].v1 ].y, obj.vertices[ obj.faces[ f ].v1 ].z);
      glTexCoord2f( obj.verticesT[ obj.faces[ f ].vt2 ].x, obj.verticesT[ obj.faces[ f ].vt2 ].y);
      glVertex3f(   obj.vertices[  obj.faces[ f ].v2  ].x, obj.vertices[ obj.faces[ f ].v2 ].y, obj.vertices[ obj.faces[ f ].v2 ].z);
      glTexCoord2f( obj.verticesT[ obj.faces[ f ].vt3 ].x, obj.verticesT[ obj.faces[ f ].vt3 ].y);
      glVertex3f(   obj.vertices[  obj.faces[ f ].v3  ].x, obj.vertices[ obj.faces[ f ].v3 ].y, obj.vertices[ obj.faces[ f ].v3 ].z);
    }
    glEnd();
  }
}

//*************************
//* メッシュの描画 ある範囲のみ
void GlView::dispMesh( Obj& obj, float sx, float gx, float sy, float gy, float sz, float gz, bool mesh_flg, bool reverse_flg ){
  if( mesh_flg ){
    for( int f=0; f<obj.f_num; f++){
      bool inside = ( ( ( sx <= obj.vertices[ obj.faces[ f ].v1 ].x ) && ( obj.vertices[ obj.faces[ f ].v1 ].x <= gx ) && 
			( sy <= obj.vertices[ obj.faces[ f ].v1 ].y ) && ( obj.vertices[ obj.faces[ f ].v1 ].y <= gy ) &&
			( sz <= obj.vertices[ obj.faces[ f ].v1 ].z ) && ( obj.vertices[ obj.faces[ f ].v1 ].z <= gz ) ) || 
		      ( ( sx <= obj.vertices[ obj.faces[ f ].v2 ].x ) && ( obj.vertices[ obj.faces[ f ].v2 ].x <= gx ) && 
			( sy <= obj.vertices[ obj.faces[ f ].v2 ].y ) && ( obj.vertices[ obj.faces[ f ].v2 ].y <= gy ) &&
			( sz <= obj.vertices[ obj.faces[ f ].v2 ].z ) && ( obj.vertices[ obj.faces[ f ].v2 ].z <= gz ) ) || 
		      ( ( sx <= obj.vertices[ obj.faces[ f ].v3 ].x ) && ( obj.vertices[ obj.faces[ f ].v3 ].x <= gx ) && 
			( sy <= obj.vertices[ obj.faces[ f ].v3 ].y ) && ( obj.vertices[ obj.faces[ f ].v3 ].y <= gy ) &&
			( sz <= obj.vertices[ obj.faces[ f ].v3 ].z ) && ( obj.vertices[ obj.faces[ f ].v3 ].z <= gz ) ) );
      if( ( inside && !reverse_flg ) || ( !inside && reverse_flg ) ){
	glBegin(GL_LINE_LOOP); 
	glVertex3f( obj.vertices[ obj.faces[ f ].v1 ].x, obj.vertices[ obj.faces[ f ].v1 ].y, obj.vertices[ obj.faces[ f ].v1 ].z);
	glVertex3f( obj.vertices[ obj.faces[ f ].v2 ].x, obj.vertices[ obj.faces[ f ].v2 ].y, obj.vertices[ obj.faces[ f ].v2 ].z);
	glVertex3f( obj.vertices[ obj.faces[ f ].v3 ].x, obj.vertices[ obj.faces[ f ].v3 ].y, obj.vertices[ obj.faces[ f ].v3 ].z);
	glEnd();
      }
    }
  }
  else{
    glBindTexture(GL_TEXTURE_2D, obj.texture_id );
    glBegin(GL_TRIANGLES);
    for( int f=0; f<obj.f_num; f++){
      bool inside = ( ( ( sx <= obj.vertices[ obj.faces[ f ].v1 ].x ) && ( obj.vertices[ obj.faces[ f ].v1 ].x <= gx ) && 
			( sy <= obj.vertices[ obj.faces[ f ].v1 ].y ) && ( obj.vertices[ obj.faces[ f ].v1 ].y <= gy ) &&
			( sz <= obj.vertices[ obj.faces[ f ].v1 ].z ) && ( obj.vertices[ obj.faces[ f ].v1 ].z <= gz ) ) || 
		      ( ( sx <= obj.vertices[ obj.faces[ f ].v2 ].x ) && ( obj.vertices[ obj.faces[ f ].v2 ].x <= gx ) && 
			( sy <= obj.vertices[ obj.faces[ f ].v2 ].y ) && ( obj.vertices[ obj.faces[ f ].v2 ].y <= gy ) &&
			( sz <= obj.vertices[ obj.faces[ f ].v2 ].z ) && ( obj.vertices[ obj.faces[ f ].v2 ].z <= gz ) ) || 
		      ( ( sx <= obj.vertices[ obj.faces[ f ].v3 ].x ) && ( obj.vertices[ obj.faces[ f ].v3 ].x <= gx ) && 
			( sy <= obj.vertices[ obj.faces[ f ].v3 ].y ) && ( obj.vertices[ obj.faces[ f ].v3 ].y <= gy ) &&
			( sz <= obj.vertices[ obj.faces[ f ].v3 ].z ) && ( obj.vertices[ obj.faces[ f ].v3 ].z <= gz ) ) );
      if( ( inside && !reverse_flg ) || ( !inside && reverse_flg ) ){
	glTexCoord2f( obj.verticesT[ obj.faces[ f ].vt1 ].x, obj.verticesT[ obj.faces[ f ].vt1 ].y);
	glVertex3f(   obj.vertices[  obj.faces[ f ].v1  ].x, obj.vertices[ obj.faces[ f ].v1 ].y, obj.vertices[ obj.faces[ f ].v1 ].z);
	glTexCoord2f( obj.verticesT[ obj.faces[ f ].vt2 ].x, obj.verticesT[ obj.faces[ f ].vt2 ].y);
	glVertex3f(   obj.vertices[  obj.faces[ f ].v2  ].x, obj.vertices[ obj.faces[ f ].v2 ].y, obj.vertices[ obj.faces[ f ].v2 ].z);
	glTexCoord2f( obj.verticesT[ obj.faces[ f ].vt3 ].x, obj.verticesT[ obj.faces[ f ].vt3 ].y);
	glVertex3f(   obj.vertices[  obj.faces[ f ].v3  ].x, obj.vertices[ obj.faces[ f ].v3 ].y, obj.vertices[ obj.faces[ f ].v3 ].z);
      }
    }
    glEnd();
  }
}

//**************
//* ボクセルの描画
void GlView::dispVoxel( Voxel& voxel, bool cube_flg ){
  const int xsize = voxel.Xsize();
  const int ysize = voxel.Ysize();
  const int zsize = voxel.Zsize();
  for(int i=0;i<xsize;i++){
    for(int j=0;j<ysize;j++){
      for(int k=0;k<zsize;k++){
	if( voxel.exist( i, j, k ) ){
	  glColor3ub( voxel.red( i, j, k ), voxel.green( i, j, k ), voxel.blue( i, j, k ) );
	  glTranslatef(-i, j, -k);
	  if( cube_flg )
	    glutSolidCube( 1.0 );
	  else{
	    glBegin(GL_POLYGON);
	    glVertex3f(0.5,0.5,0.5);
	    glVertex3f(-0.5,0.5,0.5);
	    glVertex3f(-0.5,-0.5,0.5);
	    glVertex3f(0.5,-0.5,0.5);
	    glEnd();
	    glBegin(GL_POLYGON);
	    glVertex3f(0.5,0.5,0.5);
	    glVertex3f(-0.5,0.5,0.5);
	    glVertex3f(-0.5,0.5,-0.5);
	    glVertex3f(0.5,0.5,-0.5);
	    glEnd();
	    glBegin(GL_POLYGON);
	    glVertex3f(0.5,0.5,0.5);
	    glVertex3f(0.5,-0.5,0.5);
	    glVertex3f(0.5,-0.5,-0.5);
	    glVertex3f(0.5,0.5,-0.5);
	    glEnd();
	  }
	  glTranslatef(i, -j, k);
	}
      }
    }
  }  
}

//*************************
//* ボクセルの描画 ある範囲のみ
void GlView::dispVoxel( Voxel& voxel, int sx, int rx, int sy, int ry, int sz, int rz, bool cube_flg ){
  for(int i=0;i<rx;i++){
    for(int j=0;j<ry;j++){
      for(int k=0;k<rz;k++){
	if( voxel.exist( i + sx, j + sy, k + sz ) ){
	  glColor3ub( voxel.red( i + sx, j + sy, k + sz ), voxel.green( i + sx, j + sy, k + sz ), voxel.blue( i + sx, j + sy, k + sz ) );
	  glTranslatef(-i-sx, j+sy, -k-sz);
	  if( cube_flg )
	    glutSolidCube( 1.0 );
	  else{
	    glBegin(GL_POLYGON);
	    glVertex3f(0.5,0.5,0.5);
	    glVertex3f(-0.5,0.5,0.5);
	    glVertex3f(-0.5,-0.5,0.5);
	    glVertex3f(0.5,-0.5,0.5);
	    glEnd();
	    glBegin(GL_POLYGON);
	    glVertex3f(0.5,0.5,0.5);
	    glVertex3f(-0.5,0.5,0.5);
	    glVertex3f(-0.5,0.5,-0.5);
	    glVertex3f(0.5,0.5,-0.5);
	    glEnd();
	    glBegin(GL_POLYGON);
	    glVertex3f(0.5,0.5,0.5);
	    glVertex3f(0.5,-0.5,0.5);
	    glVertex3f(0.5,-0.5,-0.5);
	    glVertex3f(0.5,0.5,-0.5);
	    glEnd();
	  }
	  glTranslatef(i+sx, -j-sy, k+sz);
	}
      }
    }
  }  
}

//************************************************
//* 3Dデータの大きさを正規化して描画するとき、初期に呼ぶもの
void GlView::viewpoint_init( float x_min, float x_max, float y_min, float y_max, float z_min, float z_max ){
  const float dx = x_max - x_min;
  const float dy = y_max - y_min;
  const float dz = z_max - z_min;
  scale = 2.0 / sqrt(dx*dx + dy*dy + dz*dz);
  
  center_x = 0.5 * (x_min + x_max);
  center_y = 0.5 * (y_min + y_max);
  center_z = 0.5 * (z_min + z_max);
}

//************************************************************
//* 3Dデータの大きさを正規化して描画するとき、display関数の最初に呼ぶもの
void GlView::display_init( float dis, bool local_view ){
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  glTranslatef(translation_x, translation_y, -dis);
  glScalef(scale, scale, scale);
  glRotatef(pitch, 1.0, 0.0, 0.0);
  glRotatef(yaw, 0.0, 0.0, 1.0);
  if( local_view )
    glTranslatef(center_x, -center_y, center_z);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

//**************//
//* private関数 *//
//**************//

void GlView::init(void) 
{
  glClearColor (0.0, 0.0, 0.5, 0.0);
  glClearDepth( 10.0 );
  glEnable( GL_DEPTH_TEST );
  glDepthFunc( GL_LESS );
  //glShadeModel (GL_SMOOTH);
  glShadeModel(GL_FLAT);
  //glEnable(GL_AUTO_NORMAL);
  glEnable(GL_NORMALIZE);
  glEnable(GL_TEXTURE_2D);
}

void GlView::reshape (int w, int h)
{
  glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
  glMatrixMode (GL_PROJECTION);
  glLoadIdentity ();
  gluPerspective(45.0, (GLfloat) w/(GLfloat) h, 0.3, 20.0);
  win_width = w;
  win_height = h;
}

void GlView::mouse(int button, int state, int x, int y)
{
  y = win_height - y;

  mouse_button = button;
  mouse_x = x;
  mouse_y = y;
  if(state == GLUT_UP){
    mouse_button = -1;
  }
  glutPostRedisplay();
}

void GlView::motion(int x, int y)
{
  y = win_height - y;

  if( ( x == mouse_x ) && ( y == mouse_y ) )
    return;
  switch(mouse_button){
  case GLUT_LEFT_BUTTON:
    yaw += (GLfloat) (x - mouse_x) * 0.5;
    pitch -= (GLfloat) (y - mouse_y) * 0.5;
    break;
  case GLUT_RIGHT_BUTTON:
    translation_x += 2.0 * (GLfloat) ( x - mouse_x ) / (GLfloat) win_width;
    translation_y += 2.0 * (GLfloat) ( y - mouse_y ) / (GLfloat) win_height;
    break;
  case GLUT_MIDDLE_BUTTON:
    scale *= exp( 2.0 * (GLfloat) ( x - mouse_x ) / (GLfloat) win_width );
    break;
  default:
    break;
  }

  mouse_x = x;
  mouse_y = y;

  glutPostRedisplay();
}

void GlView::idle()
{
  glutPostRedisplay();
}
