#include <GL/glut.h>

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <math.h>
#include <fstream>
#include <float.h>

#include <color_voxel_recognition/objFile.hpp>
#include <color_voxel_recognition/ppmFile.hpp>
#include <color_voxel_recognition/Param.hpp>
#include <color_voxel_recognition/glView.hpp>

/************************************************************/
/* 検出領域を目視で確認するためのプログラム （詳細は省略させてください） */
/* キーボード操作（環境表示のウィンドウがアクティブの状態で）          */
/*   'f': 前進                                              */
/*   'b': 後退                                              */
/*   'r': 右回転                                            */
/*   'l': 左回転                                            */
/*   'c': カメラ位置に順番に移動                                */
/*   'v': 標準入力で領域の開始地点の座標と領域サイズを入力           */
/*                           →この領域の前に移動しその領域を表示 */
/*   'm': テクスチャなしメッシュモードとテクスチャありモードの切り替え */
/*   'q'' Esc': 終了                                            */
/************************************************************/

using namespace std;

//#define MILLIMETER_MODE

const float STEP = 0.05; 
const int MAX_STEP_COUNT = 100;
const int VIEW_WAIT = 300;
const int WINDOW_W = 640;
const int WINDOW_H = 480;
const int IMAGE_WINDOW_W = 320;
const int IMAGE_WINDOW_H = 240;

float x_min = FLT_MAX, y_min = FLT_MAX, z_min = FLT_MAX;
float x_max = -FLT_MAX, y_max = -FLT_MAX, z_max = -FLT_MAX;
float x_middle, z_middle; // 物体領域の中心

unsigned char world_image[IMAGE_WINDOW_H][IMAGE_WINDOW_W][3];
Ppm *ppm;
Obj *object;
float voxel_size;

int WinID[2];
int image3D_num=1;

class TestView : public GlView {
public:
  bool trans_flg;
  bool mesh_flg;
  bool view_flg;
  bool view_last_flg;
  int view_count;
  bool camera_flg;
  int camera_num;
  float *camera_dx;
  float *camera_dy;
  float *camera_dz;
  float *camera_dx2;
  float *camera_dy2;
  float *camera_dz2;
  float dx;
  float dy;
  float dz;
  float dx2;
  float dy2;
  float dz2;
  float ang;
  int step_count;
  float step_dx, step_dy, step_dz;
  float step_dx2, step_dy2, step_dz2;
  int x_disp_min,y_disp_min,z_disp_min;
  int x_disp_range, y_disp_range,z_disp_range;

  TestView() : trans_flg(false), mesh_flg(false), view_flg(false), view_last_flg(false), view_count(0), 
	       camera_flg(false), camera_num(-1),  
	       camera_dx(NULL), camera_dy(NULL), camera_dz(NULL), camera_dx2(NULL), camera_dy2(NULL), camera_dz2(NULL), 
	       dx(0), dy(0), dz(0), dx2(0), dy2(0), dz2(0), ang(0) { me = this; }
  ~TestView(){ if( me!=NULL ) delete[] me; }
  void readCameraParam();

private:
  void display( void );
  void keyboard (unsigned char key, int x, int y);
  int find_camera_num();

};
  
void TestView::display( void ){
  //* 各モード毎に処理を行う

  if( view_last_flg ){
    glutSetWindow( WinID[0] );
    glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glDrawPixels( IMAGE_WINDOW_W, IMAGE_WINDOW_H, GL_RGB, GL_UNSIGNED_BYTE, &world_image[0][0][0] );
    glutSwapBuffers();
    glutSetWindow( WinID[1] );
  }
  
  glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  if(trans_flg){
    if( step_count > 0 ){
      dx += step_dx;
      dy += step_dy;
      dz += step_dz;
      dx2 += step_dx2;
      dy2 += step_dy2;
      dz2 += step_dz2;
      step_count--;
    }
    else{
      if(view_flg){
	view_last_flg = true;
	view_flg = false;
      }
      trans_flg = false;
      
      // 画像読み込み
      for( int j=0; j<IMAGE_WINDOW_H; j++ ){
	for( int i=0; i<IMAGE_WINDOW_W; i++ ){
	  world_image[ j ][ i ][ 0 ] = ppm[ camera_num ].ImageData()[ ( i + ( IMAGE_WINDOW_H - j - 1 ) * IMAGE_WINDOW_W ) * 3 ];
	  world_image[ j ][ i ][ 1 ] = ppm[ camera_num ].ImageData()[ ( i + ( IMAGE_WINDOW_H - j - 1 ) * IMAGE_WINDOW_W ) * 3 + 1 ];
	  world_image[ j ][ i ][ 2 ] = ppm[ camera_num ].ImageData()[ ( i + ( IMAGE_WINDOW_H - j - 1 ) * IMAGE_WINDOW_W ) * 3 + 2 ];
	}
      }
    }
  }
  else if(camera_flg){
    camera_num++;
    if(camera_num>=image3D_num)
      camera_num = 0;
    printf("camera_num: %d\n",camera_num);
    camera_flg = false;
    trans_flg = true;

    float dx_ = camera_dx[ camera_num ];
    float dy_ = camera_dy[ camera_num ];
    float dz_ = camera_dz[ camera_num ];
    float dx2_ = camera_dx2[ camera_num ];
    float dy2_ = camera_dy2[ camera_num ];
    float dz2_ = camera_dz2[ camera_num ];

    if( view_flg ){
      ang = atan2( x_middle - camera_dx[ camera_num ], z_middle - camera_dz[ camera_num ] );
      dx2_ = sin( ang );
      dz2_ = cos( ang );
    }
    else
      ang = atan2( dx2_, dz2_ );
    
    //* ステップ幅を決め、徐々に遷移させる（目的値を与える）
    dx_ -= 0.5*dx2_;
    dy_ -= 0.5*dy2_;
    dz_ -= 0.5*dz2_;
    dx2_ += dx_;
    dy2_ += dy_;
    dz2_ += dz_;
    float tmpx = dx_ - dx;
    float tmpy = dy_ - dy;
    float tmpz = dz_ - dz;
    float tmpx2 = dx2_ - dx2;
    float tmpy2 = dy2_ - dy2;
    float tmpz2 = dz2_ - dz2;
    float tmp_step = tmpx;
    if( tmp_step < 0 ) tmp_step = -tmp_step;
    if( ( tmpy > 0 ) && ( tmpy > tmp_step ) ) tmp_step = tmpy;
    else if( ( tmpy < 0 ) && ( -tmpy > tmp_step ) ) tmp_step = -tmpy;
    if( ( tmpz > 0 ) && ( tmpz > tmp_step ) ) tmp_step = tmpz;
    else if( ( tmpz < 0 ) && ( -tmpz > tmp_step ) ) tmp_step = -tmpz;
    if( ( tmpx2 > 0 ) && ( tmpx2 * 3 > tmp_step ) ) tmp_step = tmpx2 * 3;
    else if( ( tmpx2 < 0 ) && ( -tmpx2 * 3 > tmp_step ) ) tmp_step = -tmpx2 * 3;
    if( ( tmpy2 > 0 ) && ( tmpy2 * 3 > tmp_step ) ) tmp_step = tmpy2 * 3;
    else if( ( tmpy2 < 0 ) && ( -tmpy2 * 3 > tmp_step ) ) tmp_step = -tmpy2 * 3;
    if( ( tmpz2 > 0 ) && ( tmpz2 * 3 > tmp_step ) ) tmp_step = tmpz2 * 3;
    else if( ( tmpz2 < 0 ) && ( -tmpz2 * 3 > tmp_step ) ) tmp_step = -tmpz2 * 3;
    
    step_count = tmp_step / STEP;
    if( step_count > MAX_STEP_COUNT ) step_count = MAX_STEP_COUNT;
    
    if( step_count > 0 ){
      step_dx = tmpx / step_count;
      step_dy = tmpy / step_count;
      step_dz = tmpz / step_count;
      step_dx2 = tmpx2 / step_count;
      step_dy2 = tmpy2 / step_count;
      step_dz2 = tmpz2 / step_count;
    }
    else{
      step_dx = 0;
      step_dy = 0;
      step_dz = 0;
      step_dx2 = 0;
      step_dy2 = 0;
      step_dz2 = 0;
    }
    
    //* 一回分進める
    dx += step_dx;
    dy += step_dy;
    dz += step_dz;
    dx2 += step_dx2;
    dy2 += step_dy2;
    dz2 += step_dz2;
    step_count--;
  }
  else if(view_flg){
    camera_num = find_camera_num();
    if( camera_num == -1 ){
      printf("This area is not included in the scene.\n");
      camera_num = 0;
    }
    camera_flg = true;
  }
  else{
    dx2 = dx + sin(ang);
    dy2 = dy;
    dz2 = dz + cos(ang);
  }
  
  gluLookAt(dx, dy, dz, dx2, dy2, dz2, 0.0, 1.0, 0.0);
  
  glRotatef( -pitch, 1.0, 0.0, 0.0 );        
  glRotatef( -yaw, 0.0, 1.0, 0.0 );
  
  glPushMatrix();
  {
    glTranslatef(1.5,0.5,3.5);
    glTranslatef(-1.5,-0.5,-3.5);
    
    //* メッシュの描画
#ifdef MILLIMETER_MODE
    float sx = x_disp_min * voxel_size / 1000.0 + x_min;
    float gx = ( x_disp_min + x_disp_range + 1 ) * voxel_size / 1000.0 + x_min;
    float sy = y_disp_min * voxel_size / 1000.0 + y_min;
    float gy = ( y_disp_min + y_disp_range + 1 ) * voxel_size / 1000.0 + y_min;
    float sz = z_disp_min * voxel_size / 1000.0 + z_min;
    float gz = ( z_disp_min + z_disp_range + 1 ) * voxel_size / 1000.0 + z_min;
#else
    float sx = x_disp_min * voxel_size + x_min;
    float gx = ( x_disp_min + x_disp_range + 1 ) * voxel_size + x_min;
    float sy = y_disp_min * voxel_size + y_min;
    float gy = ( y_disp_min + y_disp_range + 1 ) * voxel_size + y_min;
    float sz = z_disp_min * voxel_size + z_min;
    float gz = ( z_disp_min + z_disp_range + 1 ) * voxel_size + z_min;
#endif
    dispBox( sx, gx, sy, gy, sz, gz, 3 );
    
    if( mesh_flg ){
      glDisable(GL_TEXTURE_2D);
      glColor3f(1.0,1.0,1.0);  
      glLineWidth( 1 );
    }
 
    if( view_last_flg ){
      if( view_count++ == VIEW_WAIT ){
	view_count = 0;
	view_flg = false;
	view_last_flg = false;
      }
      for( int c=0; c<image3D_num; c++ )
	dispMesh( object[ c ], sx, gx, sy, gy, sz, gz, mesh_flg );
    }
    else{
      for( int c=0; c<image3D_num; c++ )
	dispMesh( object[ c ], mesh_flg );
    }
  }
  glPopMatrix();
  glutSwapBuffers();
}

void TestView::keyboard (unsigned char key, int x, int y)
{
  switch (key) {
  case 'f': // 前に進む
    dx += 0.1 * sin(ang);
    dz += 0.1 * cos(ang);
    break;
  case 'b': // 後ろに進む
    dx -= 0.1 * sin(ang);
    dz -= 0.1 * cos(ang);
    break;
  case 'r': // 右に回転
    ang -= M_PI/180;
    break;
  case 'l': // 左に回転
    ang += M_PI/180;
    break;
  case 'c': // カメラ位置に、順番に移動
    camera_flg = true;
    break;
  case 'v': // 標準入力で、領域の開始地点の座標と領域サイズを入力すると、この領域の前に移動し、その領域を表示する
    if( view_last_flg ) view_last_flg = false;
    else{
      view_flg = true;
      char line[100];
      printf("x_disp_min | x_disp_range | y_disp_min | y_disp_range | z_disp_min | z_disp_range\n");
      fgets(line,sizeof(line),stdin);
      sscanf(line,"%d %d %d %d %d %d\n",&x_disp_min,&x_disp_range,&y_disp_min,&y_disp_range,&z_disp_min,&z_disp_range);
    }
    break;
  case 'm': // テクスチャなしメッシュモードとテクスチャありモードの切り替え
    if(mesh_flg) mesh_flg = false;
    else mesh_flg = true;
    break;
  case 27: // Esc 終了
  case 'q': // 終了
    glutDestroyWindow( WinID[0] );
    glutDestroyWindow( WinID[1] );
    exit(0);
    break;
  }
}

//* 表示領域の近くのカメラ位置を返す
int TestView::find_camera_num(){
  int num = -1;
  float dis;
#ifdef MILLIMETER_MODE
  x_middle = ( x_disp_min + x_disp_range / 2.0 ) * voxel_size / 1000.0 + x_min; 
  z_middle = ( z_disp_min + z_disp_range / 2.0 ) * voxel_size / 1000.0 + z_min; 
#else
  x_middle = ( x_disp_min + x_disp_range / 2.0 ) * voxel_size + x_min; 
  z_middle = ( z_disp_min + z_disp_range / 2.0 ) * voxel_size + z_min; 
#endif
  float dis_min = FLT_MAX;
  for(int c=0;c<image3D_num;c++){
    float x1 = x_middle - camera_dx[ c ];
    float z1 = z_middle - camera_dz[ c ];
    if( x1 * camera_dx2[ c ] +  z1 * camera_dz2[ c ] > 0 ){ // 内積が正
      float dis1 = x1 * x1 + z1 * z1;
      float dis2 =  camera_dx2[ c ]*camera_dx2[ c ] + camera_dz2[ c ]*camera_dz2[ c ];
      dis = dis1 - ( x1*camera_dx2[ c ] + z1*camera_dz2[ c ] ) * ( x1*camera_dx2[ c ] + z1*camera_dz2[ c ] ) / dis2;
      if( (( num == -1 ) || ( dis < dis_min )) && (dis>0) ){
 	dis_min = dis;
 	num = c;
       }
    }
  }
  return num;
}

void TestView::readCameraParam(){
  camera_dx = new float[ image3D_num ];
  camera_dy = new float[ image3D_num ];
  camera_dz = new float[ image3D_num ];
  camera_dx2 = new float[ image3D_num ];
  camera_dy2 = new float[ image3D_num ];
  camera_dz2 = new float[ image3D_num ];
  char tmpname[100];
  float tmpv;
  for( int c=0; c<image3D_num; c++ ){
    sprintf(tmpname,"scene/Camera/%03d.dat",c );  
    FILE *fp_camera = fopen(tmpname,"r");
    fscanf(fp_camera,"%f %f %f\n",camera_dx+c,camera_dy+c,camera_dz+c);
    fscanf(fp_camera,"%f %f %f %f %f %f %f %f %f\n",&tmpv,&tmpv,camera_dx2+c,&tmpv,&tmpv,camera_dy2+c,&tmpv,&tmpv,camera_dz2+c);
    fclose(fp_camera);
  }
}

int main(int argc, char** argv)
{
  if( argc != 2 ){
    cout<<"usage: "<<argv[0]<<" <scene_num>"<<endl;
    exit( EXIT_FAILURE );
  }

  glutInit(&argc, argv);
  TestView testView;
  testView.setWinSize( WINDOW_W, WINDOW_H );
  WinID[0] = testView.createWindow( IMAGE_WINDOW_W, IMAGE_WINDOW_H, WINDOW_W+20, 50, "Image" );
  WinID[1] = testView.createWindow( WINDOW_W, WINDOW_H, 0, 50, "3D Scene" );
   
  //* 各種データの読み込み
  int num_texture = -1;
  char tmpname[100];
  image3D_num = atoi(argv[1]);
  object = new Obj[ image3D_num ];
  ppm = new Ppm [ image3D_num ];
  for(int i=0; i<image3D_num; i++){
    sprintf(tmpname,"./scene/Obj_for_view/%03d.obj",i);
    object[i].readMesh( tmpname );
#ifdef MILLIMETER_MODE
    object[i].mm2m();
#endif
    if( object[i].x_min < x_min ) x_min = object[i].x_min;
    if( object[i].y_min < y_min ) y_min = object[i].y_min;
    if( object[i].z_min < z_min ) z_min = object[i].z_min;
    if( object[i].x_max > x_max ) x_max = object[i].x_max;
    if( object[i].y_max > y_max ) y_max = object[i].y_max;
    if( object[i].z_max > z_max ) z_max = object[i].z_max;
    sprintf(tmpname,"./scene/Textures/%03d.ppm",i);
    ppm[ i ].read( tmpname );
    object[i].setTextureID( ++num_texture );
    testView.setTexture( ppm[ i ].Width(), ppm[ i ].Height(), ppm[ i ].ImageData(), num_texture );
    printf("finished reading... %d\n",i);
  }
  cout<<"x_min: "<<x_min<<"  x_max: "<<x_max<<endl;
  cout<<"y_min: "<<y_min<<"  y_max: "<<y_max<<endl;
  cout<<"z_min: "<<z_min<<"  z_max: "<<z_max<<endl;

  //* ボクセルの一辺の長さ（mm）の読み込み
  voxel_size = Param::readVoxelSize();

  testView.readCameraParam();
  testView.start();
   
  return 0;
}
