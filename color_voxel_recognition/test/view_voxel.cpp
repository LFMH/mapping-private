#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

#include <color_voxel_recognition/glView.hpp>
#include <color_voxel_recognition/Voxel.hpp>
#include "../param/FILE_MODE"

/***************************************************************/
/* ボクセルを表示する                                              */
/* コマンドライン引数で sx rx sy ry sz rz （開始点座標と幅）を指定すると */
/* その範囲内のみを表示する                                         */
/***************************************************************/

using namespace std;

#define CUBE
#ifdef CUBE
const bool cube_flg = true;
#else
const bool cube_flg = false;
#endif
static int GLUTwindow = 0;

Voxel voxel( ASCII_MODE_V );

class TestView : public GlView {
public:
  TestView() : sx(-1) { me = this; }
  ~TestView(){ if( me!=NULL ) delete[] me; }
  int sx, rx, sy, ry, sz, rz;

private:
  void display( void );
  void keyboard (unsigned char key, int x, int y);
};

void TestView::display(void)
{
  display_init( 3 );

  if( sx == -1 )
    dispVoxel( voxel, cube_flg );
  else
    dispVoxel( voxel, sx, rx, sy, ry, sz, rz, cube_flg );
  
  glutSwapBuffers();
}    

void TestView::keyboard(unsigned char key, int x, int y)
{
  switch (key) {
  case 'Q':
  case 'q':
  case 27: // ESCAPE
    // Destroy window 
    glutDestroyWindow(GLUTwindow);
    exit(0);
    break;

  case 'p':
    capture("gomi/tmp.png");
    break;

  case 'r':
    {
      char line[100];
      int x_min,x_max,y_min,y_max,z_min,z_max;
      cout << "x_min | x_max | y_min | y_max | z_min | z_max" << endl;
      fgets(line,sizeof(line),stdin);
      sscanf(line,"%d %d %d %d %d %d\n",&x_min,&x_max,&y_min,&y_max,&z_min,&z_max);
      float dx = x_max - x_min;
      float dy = y_max - y_min;
      float dz = z_max - z_min;
      scale = 2.0 / sqrt(dx*dx + dy*dy + dz*dz);
      center_x = 0.5 * (x_min + x_max);
      center_y = 0.5 * (y_min + y_max);
      center_z = 0.5 * (z_min + z_max);
    }
    break;
  }
}

int main(int argc, char **argv)
{

  if((argc!=2)&&(argc!=8)){
    cout<<"usage: "<<argv[0]<<" [voxel_filename]"<<endl;
    cout<<"   or"<<endl;
    cout<<"usage: "<<argv[0]<<" [voxel_filename] sx rx sy ry sz rz"<<endl;
    exit(0);
  }
  glutInit(&argc, argv);
  TestView testView;

  if( argc==8 ){
    testView.sx = atoi(argv[2]);
    testView.rx = atoi(argv[3]);
    testView.sy = atoi(argv[4]);
    testView.ry = atoi(argv[5]);
    testView.sz = atoi(argv[6]);
    testView.rz = atoi(argv[7]);
  }

  GLUTwindow = testView.createWindow( 640, 480, 100, 100, argv[0] );
  if( argc==2 )
    testView.readVoxel( voxel, argv[1] );
  else
    testView.readVoxel( voxel, argv[1], testView.sx, testView.rx, testView.sy, testView.ry, testView.sz, testView.rz );
  testView.start();
  return 0;
}
