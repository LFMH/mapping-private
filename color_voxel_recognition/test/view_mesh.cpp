#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <assert.h>
#include <unistd.h>

#include <color_voxel_recognition/glView.hpp>
#include <color_voxel_recognition/objFile.hpp>
#include <color_voxel_recognition/ppmFile.hpp>

/******************************************************/
/* メッシュを表示する                                    */
/* 複数のファイルのメッシュを同時に表示することも可能           */
/* その場合は対応するテクスチャ画像を正しい順番で並べることに注意 */
/******************************************************/

using namespace std;

static int GLUTwindow = 0;
Obj *object;
int obj_num;

class TestView : public GlView {
public:
  TestView() : mesh_flg(false) { me = this; }
  ~TestView(){ if( me!=NULL ) delete[] me; }
private:
  bool mesh_flg;
  void display( void );
  void keyboard (unsigned char key, int x, int y);
};

void TestView::display(void)
{
  display_init( 2, true );

  if( mesh_flg ){
    glColor3f(1.0,1.0,1.0);  
    glLineWidth( 1 );
  }
  for( int c=0; c<obj_num; c++ )
    dispMesh( object[ c ], mesh_flg );
  
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
  case 'm': // テクスチャなしメッシュモードとテクスチャありモードの切り替え
    if(mesh_flg) mesh_flg = false;
    else mesh_flg = true;
    break;

  case 'p':
    capture("gomi/tmp.png");
    break;
  }
}

int main(int argc, char **argv)
{

  if( (argc < 3) || (argc%2 != 1) ){
    printf("usage: %s [input1.obj] [input2.obj] ... [input1.ppm] [input2.ppm] ... \n",argv[0]);
    exit(1);
  }
  glutInit(&argc, argv);
  TestView testView;
  GLUTwindow = testView.createWindow( 640, 480, 100, 100, argv[0] );

  Ppm *ppm;
  obj_num = (argc - 1)/2;
  char **obj_names = new char*[ obj_num ];
  char **tex_names = new char*[ obj_num ];
  for(int i=0;i<obj_num;i++){
    obj_names[i] = argv[i+1];
    tex_names[i] = argv[i+1+obj_num];
  }
  printf("obj_num: %d\n",obj_num);
  object = new Obj[ obj_num ];
  ppm = new Ppm [ obj_num ];

  testView.readMesh( &object, &ppm, obj_names, tex_names, obj_num, true );
  testView.start();

  return 0;
}
