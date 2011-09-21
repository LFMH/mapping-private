#include <stdlib.h>
//#include <stdio.h>
#include <fstream>
#include <iostream>
#include <string>
/* Ensure we are using opengl's core profile only */
#define GL3_PROTOTYPES 1
#include <GL3/gl3.h>
//#include <GL/gl.h>
#include <GL/freeglut.h>
#define GL_GLEXT_PROTOTYPES

#include <GL/glext.h>
#include <stdio.h>
#include <math.h>
 
#define PROGRAM_NAME "Tutorial2"

#include <realtime_perception/shader_wrapper.h>

void display ()
{
  static realtime_perception::ShaderWrapper shader = realtime_perception::ShaderWrapper::fromFiles ("include/shaders/test1.vert", "include/shaders/test1.frag");
  glClearColor(0.0, 0.0, 1.0, 1.0);
  glClear(GL_COLOR_BUFFER_BIT);
  glColor4f(1,1,1,1);
  glTranslatef (0,0,-1);
  
  shader ();

  glutSolidCube (1.0);
  glutSwapBuffers ();
  glutPostRedisplay ();
}

void glInit (int w, int h)
{
  // set up our OpenGL state
  glEnable(GL_TEXTURE_3D);  // enable 3d texturing
  glEnable(GL_DEPTH_TEST);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // our texture colors will replace the untextured colors
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glColor4d(0.0, 0.0, 0.0, 1.0);
  glPointSize(3.0);
  
  glShadeModel(GL_FLAT);
  
  glViewport (0, 0, (GLsizei) 640, (GLsizei) 480);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90, 4.0/3.0, 0.1, 10.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
}
 
/* Our program's entry point */
int main(int argc, char *argv[])
{
  glutInit(&argc, argv);
  
  glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize (640, 480);
  glutInitWindowPosition(80, 80);
  glutCreateWindow ("shader_test");
  
  glutDisplayFunc (display);
  
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  
 // // VERY IMPORTANT:
 // // this line loads the address of the glTexImage3D function into the function pointer of the same name.
 // // glTexImage3D is not implemented in the standard GL libraries and must be loaded dynamically at run time,
 // // the environment the program is being run in MAY OR MAY NOT support it, if not we'll get back a NULL pointer.
 // // this is necessary to use any OpenGL function declared in the glext.h header file
 // // the Pointer to FunctioN ... PROC types are declared in the same header file with a type appropriate to the function name
 // glTexImage3D = (PFNGLTEXIMAGE3DPROC) wglGetProcAddress("glTexImage3D");
 // if (glTexImage3D == NULL) {
 //   printf("Error in line %d: Couldn't load glTexImage3D function. Aborting.\n", __LINE__);
 //   return -1;
 // }
  
  glInit (640, 480);
  glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);

GLuint vao, vbo[2]; /* Create handles for our Vertex Array Object and two Vertex Buffer Objects */

/* We're going to create a simple diamond made from lines */
const GLfloat diamond[4][2] = {
{  0.0,  1.0  }, /* Top point */
{  1.0,  0.0  }, /* Right point */
{  0.0, -1.0  }, /* Bottom point */
{ -1.0,  0.0  } }; /* Left point */

const GLfloat colors[4][3] = {
{  1.0,  0.0,  0.0  }, /* Red */
{  0.0,  1.0,  0.0  }, /* Green */
{  0.0,  0.0,  1.0  }, /* Blue */
{  1.0,  1.0,  1.0  } }; /* White */

/* Allocate and assign a Vertex Array Object to our handle */
glGenVertexArrays(1, &vao);

/* Bind our Vertex Array Object as the current used object */
glBindVertexArray(vao);

/* Allocate and assign two Vertex Buffer Objects to our handle */
glGenBuffers(2, vbo);

/* Bind our first VBO as being the active buffer and storing vertex attributes (coordinates) */
glBindBuffer(GL_ARRAY_BUFFER, vbo[0]);

/* Copy the vertex data from diamond to our buffer */
/* 8 * sizeof(GLfloat) is the size of the diamond array, since it contains 8 GLfloat values */
glBufferData(GL_ARRAY_BUFFER, 8 * sizeof(GLfloat), diamond, GL_STATIC_DRAW);

/* Specify that our coordinate data is going into attribute index 0, and contains two floats per vertex */
glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 0, 0);

/* Enable attribute index 0 as being used */
glEnableVertexAttribArray(0);

/* Bind our second VBO as being the active buffer and storing vertex attributes (colors) */
glBindBuffer(GL_ARRAY_BUFFER, vbo[1]);

/* Copy the color data from colors to our buffer */
/* 12 * sizeof(GLfloat) is the size of the colors array, since it contains 12 GLfloat values */
glBufferData(GL_ARRAY_BUFFER, 12 * sizeof(GLfloat), colors, GL_STATIC_DRAW);

/* Specify that our color data is going into attribute index 1, and contains three floats per vertex */
glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, 0);

/* Enable attribute index 1 as being used */
glEnableVertexAttribArray(1);

  /* Compile, Link and Load shaders. */
  glutMainLoop ();

  return 0;
}

//////////////////////////////////////////////////////////
// lalala
//  int windownumber;

//  glutPostRedisplay();
//  glutSwapBuffers();


