// 3D Texturing Tutorial.cpp : Defines the entry point for the application.
// Doug Sheets (doug.sheets@gmail.com)
// 11-9-04
//
// GLUT conversion made by Ciro Durán (ciro.duran@gmail.com)
// Date: 27-III-2005

#include <GL/glut.h>
#include <GL/glext.h>
#include <stdio.h>
#include <math.h>

#include "3dtex.h"

unsigned int fpsCurrent = 0;
unsigned int fpsCount = 0;

unsigned long tick;
PFNGLTEXIMAGE3DPROC glTexImage3D;
int windownumber;

unsigned int texname;
unsigned char *tex = NULL;

// these define a square in the X-Y plane clockwise order from the lower left,
GLdouble verts[4][3] = { { -1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0}, {1.0, 1.0, 0.0}, {1.0, -1.0, 0.0} };
GLdouble centervert[3] = { 0.0, 0.0, 0.0 };

int main(int argc, char **argv) {
  glutInit(&argc, argv);
  
  glutInitDisplayMode (GLUT_RGBA | GLUT_DOUBLE);
  glutInitWindowSize (SCREEN_WIDTH, SCREEN_HEIGHT);
  glutInitWindowPosition(80, 80);
  windownumber = glutCreateWindow (APP_TITLE);
  
  glutKeyboardFunc (keyboard);
  glutDisplayFunc (display);
  glutReshapeFunc (reshape); 
  
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  
  // VERY IMPORTANT:
  // this line loads the address of the glTexImage3D function into the function pointer of the same name.
  // glTexImage3D is not implemented in the standard GL libraries and must be loaded dynamically at run time,
  // the environment the program is being run in MAY OR MAY NOT support it, if not we'll get back a NULL pointer.
  // this is necessary to use any OpenGL function declared in the glext.h header file
  // the Pointer to FunctioN ... PROC types are declared in the same header file with a type appropriate to the function name
  glTexImage3D = (PFNGLTEXIMAGE3DPROC) wglGetProcAddress("glTexImage3D");
  if (glTexImage3D == NULL) {
    printf("Error in line %d: Couldn't load glTexImage3D function. Aborting.\n", __LINE__);
    return -1;
  }
  
  tex = build_texture();
  
  glInit (WIDTH, HEIGHT);
  glutTimerFunc(1000, countFPS, 1);
  glutIdleFunc(idle);
  glutMainLoop ();
  
  return 0;
}

void keyboard(unsigned char key, int x, int y) {
  switch ( key ) {
  case ESCAPE:
    printf("Escape key pressed. exit.\n");
    exit(0);
  default:
    printf("Key %d corresponding to char '%c' is not assigned to any action.\n", key, key);
  break;
  }
}

void display() {
  float y, z;
  int x;
  
  // tell GL we want it to use this texture when it textures our polygons
  // (this function has multiple uses, see BuildTexture())
  glBindTexture(GL_TEXTURE_3D, texname);

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();
  // move to somewhere we can see
  glTranslated(0.0, -1.0, -2.4);  
  // spin the pyramid to a good orientation to see what happens to the 3d texturing when the vertex moves
  glRotated(-90, 1.0, 0.0, 0.0);
  glRotated((double)tick / 100.0, 0.0, 0.0, 1.0);
  centervert[2] = sin((double)tick / 3000.0) + 1.0;

  // draw the pyramid with the fluctuating apex
  // turn on 3d texturing if its not already
  glEnable(GL_TEXTURE_3D);
  glBegin(GL_TRIANGLES);
    // texture coordinates are always specified before the vertex they apply to.
    for (x = 0; x <= 3; x++) {
      glTexCoord3d(centervert[0], centervert[1], centervert[2]);
      //glTexCoord3d(centervert[0], centervert[1], 2.0);      // texture stretches rather than glides over the surface with this
      glVertex3d(centervert[0], centervert[1], centervert[2]);

      glTexCoord3d(verts[x][0], verts[x][1], verts[x][2]);
      glVertex3d(verts[x][0], verts[x][1], verts[x][2]);

      glTexCoord3d(verts[(x+1)%4][0], verts[(x+1)%4][1], verts[(x+1)%4][2]);
      glVertex3d(verts[(x+1)%4][0], verts[(x+1)%4][1], verts[(x+1)%4][2]);
    }
  glEnd();

  // we don't want the lines and points textured, so disable 3d texturing for a bit
  glDisable(GL_TEXTURE_3D);
  glBegin(GL_LINES);
    // draw the grids
    for (z = 0; z < 2.0; z += 0.5) {
      for (y = -1.0; y <= 1.0; y += 0.5) {
          glVertex3d(y, -1.0, z); 
          glVertex3d(y, 1.0, z);
      }
      for (y = -1.0; y <= 1.0; y += 0.5) {
          glVertex3d(-1.0, y, z); 
          glVertex3d(1.0, y, z);
      }
    }
    // draw the wire frame size of the pyramid
    for (x = 0; x < 4; x++) {
      glVertex3d(centervert[0], centervert[1], centervert[2]);
      glVertex3d(verts[x][0], verts[x][1], verts[x][2]);
    }
  glEnd();
  // draw the apex point
  glBegin(GL_POINTS);
    glVertex3d(centervert[0], centervert[1], centervert[2]);
  glEnd();
  
  fpsCurrent++;
  
  glutSwapBuffers();
}

void reshape(int w, int h)
{
  glViewport(0, 0, w, h); // Reset The Current Viewport And Perspective Transformation
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90, 1, 0.1, 10.0);
  
  glMatrixMode(GL_MODELVIEW);
}

void glInit (int w, int h) {
  
  // set up our OpenGL state
  glEnable(GL_TEXTURE_3D);  // enable 3d texturing
  glEnable(GL_DEPTH_TEST);
  glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE); // our texture colors will replace the untextured colors
  glClearColor(1.0, 1.0, 1.0, 1.0);
  glColor4d(0.0, 0.0, 0.0, 1.0);
  glPointSize(3.0);
  
  glShadeModel(GL_FLAT);
  
  glViewport (0, 0, (GLsizei) SCREEN_WIDTH, (GLsizei) SCREEN_HEIGHT);
  
  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();
  gluPerspective(90, 4.0/3.0, 0.1, 10.0);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();
  
  fpsCount = 0;
  fpsCurrent = 0;
}

void idle() {
  tick += 10;
  glutPostRedisplay();
}

void countFPS(int value) {
  char title[120];
  fpsCount = fpsCurrent;
  fpsCurrent = 0;
  
  snprintf(title, 120, "%s. FPS: %d", APP_TITLE, fpsCount);
  glutSetWindowTitle(title);
  glutTimerFunc(1000, countFPS, 1);
}

unsigned char*build_texture(void)
{

  // ask for enough memory for the texels and make sure we got it before proceeding
  unsigned char *texels = (BYTE *)malloc(WIDTH * HEIGHT * DEPTH * BYTES_PER_TEXEL);
  if (texels == NULL)
    return NULL;
  int s, t;

  // each of the following loops defines one layer of our 3d texture, there are 3 unsigned bytes (red, green, blue) for each texel so each iteration sets 3 bytes
  // the memory pointed to by texels is technically a single dimension (C++ won't allow more than one dimension to be of variable length), the 
  // work around is to use a mapping function like the one above that maps the 3 coordinates onto one dimension
  // layer 0 occupies the first (width * height * bytes per texel) bytes, followed by layer 1, etc...
  // define layer 0 as red
  for (s = 0; s < WIDTH; s++) {
    for (t = 0; t < HEIGHT; t++) {
      texels[TEXEL3(s, t, 0)] = 0x80;
      texels[TEXEL3(s, t, 0)+1] = 0x00;
      texels[TEXEL3(s, t, 0)+2] = 0x00;
    }
  }
  // define layer 1 as green
  for (s = 0; s < WIDTH; s++) {
    for (t = 0; t < HEIGHT; t++) {
      texels[TEXEL3(s, t, 1)] = 0x00;
      texels[TEXEL3(s, t, 1)+1] = 0x80;
      texels[TEXEL3(s, t, 1)+2] = 0x00;
    }
  }
  // define layer 2 as blue
  for (s = 0; s < WIDTH; s++) {
    for (t = 0; t < HEIGHT; t++) {
      texels[TEXEL3(s, t, 2)] = 0x00;
      texels[TEXEL3(s, t, 2)+1] = 0x00;
      texels[TEXEL3(s, t, 2)+2] = 0x80;
    }
  }

  // define layer 3 as grey
  for (s = 0; s < WIDTH; s++) {
    for (t = 0; t < HEIGHT; t++) {
      texels[TEXEL3(s, t, 3)] = 0x80;
      texels[TEXEL3(s, t, 3)+1] = 0x80;
      texels[TEXEL3(s, t, 3)+2] = 0x80;
    }
  }

  // request 1 texture name from OpenGL
  glGenTextures(1, &texname); 
  // tell OpenGL we're going to be setting up the texture name it gave us 
  glBindTexture(GL_TEXTURE_3D, texname);  
  // when this texture needs to be shrunk to fit on small polygons, use linear interpolation of the texels to determine the color
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  // when this texture needs to be magnified to fit on a big polygon, use linear interpolation of the texels to determine the color
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  // we want the texture to repeat over the S axis, so if we specify coordinates out of range we still get textured.
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  // same as above for T axis
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_T, GL_REPEAT);
  // same as above for R axis
  glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_WRAP_R, GL_REPEAT);
  // this is a 3d texture, level 0 (max detail), GL should store it in RGB8 format, its WIDTHxHEIGHTxDEPTH in size, 
  // it doesnt have a border, we're giving it to GL in RGB format as a series of unsigned bytes, and texels is where the texel data is.
  glTexImage3D(GL_TEXTURE_3D, 0, GL_RGB8, WIDTH, HEIGHT, DEPTH, 0, GL_RGB, GL_UNSIGNED_BYTE, texels);

  return texels;
}
