//  cc -o me me.cc ../common/CommonTerminalRoutines.cc ../common/StringTokenizer.cc -lglut -lGLU -lGL -lXmu -lXext -lX11 -lm -g
// ./me triangle.vtk img.ppm -position -1.01656,3.58189,2.6436 -focal_point 1.76435,-0.897017,0.58908 -view_up 0.271599,-0.256825,0.92751 -resolution 480,640 -displayWin 1

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <float.h>
#include <vector>

#include "../common/CommonTerminalRoutines.h"

#include <GL/glut.h>
#include<X11/X.h>
#include<X11/Xlib.h>
#include<GL/glx.h>


typedef struct point_3D
{
  double x,y,z;
  int i;
};

typedef struct triangle
{
  int a,b,c;
};

volatile int mouse_x;
volatile int mouse_y;
volatile int mouse_init_x;
volatile int mouse_init_y;
volatile bool mouse_down = false;

 std::vector<point_3D> points;
 std::vector<triangle> triangles;
 int nr_pct;
 int nr_tr;

point_3D position;
point_3D focal_point;
point_3D view_up;
int displayWin = 1;
int width = 640, height = 480;
char *output_ppm;

int read_data(char input[], std::vector<point_3D> &points, int &nr_pct, std::vector<triangle> &triangles, int &nr_tr)
{
#define BUFSIZE 1024
  char *s = (char*) malloc (BUFSIZE * sizeof(char));
  FILE *f;

  f = fopen(input, "r");

  if (f==NULL)
      printf("error\n");

  for (int i=0; i<11; i++)
    fscanf(f,"%s",s);


   fscanf(f,"%d",&nr_pct);

   points.resize (nr_pct);

   fscanf(f,"%s\n",s);

   for (int i = 0; i < nr_pct; i++)
   {
     fscanf (f,"%lf %lf %lf\n", &(points[i].x), &(points[i].y), &(points[i].z));
   }

   fscanf(f,"%s",s);

   int a;
   fscanf(f,"%d",&a);
   fscanf(f,"%d",&a);

   for (int i = 0; i < nr_pct; i++)
   {
     fscanf(f,"%d %d",&a, &a);
   }

   fscanf(f,"%s\n",s);

   fscanf(f,"%d",&nr_tr);

   triangles.resize (nr_tr);

   fscanf(f,"%d",&a);

   for (int i = 0; i < nr_tr; i++)
   {
     fscanf (f,"%d %d %d %d\n", &a, &(triangles[i].a), &(triangles[i].b), &(triangles[i].c));
   }

   for (int i=0; i<7; i++)
     fscanf(f,"%s",s);

   for (int i = 0; i < nr_pct; i++)
   {
     fscanf(f,"%d",&(points[i].i));
   }

   fclose(f);
   return 0;
}

point_3D point3D_from_window_displayed(int x, int y)
{
  float depth;
  glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

  double modelview[16];
  double projection[16];
  int viewport[4];

  glGetDoublev(GL_MODELVIEW_MATRIX , modelview);
  glGetDoublev(GL_PROJECTION_MATRIX , projection);
  glGetIntegerv(GL_VIEWPORT, viewport );

  point_3D point;
  gluUnProject(x, y, depth, modelview, projection, viewport, &point.x, &point.y, &point.z);

  return point;
}

void image(char output[], int width, int height)
{
  int maxval = 255;

 unsigned char *pixels = new unsigned char [(width) * (height)];

  glPixelStorei(GL_PACK_ALIGNMENT, 1);
//  glReadBuffer(GL_FRONT);
//  glReadBuffer(GL_RENDERBUFFER_EXT);
  glReadPixels(0, 0, width, height, GL_RED, GL_UNSIGNED_BYTE, pixels);

  FILE* f;

  f=fopen(output,"w");

  fprintf(f,"P3\n");
  fprintf(f,"%d %d\n", width, height); 
  fprintf(f,"%d\n",maxval);

  for (int i=height-1; i>=0; i--)
  {
    for (int j=0; j<width; j++)
    {
      fprintf(f," ");
      int intensity = pixels[i*width+j];
      if (intensity > maxval)
        intensity = maxval;
      fprintf(f, "%d %d %d", intensity, intensity, intensity);
      fprintf(f," ");
    }
    fprintf(f,"\n");
  }
  fclose(f);

//   FILE *file;
//   file = fopen("pct3D.txt","w");
//   point_3D a;
//   for (int i=height-1; i>=0; i--)
//   {
//     for (int j=0; j<width; j++)
//     {
//       a = point3D_from_window_displayed(i, j);
//       a.i = pixels[i*width+j];
//       fprintf(file, "%g %g %g %d", a.x, a.y, a.z, a.i);
//       fprintf(file,"\n");
//     }
//   }
//   fclose(file);
}

void display (  void )
{
  static float rot_x=0, rot_y=0;

  if (mouse_down)
  {
    rot_x += mouse_x/10.0;
    rot_y += mouse_y/10.0;
  }
//   gluLookAt (-0.248932, 0.626674, 1.13531, 1.07643, 1.16711, 0.44008, 0.216201, -0.22629, 0.949763);
//   gluLookAt (-1.36643,1.11344,1.77504, 2.42245,-0.734004,0.408325, 0.354688,0.0362512,0.934282);
//   gluLookAt (1.59788,-0.331947,0.705763, 2.42245,-0.734004,0.408325, 0.354688,0.0362512,0.934282);
//   gluLookAt (0.961674,-0.0217355,0.935255, 2.42245,-0.734004,0.408325, 0.354688,0.0362512,0.934282);

// gluLookAt (-3.03527,1.70759,2.92716,2.42245,-0.734004,0.408325,0.35361,-0.160285,0.921558);

  //specify the position of the camera according to the scene
  gluLookAt (position.x, position.y, position.z, focal_point.x, focal_point.y, focal_point.z, view_up.x, view_up.y, view_up.z);

  glRotatef (rot_y,0.0,1.0,0.0);
  glRotatef (rot_x,0.0,0.0,1.0);

  //rendering the triangles in the scene
  for (int i=0; i<nr_tr; i++)
  {
    int col;
    glBegin (GL_TRIANGLES);
      col = points[triangles[i].a].i;
      glColor3b(col, col, col);
      glVertex3d(points[triangles[i].a].x, points[triangles[i].a].y, points[triangles[i].a].z);

      col = points[triangles[i].b].i;
      glColor3b(col, col, col);
      glVertex3d(points[triangles[i].b].x, points[triangles[i].b].y, points[triangles[i].b].z);

      col = points[triangles[i].c].i;
      glColor3b(col, col, col);
      glVertex3d(points[triangles[i].c].x, points[triangles[i].c].y, points[triangles[i].c].z);
    glEnd ();
   }

  glFlush ();
  glFinish ();
  image(output_ppm, width, height);
  if (displayWin == 0)
    exit(0);
}

static void
    mouse (int button, int state, int x, int y)
{
  if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
  {
    mouse_down = true;
    mouse_init_x = x;
    mouse_init_y = y;
    glutPostRedisplay ();
  }

  if (button == GLUT_LEFT_BUTTON && state == GLUT_UP)
  {
    mouse_down = false;
    glutPostRedisplay ();
  }
}

static void motion (int x,int y)
{
  if (mouse_down)
  {
    mouse_x = x - mouse_init_x;
    mouse_y = y - mouse_init_y;
    glutPostRedisplay ();
  }
}

static void
    draw(void)
{
  glPushMatrix();
  glMatrixMode(GL_MODELVIEW);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  glClear(GL_COLOR_BUFFER_BIT);
  glClearDepth(1.0f);
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);
  glLoadIdentity ();

  display();

  glPopMatrix();
  glutSwapBuffers();
}


void reshape(int width,  int height)
{
  GLdouble    aspect, left, right, bottom, top;
  glViewport   ( 0, 0, width, height );

  aspect = (GLdouble) width / (GLdouble) height;
  if ( aspect < 1.0 ) {
    left = -2.0;
    right = 2.0;
    bottom = -2.0 * ( 1.0 / aspect );
    top = 2.0 * ( 1.0 / aspect );
  } else {
    left = -2.0 * aspect;
    right = 2.0 * aspect;
    bottom = -2.0;
    top = 2.0;
  }

  glMatrixMode( GL_PROJECTION );
  glLoadIdentity();
//  glOrtho( left, right, bottom, top, -5.0, 5.0 );
  gluPerspective (35.0,aspect,0.001,20);
  glMatrixMode( GL_MODELVIEW );
}

int main (  int argc, char* argv[] )
{
  position.x = 0.0; position.y = 0.0; position.z = 0.0;
  focal_point.x = 0.0; focal_point.y = 0.0; focal_point.z = 0.0;
  view_up.x = 0.0; view_up.y = 0.0; view_up.z = 0.0;

  if (argc < 12)
  {
    print_error (stderr, "Syntax is: %s <input>.vtk <output>.ppm <options>\n", argv[0]);
    fprintf (stderr, "  where options are:\n");
    fprintf (stderr,"      -position x,y,z = specify the view point coordinates (default ");
      print_value (stderr,"%g %g %g", position.x, position.y, position.z); fprintf (stderr,")\n");
    fprintf (stderr,"      -focal_point x,y,z = specify the focal point (default ");
      print_value (stderr,"%g %g %g", focal_point.x, focal_point.y, focal_point.z); fprintf (stderr,")\n");
    fprintf (stderr,"      -view_up x,y,z = specify the view up vector (default ");
      print_value (stderr,"%g %g %g", view_up.x, view_up.y, view_up.z); fprintf (stderr,")\n");
    fprintf (stderr, "      -resolution height,width = specify the resolution of the image (default ");
      print_value (stderr, "%d %d", height, width); fprintf (stderr, ")\n");
    fprintf (stderr, "      -displayWin display = specify if the window will be displayed or not (true = 1; false = 0) (default ");
      print_value (stderr, "%d", displayWin); fprintf (stderr, ")\n");
  }

   // Get the .vtk files
  std::vector<int> pVTKFileIndices = ParseFileExtensionArgument (argc, argv, ".vtk");

  if (pVTKFileIndices.size () < 1)
  {
    print_error (stderr, "Need an input .VTK file!\n");
    return (-1);
  }

  char *input_vtk = argv[pVTKFileIndices.at (0)];

  // Get the .ppm files
  std::vector<int> pPPMFileIndices = ParseFileExtensionArgument (argc, argv, ".ppm");

  if (pPPMFileIndices.size () < 1)
  {
    print_error (stderr, "Need an output .PPM file!\n");
    return (-1);
  }

  output_ppm = argv[pPPMFileIndices.at (0)]; 

  //parsing the rest of the arguments
  Parse3xArguments (argc, argv, "-position", position.x, position.y, position.z);
  Parse3xArguments (argc, argv, "-focal_point", focal_point.x, focal_point.y, focal_point.z);
  Parse3xArguments (argc, argv, "-view_up", view_up.x, view_up.y, view_up.z);
  ParseRangeArguments (argc, argv, "-resolution", height, width);
  ParseArgument (argc, argv, "-displayWin", displayWin);

  glutInit ( &argc, argv );
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize ( width, height );
  glutCreateWindow   ( argv[0]  );

  glEnable(GL_DEPTH_TEST); 

  //read data ( points, triangles and intensities ) from the .vtk file
  read_data(input_vtk, points, nr_pct, triangles, nr_tr);

  glutMotionFunc  ( motion );
  glutMouseFunc  ( mouse );

  glutDisplayFunc  ( draw  );

  glutReshapeFunc  ( reshape  );

  glutMainLoop ( );

  return 0;
}
