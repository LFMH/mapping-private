/* 
 * Copyright (c) 2010, 
 * Ioana <ioana_the_first@yahoo.com>
 * Zoltan-Csaba Marton <marton@cs.tum.edu>
 * Dejan Pangercic <pangercic@cs.tum.edu>
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** 
@file

@brief laser_camera_virtual_view_calibration takes in a triangular mesh (from 
triangulated PointCloud message) from a vtk file
and generates the scene image for a given focal length and a view point. The generated 
image is then used to carry out tilting laser-camera calibration as a normal
stereo calibration.
@par Execute:
 - ./laser_camera_virtual_view_calibration configuration.yaml

@par Example configuration file:
- vtk_file: path_to_file/test.vtk
- ppm_file: output.ppm
- position: [2.18616,0.602594,0.224822]
- focal_point: [0.66648,-3.03167,2.5434]
- view_up: [0.339361,0.400962,0.850919]
- height: 480
- width: 640
- display_win: 1
*/

#include <cloud_tools/laser_camera_virtual_view_calibration.h>

//parameters
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
std::string output_ppm;
sensor_msgs::Image output_ppm_ros;

/**
 * \brief overloaded operator 
 * \param node YAML node
 * \param v point_3D
 */
void operator >> (const YAML::Node& node, point_3D &v) 
{
  node[0] >> v.x;
  node[1] >> v.y;
  node[2] >> v.z;
}

/**
 * \brief reads the data from a vtk file
 * \param input vtk file
 * \param points points from a 3D scene
 * \param nr_pct number of points in a 3D scene
 * \param triangles points triangulated
 * \param nr_tr number of triangles
 */
int read_data(const char input[], std::vector<point_3D> &points, int &nr_pct, std::vector<triangle> &triangles, int &nr_tr)
{
#define BUFSIZE 1024
  char *s = (char*) malloc (BUFSIZE * sizeof(char));
  FILE *f;

  f = fopen(input, "r");

  if (f==NULL)
  {
    printf("error opening file %s\n", input);
    exit(0);
  }

  //skip header:
  //# vtk DataFile Version 3.0
  //vtk output
  //ASCII
  //DATASET POLYDATA
  //POINTS
  for (int i=0; i<11; i++)
    fscanf(f,"%s",s);

  //get number of points
   fscanf(f,"%d",&nr_pct);
   //std::cerr << "nr pct: " << nr_pct << std::endl;
   points.resize (nr_pct);

   //skip "float"
   fscanf(f,"%s\n",s);

   //get points
   for (int i = 0; i < nr_pct; i++)
   {
     fscanf (f,"%lf %lf %lf\n", &(points[i].x), &(points[i].y), &(points[i].z));
     //printf ("%lf %lf %lf\n", points[i].x, points[i].y, points[i].z);
   }

   int a;

   //skip POLYGONS
   fscanf(f,"%s\n",s);
   //nr of triangles/polygons
   fscanf(f,"%d",&nr_tr);
   //printf("nr_tr: %d\n", nr_tr);
   triangles.resize (nr_tr);
   //skip number after number of polygons
   fscanf(f,"%d",&a);

   //read in triangle vertices
   for (int i = 0; i < nr_tr; i++)
   {
     fscanf (f,"%d %d %d %d\n", &a, &(triangles[i].a), &(triangles[i].b), &(triangles[i].c));
   }
   //skip:
   //POINT_DATA 77782
   //SCALARS scalars double
   //LOOKUP_TABLE default
   for (int i=0; i<7; i++)
     fscanf(f,"%s",s);

   //read in intensities
   for (int i = 0; i < nr_pct; i++)
   {
     fscanf(f,"%d",&(points[i].i));
   }

   fclose(f);
   return 0;
}


/**
 * \brief ubnproject 2D point
 * \param x x image coordinate
 * \param y y image coordinate
 */
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


/**
 * \brief generate the image
 * \param output image name
 * \param width image width
 * \param height image height
 */
void image(std::string laser_image_name, int width, int height)
{
  int maxval = 255;
  unsigned char *pixels = new unsigned char [(width) * (height)];
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  //  glReadBuffer(GL_FRONT);
  //  glReadBuffer(GL_RENDERBUFFER_EXT);
  glReadPixels(0, 0, width, height, GL_RED, GL_UNSIGNED_BYTE, pixels);
  
  IplImage * cv_image = NULL;
  cv_image = cvCreateImage(cvSize(width, height), IPL_DEPTH_8U, 1);
  int k = 0;
  for (int i=height-1; i>=0; i--)
  {
    for (int j=0; j<width; j++)
    {
      int intensity = pixels[i*width+j];
      if (intensity > maxval)
        intensity = maxval;
      cv_image->imageData[k] = intensity;
      k++;
    }
  }
  cv::imwrite(laser_image_name, cv_image);
  cvReleaseImage(&cv_image);
}

/**
 * \brief display the image
 */
void display (  void )
{
  static float rot_x=0, rot_y=0;

  if (mouse_down)
  {
    rot_x += mouse_x/10.0;
    rot_y += mouse_y/10.0;
  }
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
  image(output_ppm.c_str(), width, height);
  if (displayWin == 0)
  {
    std::cerr << "returning to loop" << std::endl;
    glutLeaveMainLoop();
    //exit(0);
  }
  //return 0;
}

/**
 * \brief mouse callback for enable scene drag
 */
static void  mouse (int button, int state, int x, int y)
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

/**
 * \brief motion callback
 */
static void motion (int x,int y)
{
  if (mouse_down)
  {
    mouse_x = x - mouse_init_x;
    mouse_y = y - mouse_init_y;
    glutPostRedisplay ();
  }
}

/**
 * \brief display/draw callback
 */
static void draw(void)
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
  //glutLeaveMainLoop();
}

/**
 * \brief reshape callback for window reshape
 */
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
  gluPerspective (45.0,aspect,0.001,20);
  glMatrixMode( GL_MODELVIEW );
}

int lc_main (int argc, char* argv[], std::string laser_image_name, 
             point_3D p, point_3D fp, point_3D vu, double w, double h, int d,
             std::vector<point_3D> &points_, int &nr_pct_, std::vector<triangle> &triangles_, int &nr_tr_)
{
  output_ppm = laser_image_name;
  std::cerr << "output_ppm: " << output_ppm << std::endl;
  position = p;
  std::cerr << "position: " << position.x << " " << position.y << " " << position.z << std::endl;
  focal_point = fp;
  std::cerr << "focal_point: " << focal_point.x << " " << focal_point.y << " " << focal_point.z << std::endl;
  view_up = vu;
  std::cerr << "view_up: " << view_up.x << " " << view_up.y << " " << view_up.z << std::endl;
  height = h;
  width = w;
  displayWin = d;
  std::cerr << "height: " << height << " width: " << width << " displayWin: " << displayWin << std::endl;
  nr_pct = nr_pct_;
  nr_tr = nr_tr_;
  points = points_;
  triangles = triangles_;
  
  glutInit (&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize ( width, height );
  glutCreateWindow   ( argv[0]  );
  //  glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_GLUTMAINLOOP_RETURNS);
  glutSetOption (GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
  glEnable(GL_DEPTH_TEST); 

  //read_data(input_vtk.c_str(), points, nr_pct, triangles, nr_tr);

  glutMotionFunc  ( motion );
  glutMouseFunc  ( mouse );

  glutDisplayFunc  ( draw  );

  glutReshapeFunc  ( reshape  );

  glutMainLoop ( );

  return 0;
}
