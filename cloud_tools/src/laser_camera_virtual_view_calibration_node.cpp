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


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <float.h>
#include <vector>

#include "yaml-cpp/yaml.h"
#include <fstream>

#include <GL/glut.h>
#include<X11/X.h>
#include<X11/Xlib.h>
#include<GL/glx.h>

//ROS
#include <ros/ros.h>
#include <ros/node_handle.h>
#include "sensor_msgs/Image.h"
#include "sensor_msgs/PointCloud.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <triangle_mesh/TriangleMesh.h>
#include <cloud_tools/misc.h>


using namespace cloud_tools;

class LaserCameraVirtualViewCalibration
{
public:
  ros::NodeHandle nh_;
  std::string input_mesh_topic_;
  std::string output_image_topic_;
  
  ros::Subscriber mesh_sub_;
  
  image_transport::ImageTransport it_;
  sensor_msgs::CvBridge bridge_;
  image_transport::Publisher image_pub_;
  
  sensor_msgs::PointCloud cloud_in_;
  triangle_mesh::TriangleMesh mesh_; 
  
//parameters
  static volatile int mouse_x;
  static volatile int mouse_y;
  static volatile int mouse_init_x;
  static volatile int mouse_init_y;
  static volatile bool mouse_down;
  bool mouse_down_;
  
  static std::vector<point_3D> points;
  static std::vector<triangle> triangles;
  static int nr_pct;
  static int nr_tr;
  
  static point_3D position;
  static point_3D focal_point;
  static point_3D view_up;
  
  static int displayWin;
  static int width, height;
  static std::string output_ppm;
  //public:
  LaserCameraVirtualViewCalibration (ros::NodeHandle &anode) : nh_(anode), it_(nh_)
  {
    nh_.param ("input_mesh_topic", input_mesh_topic_, std::string("mesh"));
    nh_.param ("output_image_topic", output_image_topic_, std::string("scene_image"));  
//     nh_.param ("mouse_down", mouse_down_, false);  
//     nh_.param ("displayWin", displayWin, 1);
//     nh_.param ("width", width, 640);  
//     nh_.param ("height", height, 480);  
//     mouse_down = mouse_down_;
    mesh_sub_ = nh_.subscribe (input_mesh_topic_, 1, &LaserCameraVirtualViewCalibration::mesh_cb, this);
    image_pub_ = it_.advertise(output_image_topic_, 1);
  }

  /**
   * \brief mesh callback 
   * \param mesh input mesh message
   */
  void mesh_cb (const triangle_mesh::TriangleMeshConstPtr& mesh)
  {
    read_data(mesh, points, nr_pct, triangles, nr_tr);
    glutMotionFunc  ( motion );
    glutMouseFunc  ( mouse );
    glutDisplayFunc  ( draw  );
    glutReshapeFunc  ( reshape  );    
    glutMainLoop ( );
  }


  /**
   * \brief reads the data from a TriangleMesh message
   * \param mesh input mesh message
   * \param points points from a 3D scene
   * \param nr_pct number of points in a 3D scene
   * \param triangles points triangulated
   * \param nr_tr number of triangles
   */
  void read_data(const triangle_mesh::TriangleMeshConstPtr& mesh, std::vector<point_3D> &points, 
                int &nr_pct, std::vector<triangle> &triangles, int &nr_tr)
  {
    if (mesh->points.size() != mesh->intensities.size())
      ROS_WARN("Unusual!!! TriangleMesh's points and intensities channels differ in sizes");
    //read in points
    nr_pct = mesh->points.size();
    points.resize (mesh->points.size());
    for (unsigned int i = 0; i < mesh->points.size(); i++)
    {
      points[i].x = mesh->points[i].x;
      points[i].y = mesh->points[i].y;
      points[i].z = mesh->points[i].z;
    }
    
    //read in triangles
    nr_tr = mesh->triangles.size();
    triangles.resize (mesh->triangles.size());
    for (unsigned int i = 0; i < mesh->triangles.size(); i++)
    {
      triangles[i].a = mesh->triangles[i].i;
      triangles[i].b = mesh->triangles[i].j;
      triangles[i].c = mesh->triangles[i].k;
    }

    //read in intensities
    for (unsigned int i = 0; i < mesh->intensities.size(); i++)
    {
      points[i].i = mesh->intensities[i];
    }
  }
  
  /**
   * \brief unproject 2D point
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
  static void image(const char output[], int width, int height)
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
}

  /**
   * \brief display the image
   */
  static void display (  void )
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
      exit(0);
  }

  /**
   * \brief mouse callback to enable scene drag
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
  }

  /**
   * \brief reshape callback for window reshape
   */
  static void reshape(int width,  int height)
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
    //TODO: parametrize following arguments
    gluPerspective (45.0,aspect,0.001,20);
    glMatrixMode( GL_MODELVIEW );
  }
};



int main (int argc, char* argv[])
{
  glutInit (&argc, argv);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowSize ( 640, 480 );
  glutCreateWindow   ( argv[0]  );
  
  glEnable(GL_DEPTH_TEST); 
  ros::init (argc, argv, "laser_camera_virtual_view_calibration_node");
  ros::NodeHandle nh("~");
  LaserCameraVirtualViewCalibration n (nh);
  ros::spin ();

  return (0);
}



















// int main (  int argc, char* argv[] )
// {
//   if (argc < 2)
//   {
//     fprintf(stderr, "Syntax is %s <configuration.yaml>\n", argv[0]);
//     exit(2);
//   }

//   //parse YAML file
//   std::ifstream fin(argv[1]);
//   YAML::Parser parser(fin);
//   YAML::Node doc;
//   parser.GetNextDocument(doc);
//   std::string input_vtk;
//   doc["vtk_file"] >> input_vtk;
//   std::cerr << "input_vtk: " << input_vtk << std::endl;
//   doc["ppm_file"] >> output_ppm;
//   std::cerr << "output_ppm: " << output_ppm << std::endl;
//   doc["position"] >> position;
//   std::cerr << "position: " << position.x << " " << position.y << " " << position.z << std::endl;
//   doc["focal_point"] >> focal_point;
//   std::cerr << "focal_point: " << focal_point.x << " " << focal_point.y << " " << focal_point.z << std::endl;
//   doc["view_up"] >> view_up;
//   std::cerr << "view_up: " << view_up.x << " " << view_up.y << " " << view_up.z << std::endl;
//   doc["height"] >> height;
//   doc["width"] >> width;
//   doc["display_win"] >> displayWin;
//   std::cerr << "height: " << height << " width: " << width << " displayWin: " << displayWin << std::endl;
//   return 0;
// }
