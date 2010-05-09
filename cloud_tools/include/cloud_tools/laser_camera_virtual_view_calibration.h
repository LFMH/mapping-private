/* 
 * Copyright (c) 2010, 
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

#ifndef CLOUD_TOOLS_LASER_CAMERA_VIRTUAL_VIEW_CALIBRATION_H
#define CLOUD_TOOLS_LASER_CAMERA_VIRTUAL_VIEW_CALIBRATION_H
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <float.h>
#include <vector>

#include "yaml-cpp/yaml.h"
#include <fstream>

#include <GL/freeglut.h>
#include<X11/X.h>
#include<X11/Xlib.h>
#include<GL/glx.h>
#include <sensor_msgs/Image.h>
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

//data structs
struct point_3D
{
  double x,y,z;
  int i;
};

struct triangle
{
  int a,b,c;
};

int lc_main (int argc, char* argv[], std::string laser_image_name, 
             point_3D p, point_3D fp, point_3D vu, double w, double h, int d,
             std::vector<point_3D> &points_, int &nr_pct_, std::vector<triangle> &triangles_, int &nr_tr_);
#endif
