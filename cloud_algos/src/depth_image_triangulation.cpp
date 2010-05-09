/* 
 * Copyright (c) 2010, Dejan Pangercic <dejan.pangercic@cs.tum.edu>, 
 Zoltan-Csaba Marton <marton@cs.tum.edu>, Nico Blodow <blodow@cs.tum.edu>
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

#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/depth_image_triangulation.h>
#include <ros/this_node.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <iostream>
#include <math.h>
#include <map>
#include <algorithm>
//for savePCDFile ()
#include <point_cloud_mapping/cloud_io.h>

//#define DEBUG 1
using namespace cloud_algos;

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::get_scan_and_point_id (sensor_msgs::PointCloud &cloud, signed int &line_index)
{
  int scan_id = 0;
  int point_id = 0;
  int temp_point_id = 0;
  ros::Time ts = ros::Time::now ();
  cloud.channels.resize(cloud.channels.size()+1);
  cloud.channels[line_index].name = "line";
  
  int j = cloud_io::getIndex (cloud, "index");
  
  cloud.channels[line_index].values.resize(cloud.channels[j].values.size());
  for (unsigned int k = 0; k < cloud.channels[j].values.size()-1; k++)
  {
    point_id = cloud.channels[j].values[k];
    temp_point_id =  cloud.channels[j].values[k+1];
    cloud.channels[line_index].values[k] = scan_id;
    //new line found
    if (temp_point_id < point_id)
    {
      scan_id++;
      //find max point index  in the whole point cloud
      if (point_id > max_index_)
        max_index_ = point_id;
    }
  }
  
  //nr of lines in point cloud
  max_line_ = scan_id;
  if(save_pcd_)
  {
    ROS_INFO("Saving PCD containing line numbers as cloud_line.pcd in ROS home.");
    cloud_io::savePCDFile ("cloud_line.pcd", cloud, false);
  }
#ifdef DEBUG
  ROS_INFO("Nr lines: %d, Max point ID: %d Completed in %f seconds", max_line_, max_index_,  (ros::Time::now () - ts).toSec ());
#endif
}

////////////////////////////////////////////////////////////////////////////////
float DepthImageTriangulation::dist_3d(const sensor_msgs::PointCloud &cloud_in, int a, int b)
{

  return sqrt( (cloud_in.points[a].x - cloud_in.points[b].x) * (cloud_in.points[a].x - cloud_in.points[b].x)
               + (cloud_in.points[a].y - cloud_in.points[b].y) * (cloud_in.points[a].y - cloud_in.points[b].y) 
               + (cloud_in.points[a].z - cloud_in.points[b].z) * (cloud_in.points[a].z - cloud_in.points[b].z) );
}

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::init (ros::NodeHandle &nh)
{
  // node handler and publisher
  nh_ = nh;
  //ROS_INFO("DepthImageTriangulation Node initialized");
  nh_.param("save_pcd", save_pcd_, save_pcd_);
}

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::pre () 
  {

  }

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::post ()
  {
 
  }

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> DepthImageTriangulation::requires () 
  {
    return std::vector<std::string>();
  }

////////////////////////////////////////////////////////////////////////////////
std::vector<std::string> DepthImageTriangulation::provides ()
  {
    return std::vector<std::string>();
  }

////////////////////////////////////////////////////////////////////////////////
std::string DepthImageTriangulation::process (const boost::shared_ptr<const DepthImageTriangulation::InputType>& cloud_in)
{
#ifdef DEBUG
  ROS_INFO("\n");
  ROS_INFO("PointCloud msg with size %ld received", cloud_in->points.size());
#endif

  cloud_with_line_ = *(cloud_in.get());
  //we assume here having either SICK LMS400 data which has line and index coeff
  //or
  //Hokuyo UTM 30LX which only returns index coeff, line has to be computed
  for (unsigned int i = 0; i < cloud_with_line_.channels.size(); i++)
  {
    if (cloud_with_line_.channels[i].name == "line" || cloud_with_line_.channels[i].name == "pid")
      line_nr_in_channel_ = i;
    if (cloud_with_line_.channels[i].name == "index" || cloud_with_line_.channels[i].name == "sid")
      index_nr_in_channel_ = i;
  }
  ROS_INFO("line index: %d, index index: %d", line_nr_in_channel_, index_nr_in_channel_);

  //format of this pcd is not known
  if (index_nr_in_channel_ == -1)
  {
    ROS_ERROR("Channel index is missing - this is NOT allowed");
    return std::string("");
  }
  
  //Hokuyo data, compute line
  if (line_nr_in_channel_ == -1)
  {
    line_nr_in_channel_ =  cloud_with_line_.channels.size();
    
    {
      boost::mutex::scoped_lock lock(cloud_lock_);
      //cloud_in gets processed and copied into cloud_with_line_;
      get_scan_and_point_id(cloud_with_line_, line_nr_in_channel_);
    }
  }

  if (max_line_ == 0 || max_index_ == 0)
  {
    max_line_ =  *max_element(cloud_with_line_.channels[line_nr_in_channel_].values.begin(), 
                              cloud_with_line_.channels[line_nr_in_channel_].values.end());

    max_index_ =  *max_element(cloud_with_line_.channels[index_nr_in_channel_].values.begin(), 
                                  cloud_with_line_.channels[index_nr_in_channel_].values.end());
    ROS_INFO("max line: %d, max index: %d", max_line_, max_index_);
    //exit(2);
  }

  ros::Time ts = ros::Time::now ();
  std::vector<triangle> tr;
  //resize big time to prevent seg fault
  tr.resize(2*max_line_*max_index_);    
  mesh_ = boost::shared_ptr<DepthImageTriangulation::OutputType>(new DepthImageTriangulation::OutputType);
  mesh_->points.resize(0);//2*max_line_*max_index_);
  mesh_->triangles.resize(0);//2*max_line_*max_index_);
  mesh_->intensities.resize(0);

  int nr = 0; //number of triangles
  int nr_tr;
    
  int a=0, b, c, d, e;
  bool skipped = false;
    
  //scan line
  for (int i = 0; i <=  max_line_; i++)
  {
    //laser beam in a scan line
    for (int j = 0; j <= max_index_; j++)
    {       
     // find top left corner
      if (cloud_with_line_.channels[line_nr_in_channel_].values[a] == i && cloud_with_line_.channels[index_nr_in_channel_].values[a] == j)
      {
        //ROS_INFO("found point a = %d\n", a);
        b = -1;
        c = -1;
        d = -1;
        e = -1;
          
        // find top right corner
        if (a+1 < cloud_with_line_.points.size() && cloud_with_line_.channels[line_nr_in_channel_].values[a+1] == i
                   && cloud_with_line_.channels[index_nr_in_channel_].values[a+1] == j+1)
          b = a+1;
          
        //ROS_INFO("resolved point b = %d\n", b);
          
        // go to next line
        int test = a;
        while ((unsigned long)test < cloud_with_line_.points.size() &&  cloud_with_line_.channels[line_nr_in_channel_].values[test] < i+1)
          test++;
          
        //ROS_INFO("resolved next line\n");
          
        // if next line exists
        if ((unsigned long)test < cloud_with_line_.points.size() && cloud_with_line_.channels[line_nr_in_channel_].values[test] == i+1)
        {
          // a skipped triangle exists because of missing 'a'
          if (skipped)
          {
            skipped = false; // reset var
              
            // go to column j-1
            while ((unsigned long)test <  cloud_with_line_.points.size() &&  cloud_with_line_.channels[index_nr_in_channel_].values[test] < j-1)
              test++;
              
            // if not at the end of dataset
            if ((unsigned long)test <  cloud_with_line_.points.size())
            {
              // if column exists
              if (cloud_with_line_.channels[line_nr_in_channel_].values[test] == i+1 && cloud_with_line_.channels[index_nr_in_channel_].values[test])
              {
                e = test;
                test++;
              }
            }
          }
          else
          {
            // go to column j
            while ((unsigned long)test < cloud_with_line_.points.size() && cloud_with_line_.channels[index_nr_in_channel_].values[test] < j)
              test++;
          }
            
          // if not at the end of dataset
          if ((unsigned long)test < cloud_with_line_.points.size())
          {
            // if column exists
            if (cloud_with_line_.channels[line_nr_in_channel_].values[test] == i+1 && cloud_with_line_.channels[index_nr_in_channel_].values[test] == j)
            {
              c = test;
              if ((unsigned long)c+1 < cloud_with_line_.points.size() && cloud_with_line_.channels[line_nr_in_channel_].values[c+1] == i+1
                  && cloud_with_line_.channels[index_nr_in_channel_].values[c+1] == j+1)
                d = c+1;
            }
            // if next column was found
            else if (cloud_with_line_.channels[line_nr_in_channel_].values[test] == i+1 && cloud_with_line_.channels[index_nr_in_channel_].values[test] == j+1)
              d = test;
          }
        }
          
        if (c != -1)
        {
          float AC = dist_3d (cloud_with_line_, a, c);
          if (e != -1)
          {
            //ROS_INFO ("a c e\n");
            // a c e
            float AE = dist_3d (cloud_with_line_, a, e);
            float CE = dist_3d (cloud_with_line_, c, e);
            if (AC < max_length && CE < max_length && AE < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = c;
              tr[nr].c = e;
              nr++;
            }
          }
            
          if (b != -1)
          {
            //ROS_INFO ("a b c\n");
            // a b c
            float AB = dist_3d (cloud_with_line_, a, b);
            float BC = dist_3d (cloud_with_line_, b, c);
            float AC = dist_3d (cloud_with_line_, a, c);
            if (AB < max_length && BC < max_length && AC < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = b;
              tr[nr].c = c;
              nr++;
            }
              
            if (d != -1)
            {
              //ROS_INFO ("b c d\n");
              // b c d
              float BD = dist_3d (cloud_with_line_, b, d);
              float CD = dist_3d (cloud_with_line_, c, d);
              if (BD < max_length && BC < max_length && CD < max_length)
              {
                tr[nr].a = b;
                tr[nr].b = c;
                tr[nr].c = d;
                nr++;
              }
            }
          }
          else if (d != -1)
          {
            //ROS_INFO ("a c d\n");
            // a c d
            float AD = dist_3d (cloud_with_line_, a, d);
            float CD = dist_3d (cloud_with_line_, c, d);
            if (AD < max_length && CD < max_length && AC < max_length)
            {
              tr[nr].a = a;
              tr[nr].b = c;
              tr[nr].c = d;
              nr++;
            }
          }
        }
        else if (b != -1 && d != -1)
        {
          //ROS_INFO ("a b d\n");
          // a b d
          float AB = dist_3d (cloud_with_line_, a, b);
          float AD = dist_3d (cloud_with_line_, a, d);
          float BD = dist_3d (cloud_with_line_, b, d);
          if (AD < max_length && BD < max_length && AB < max_length)
          {
            tr[nr].a = a;
            tr[nr].b = b;
            tr[nr].c = d;
            nr++;
          }
        }
          
        // move to next point
        a++;
        //skipped = false;
        if ((unsigned long)a >= cloud_with_line_.points.size())
          break;
      } // END OF: top left corner found
      else
        skipped = true;
    }
    if ((unsigned long)a >= cloud_with_line_.points.size())
      break;
  }
  nr_tr = nr;
  tr.resize(nr);
  mesh_->header = cloud_with_line_.header;   
  mesh_->sending_node = ros::this_node::getName();   
  geometry_msgs::Point32 tr_i, tr_j, tr_k;
  triangle_mesh::Triangle tr_mesh;

#ifdef DEBUG  
  ROS_INFO("Triangle a: %d, b: %d, c: %d", tr[i].a, tr[i].b, tr[i].c);
#endif
  //fill up TriangularMesh msg and send it on the topic
  for (unsigned long i=0; i<cloud_with_line_.points.size(); i++)
    mesh_->points.push_back (cloud_with_line_.points[i]);

  for (unsigned long i = 0; i < nr; i++)
  {
    if ((unsigned long)tr[i].a >= cloud_with_line_.points.size() || tr[i].a < 0 || isnan(tr[i].a))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else if ((unsigned long)tr[i].b >= cloud_with_line_.points.size() || tr[i].b < 0 || isnan(tr[i].b))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else if ((unsigned long)tr[i].c >= cloud_with_line_.points.size() || tr[i].c < 0 || isnan(tr[i].c))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else if (tr[i].a == tr[i].b || tr[i].a == tr[i].c || tr[i].b == tr[i].c)
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, tr[i].a, tr[i].b, tr[i].c, cloud_with_line_.points.size());
    else
    {
      tr_mesh.i = tr[i].a;
      tr_mesh.j = tr[i].c;
      tr_mesh.k = tr[i].b;
      mesh_->triangles.push_back(tr_mesh);
    } 
  }

  //fill in intensities (needed for e.g. laser-to-camera calibration
  int iIdx = getChannelIndex(cloud_with_line_, "intensities");
  if (iIdx == -1)
    ROS_WARN ("[DepthImageTriangulaton] \"intensites\" channel does not exist");
  
  else
  {
    mesh_->intensities.resize(cloud_with_line_.channels[iIdx].values.size());
    for(unsigned long i=0; i < cloud_with_line_.channels[iIdx].values.size(); i++)
      //for (unsigned long i = 0; i < nr; i++)
      mesh_->intensities[i] = cloud_with_line_.channels[iIdx].values[i];  
  }
  

  //set indices back to initial values
  line_nr_in_channel_ = index_nr_in_channel_ = -1;
  
//  mesh_.triangles.resize(nr);
//  mesh_.points.resize(nr*3);
  
  //write to vtk file for display in e.g. Viewer
  //if(write_to_vtk_)
    //write_vtk_file("triangles.vtk", tr, cloud_with_line_, nr);
  ROS_INFO("Triangulation with %ld triangles completed in %g seconds", tr.size(), (ros::Time::now () - ts).toSec());
  return std::string("");
}

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::write_vtk_file(std::string output, std::vector<triangle> triangles,  
                                             const sensor_msgs::PointCloud &cloud_in,
                                             int nr_tr)
{
  /* writing VTK file */
  
  FILE *f;
  
  f = fopen(output.c_str(),"w");
  
  fprintf (f, "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS %ld float\n",cloud_with_line_.points.size());
  unsigned long i;
  
  for (i=0; i<cloud_with_line_.points.size(); i++)
  {
    fprintf (f,"%f %f %f ", cloud_with_line_.points[i].x, cloud_with_line_.points[i].y,
               cloud_with_line_.points[i].z);
    fprintf (f,"\n");
  }
  
  fprintf(f,"VERTICES %ld %ld\n", cloud_with_line_.points.size(), 2*cloud_with_line_.points.size());
  for (i=0; i<cloud_with_line_.points.size(); i++)
    fprintf(f,"1 %ld\n", i);
    
  ROS_INFO("vector: %ld, nr: %d  ", triangles.size(), nr_tr);
  
  fprintf(f,"\nPOLYGONS %d %d\n", nr_tr, 4*nr_tr);
  for (int i=0; i<nr_tr; i++)
  {
    if ((unsigned long)triangles[i].a >= cloud_with_line_.points.size() || triangles[i].a < 0 || isnan(triangles[i].a))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else if ((unsigned long)triangles[i].b >= cloud_with_line_.points.size() || triangles[i].b < 0 || isnan(triangles[i].b))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else if ((unsigned long)triangles[i].c >= cloud_with_line_.points.size() || triangles[i].c < 0 || isnan(triangles[i].c))
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else if (triangles[i].a == triangles[i].b || triangles[i].a == triangles[i].c || triangles[i].b == triangles[i].c)
      ;//printf("triangle %d/%d: %d %d %d / %d\n", i, nr_tr, triangles[i].a, triangles[i].b, triangles[i].c, cloud_with_line_.points.size());
    else
      fprintf(f,"3 %d %d %d\n",triangles[i].a, triangles[i].c, triangles[i].b);
  }
}

////////////////////////////////////////////////////////////////////////////////
boost::shared_ptr<const DepthImageTriangulation::OutputType> DepthImageTriangulation::output ()
  {
    return mesh_;
  }


#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <DepthImageTriangulation> (argc, argv);
}
#endif


