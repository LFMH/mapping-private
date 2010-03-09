/*
 * Copyright (c) 2008 Dejan Pangercic <pangerci -=- cs.tum.edu>
 *
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
 *
 * $Id: triangular_mesh_to_vtk.cpp 21050 2009-08-07 21:24:30Z pangercic $
 *
 */

/**
   @mainpage

   @htmlinclude manifest.html

   \author Dejan  Pangercic

   @b triangular_mesh_to_vtk concatenates n [ias_table_msgs/TriangularMesh] 
   messages and writes them to a VTK compliant format.

**/

// ROS core
#include <ros/node_handle.h>
#include <point_cloud_mapping/cloud_io.h>
#include <ias_table_msgs/TriangularMesh.h>

#include <math.h>

using namespace std;

class TriangularMeshToVTK
{
protected:
  ros::NodeHandle nh_;
  ros::Subscriber mesh_sub_;
  ros::Publisher mesh_pub_;
  std::string input_mesh_topic_, output_vtk_file_;
  int file_name_counter_;
public:

  int mesh_count_, nr_of_mesh_publishers_;
  
  // ROS messages
  ias_table_msgs::TriangularMesh mesh_; 


  TriangularMeshToVTK (ros::NodeHandle &n) : nh_(n)
  {
    nh_.param("input_mesh_topic", input_mesh_topic_, std::string("mesh"));
    nh_.param("nr_of_mesh_publishers", nr_of_mesh_publishers_, 1);
    nh_.param("output_vtk_file", output_vtk_file_, std::string("mesh.vtk"));
    mesh_sub_ = nh_.subscribe(input_mesh_topic_, 10, &TriangularMeshToVTK::mesh_cb, this);
    mesh_pub_ = nh_.advertise<ias_table_msgs::TriangularMesh> ("output_mesh", 1);
    mesh_count_ = 0;
    mesh_.points.resize(0);
    mesh_.triangles.resize(0);
    file_name_counter_ = 0;
    std::cerr << "nr_of_mesh_publishers_: " << nr_of_mesh_publishers_ << std::endl;
  }

  ////////////////////////////////////////////////////////////////////////////////
  // \brief mesh callback
  void mesh_cb(const ias_table_msgs::TriangularMeshConstPtr& mesh)
  {
    mesh_.header = mesh->header;
    //mesh_.points.resize(mesh_points_size_before + mesh->points.size());
    //mesh_.triangles.resize(mesh_triangles_size_before + mesh->triangles.size());
    unsigned long size_points = mesh_.points.size();
    unsigned long size_triangles = mesh_.triangles.size();
    for (unsigned long i = 0; i < mesh->points.size(); i++)
      mesh_.points.push_back(mesh->points[i]);

    for (unsigned long j = 0; j < mesh->triangles.size(); j++)
    {
      mesh_.triangles.push_back(mesh->triangles[j]);
      mesh_.triangles[size_triangles + j].i += size_points;
      mesh_.triangles[size_triangles + j].j += size_points;
      mesh_.triangles[size_triangles + j].k += size_points;
    }

    mesh_count_++;
//     if (mesh_count_ == nr_of_mesh_publishers_)
//     {
//       write_vtk_file("mesh1.vtk", mesh_);
//       mesh_count_ = 0;
//       mesh_.points.resize(0);
//       mesh_.triangles.resize(0);
//     }
//     else
//     {
//       write_vtk_file("mesh2.vtk", mesh_);
//       mesh_.points.resize(0);
//       mesh_.triangles.resize(0);
//     }
    
    //TODO: I assume here that messages are coming in sequentially
    //Solve this through identification of publishers
    //std::cerr << "nr_of_mesh_publishers_ "<< nr_of_mesh_publishers_ << std::endl;
    //if (mesh_count_ == nr_of_mesh_publishers_)
    if (mesh_count_ == 2)
    {
      mesh_pub_.publish(mesh_);
      write_vtk_file(output_vtk_file_, mesh_);
      mesh_.points.resize(0);
      mesh_.triangles.resize(0);
      mesh_count_ = 0;
    }
  }  
  ////////////////////////////////////////////////////////////////////////////////
  // \brief write TriangularMesh to vtk file
  void write_vtk_file(std::string output, ias_table_msgs::TriangularMesh &mesh_)
  {
    /* writing VTK file */
    ROS_WARN("Writting to vtk file");
    FILE *f;
    std::string s;
    std::stringstream out;
    file_name_counter_++;
    out << file_name_counter_;
    s = out.str();
    output = s + output;
    
    f = fopen(output.c_str(),"w");
    fprintf (f, "# vtk DataFile Version 3.0\nvtk output\nASCII\nDATASET POLYDATA\nPOINTS %ld float\n",mesh_.points.size());
    unsigned long i;
    
    for (i=0; i<mesh_.points.size(); i++)
    {
      fprintf (f,"%f %f %f ", mesh_.points[i].x, mesh_.points[i].y, mesh_.points[i].z);
      fprintf (f,"\n");
    }
    
   //   fprintf(f,"VERTICES %ld %ld\n", mesh_.points.size(), 2*mesh_.points.size());
//      for (i=0; i<mesh_.points.size(); i++)
//        fprintf(f,"1 %ld\n", i);
    
//     ROS_INFO("vector: %ld, nr: %d  ", triangles.size(), nr_tr);
    
    fprintf(f,"\nPOLYGONS %ld %ld\n", mesh_.triangles.size(), 4*mesh_.triangles.size());
    for (unsigned long i=0; i< mesh_.triangles.size(); i++)
    {
      if ((unsigned long)mesh_.triangles[i].i  >= mesh_.points.size() || mesh_.triangles[i].i < 0 ||  isnan(mesh_.triangles[i].i))
        ;
      else if  ((unsigned long)mesh_.triangles[i].j  >= mesh_.points.size() || mesh_.triangles[i].j < 0 ||  isnan(mesh_.triangles[i].j))
        ;
      else if  ((unsigned long)mesh_.triangles[i].k  >= mesh_.points.size() || mesh_.triangles[i].k < 0 ||  isnan(mesh_.triangles[i].k))
        ;
      else if ( mesh_.triangles[i].i == mesh_.triangles[i].j || mesh_.triangles[i].i == mesh_.triangles[i].k ||
                mesh_.triangles[i].j == mesh_.triangles[i].k)
        ;
      else
        fprintf(f,"3 %d %d %d\n",mesh_.triangles[i].i, mesh_.triangles[i].k, mesh_.triangles[i].j);
    }
    ROS_WARN("Writting to vtk file DONE");
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  // \brief Spin (!)
  bool spin ()
  {
    while(nh_.ok())
    {
      ros::spinOnce();
    }    
    return (true);
  }
};

/* ---[ */
int main (int argc, char** argv)
{
  ros::init (argc, argv, "triangular_mesh_to_vtk");
  ros::NodeHandle n("~");
  TriangularMeshToVTK c(n);
  c.spin ();
  
  return (0);
}
/* ]--- */
