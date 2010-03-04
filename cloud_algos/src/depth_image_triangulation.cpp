#include <cloud_algos/cloud_algos.h>
#include <cloud_algos/depth_image_triangulation.h>
#include <stdio.h>
#include <stdlib.h>
#include <float.h>
#include <iostream>
#include <math.h>
#include <map>
//for savePCDFile ()
#include <point_cloud_mapping/cloud_io.h>

//#define DEBUG 1
using namespace cloud_algos;

////////////////////////////////////////////////////////////////////////////////
void DepthImageTriangulation::get_scan_and_point_id (const boost::shared_ptr<const InputType>& cloud_in)
{
  cloud_with_line_ = *cloud_in;
  int scan_id = 0;
  int point_id = 0;
  int temp_point_id = 0;
  ros::Time ts = ros::Time::now ();
  cloud_with_line_.channels.resize(cloud_with_line_.channels.size()+1);
  cloud_with_line_.channels[cloud_with_line_.channels.size()-1].name = "line";
  for (unsigned int j = 0; j < cloud_with_line_.channels.size(); j++)
  {
    if ( cloud_with_line_.channels[j].name == "index")
    {
      cloud_with_line_.channels[j+1].values.resize(cloud_with_line_.channels[j].values.size());
      for (unsigned int k = 0; k < cloud_with_line_.channels[j].values.size()-1; k++)
      {
        point_id = cloud_with_line_.channels[j].values[k];
        temp_point_id =  cloud_with_line_.channels[j].values[k+1];
        cloud_with_line_.channels[j+1].values[k] = scan_id;
        //new line found
        if (temp_point_id < point_id)
        {
          scan_id++;
          //find max point index  in the whole point cloud
          if (point_id > max_index_)
            max_index_ = point_id;
        }
      }
    }
  }
  //nr of lines in point cloud
  max_line_ = scan_id;
  //cloud_io::savePCDFile ("cloud_line.pcd", cloud_with_line_, false);
  ROS_INFO("Nr lines: %d, Max point ID: %d Completed in %f seconds", max_line_, max_index_,  (ros::Time::now () - ts).toSec ());
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
  ROS_INFO("DepthImageTriangulation Node initialized");
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
    
  ROS_INFO("PointCloud msg with size %ld received", cloud_in->points.size());
  {
    boost::mutex::scoped_lock lock(cloud_lock_);
    //cloud_in gets processed and copied into cloud_with_line_;
    get_scan_and_point_id(cloud_in);
  }
  if (cloud_with_line_.channels.size() < 2 || cloud_with_line_.channels[0].name != "index" || 
       cloud_with_line_.channels[1].name != "line")
  {
    ROS_ERROR("Clouds's channel[0] and channel[1] must be index (index) in one line scan and line's id (line) respectively!!!"); 
    exit(2);
  }
  ros::Time ts = ros::Time::now ();
  std::vector<triangle> tr;
  tr.resize(2*max_line_*max_index_);    
  //pmap_.polygons.resize(2*max_line_*max_index_);
  mesh_pub_.points.resize(2*max_line_*max_index_);
  mesh_pub_.triangles.resize(2*max_line_*max_index_);

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
      if (cloud_with_line_.channels[1].values[a] == i && cloud_with_line_.channels[0].values[a] == j)
      {
        //ROS_INFO("found point a = %d\n", a);
        b = -1;
        c = -1;
        d = -1;
        e = -1;
          
        // find top right corner
        if (a+1 < cloud_with_line_.points.size() && cloud_with_line_.channels[1].values[a+1] == i
                   && cloud_with_line_.channels[0].values[a+1] == j+1)
          b = a+1;
          
        //ROS_INFO("resolved point b = %d\n", b);
          
        // go to next line
        int test = a;
        while ((unsigned long)test < cloud_with_line_.points.size() &&  cloud_with_line_.channels[1].values[test] < i+1)
          test++;
          
        //ROS_INFO("resolved next line\n");
          
        // if next line exists
        if ((unsigned long)test < cloud_with_line_.points.size() && cloud_with_line_.channels[1].values[test] == i+1)
        {
          // a skipped triangle exists because of missing 'a'
          if (skipped)
          {
            skipped = false; // reset var
              
            // go to column j-1
            while ((unsigned long)test <  cloud_with_line_.points.size() &&  cloud_with_line_.channels[0].values[test] < j-1)
              test++;
              
            // if not at the end of dataset
            if ((unsigned long)test <  cloud_with_line_.points.size())
            {
              // if column exists
              if (cloud_with_line_.channels[1].values[test] == i+1 && cloud_with_line_.channels[0].values[test])
              {
                e = test;
                test++;
              }
            }
          }
          else
          {
            // go to column j
            while ((unsigned long)test < cloud_with_line_.points.size() && cloud_with_line_.channels[0].values[test] < j)
              test++;
          }
            
          // if not at the end of dataset
          if ((unsigned long)test < cloud_with_line_.points.size())
          {
            // if column exists
            if (cloud_with_line_.channels[1].values[test] == i+1 && cloud_with_line_.channels[0].values[test] == j)
            {
              c = test;
              if ((unsigned long)c+1 < cloud_with_line_.points.size() && cloud_with_line_.channels[1].values[c+1] == i+1
                  && cloud_with_line_.channels[0].values[c+1] == j+1)
                d = c+1;
            }
            // if next column was found
            else if (cloud_with_line_.channels[1].values[test] == i+1 && cloud_with_line_.channels[0].values[test] == j+1)
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
  geometry_msgs::Polygon poly;
  poly.points.resize(3);
  //pmap_.header = cloud_with_line_.header;
  mesh_pub_.header = cloud_with_line_.header;
  
  geometry_msgs::Point32 tr_i, tr_j, tr_k;
  ias_table_msgs::Triangle tr_mesh;
#ifdef DEBUG  
  ROS_INFO("Triangle a: %d, b: %d, c: %d", tr[i].a, tr[i].b, tr[i].c);
#endif
  for (unsigned long i = 0; i < tr.size(); i++)
  {
    //a
    poly.points[0].x = cloud_with_line_.points[tr[i].a].x;
    poly.points[0].y = cloud_with_line_.points[tr[i].a].y;
    poly.points[0].z = cloud_with_line_.points[tr[i].a].z;

    tr_i.x = cloud_with_line_.points[tr[i].a].x;
    tr_i.y = cloud_with_line_.points[tr[i].a].y;
    tr_i.z = cloud_with_line_.points[tr[i].a].z;
    tr_mesh.i.data = tr[i].a;
    //b
    poly.points[1].x = cloud_with_line_.points[tr[i].b].x;
    poly.points[1].y = cloud_with_line_.points[tr[i].b].y;
    poly.points[1].z = cloud_with_line_.points[tr[i].b].z;

    tr_j.x = cloud_with_line_.points[tr[i].b].x;
    tr_j.y = cloud_with_line_.points[tr[i].b].y;
    tr_j.z = cloud_with_line_.points[tr[i].b].z;
    tr_mesh.j.data = tr[i].b;

    //c
    poly.points[2].x = cloud_with_line_.points[tr[i].c].x;
    poly.points[2].y = cloud_with_line_.points[tr[i].c].y;
    poly.points[2].z = cloud_with_line_.points[tr[i].c].z;

    tr_k.x = cloud_with_line_.points[tr[i].c].x;
    tr_k.y = cloud_with_line_.points[tr[i].c].y;
    tr_k.z = cloud_with_line_.points[tr[i].c].z;
    tr_mesh.k.data = tr[i].c;

    //pmap_.polygons[i] = poly;
    mesh_pub_.triangles.push_back(tr_mesh);
    mesh_pub_.points.push_back(tr_i);
    mesh_pub_.points.push_back(tr_j);
    mesh_pub_.points.push_back(tr_k);
  }
  //pmap_.polygons.resize(nr);
  mesh_pub_.triangles.resize(nr);
  mesh_pub_.points.resize(nr*3);
  if(write_to_vtk_)
    write_vtk_file("triangles.vtk", tr, cloud_with_line_, nr);
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
  
  for (i=0; i<cloud_with_line_.points.size(); i+=3)
  {
    for (unsigned long j=0; (j<3) && (i+j<cloud_with_line_.points.size()); j++)
      fprintf (f,"%f %f %f ", cloud_with_line_.points[i+j].x, cloud_with_line_.points[i+j].y,
               cloud_with_line_.points[i+j].z);
    fprintf (f,"\n");
  }
  
  fprintf(f,"VERTICES %ld %ld\n", cloud_with_line_.points.size(), 2*cloud_with_line_.points.size());
  for (i=0; i<cloud_with_line_.points.size(); i++)
    fprintf(f,"1 %ld\n", i);
    
  /*
  fprintf(f,"\nPOLYGONS %d %d\n",triangles.size(), 4*triangles.size());
  for (std::vector<triangle>::iterator it = triangles.begin(); it != triangles.end(); it++)
  fprintf(f,"3 %d %d %d\n",(*it).a, (*it).c, (*it).b);
  */

  printf("vector: %ld, nr: %d\n", triangles.size(), nr_tr);
  
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

//   fprintf (f, "\nPOINT_DATA %ld\nSCALARS scalars double\nLOOKUP_TABLE default\n", cloud_with_line_.points.size());
//   for (i=0; i<cloud_with_line_.points.size(); i+=9)
//   {
//     for (int j=0; (j<9) && (i+j<cloud_with_line_.points.size()); j++)
//       fprintf (f, "%d ", (int)cloud_with_line_.channels[0].values[i+j]);
//     fprintf (f,"\n");
//   }
}

////////////////////////////////////////////////////////////////////////////////
DepthImageTriangulation::OutputType DepthImageTriangulation::output ()
  {
    //return pmap_;
    return mesh_pub_;
  }


#ifdef CREATE_NODE
int main (int argc, char* argv[])
{
  return standalone_node <DepthImageTriangulation> (argc, argv);
}
#endif


