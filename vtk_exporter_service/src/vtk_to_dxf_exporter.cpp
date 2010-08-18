/*
 *  Load and export a VTK 3D point cloud
 *  Copywrong (K) 2010 Z.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * $Id: VTKExporter.cc,v 1.0 2010/02/23 12:00:00 zoli Exp $
 */


//TODO: check what exactly does this mean!!!
volatile bool g_stopall = false;

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

//terminal_tools includes
#include <terminal_tools/print.h>
#include <terminal_tools/parse.h>

//local includes
#include "pcl_cloud_tools/misc.h"
#include "pcl_cloud_tools/dxf_writer.h"



/**
   This is the required type for the save object service
*/
#include <vision_srvs/cop_save.h>
#include <ros/ros.h>

/**
This include is needed by the subscription to a triangle mesh
*/
#include <triangle_mesh_msgs/TriangleMesh.h>


#include <annotation_srvs/TriangleMeshToName.h>
/**
   This contains the ShapeModel Class that takes the dxf file
*/
#define MESHPROCESSING_H
 #include <ShapeModel.h>
/**
   Helper Class that save the stuff to a file
*/
#include <XMLTag.h>



#ifdef WIN32
#include <direct.h>
#define GETCWD(A, B) _getcwd(A, B)
#else
#include <unistd.h>
#define GETCWD(A, B) getcwd(A, B)
#endif

using namespace vision_srvs;

class ObjectSaverService
{
public:
  ros::ServiceServer savesrv;
  ros::NodeHandle* node;

  ObjectSaverService(ros::NodeHandle *nodeptr) :
    node(nodeptr)
  {

     printf("advertiseService<cop_save> (%s, ...)\n", node->resolveName("save").c_str());
    savesrv = node->advertiseService(node->resolveName("save"), &ObjectSaverService::SaveCallBack, this);

     char cwd[2048];
     char* cwdptr;
     std::ostringstream os;
     cwdptr = GETCWD(cwd, 2048);
     os << cwd << "/";
     m_targetXMLFilename = os.str();

     /**
        TOOD subscribe to topic with meshdata + some metas
     */
   }

  /**
     @param objectType         annotation for this model (like our famous "IceTea")
     @param dxfFilename        Absolute path to the dxffile
     @param targetXMLFilename  name where the xml file should be written.

  */
   bool write_shape_model(std::string objectType,
                                     std::string dxfFilename,
                                     std::string targetXMLFilename,
                                     cop_save::Response&  answer)
  {
    try
    {
      using namespace cop;

      /** Create the ShapeModel with an annotation "objectType" */
      ShapeModel *sm = new ShapeModel(new Class(objectType, 10000001));
      sm->m_initializationLevel = 1.0;

      /** Add the dxf file */
      ShapeModelParamSet* pm = new ShapeModelParamSet(NULL,0.0,0.0,0.0);
      sm->SetShapeModelParamSet(pm, dxfFilename, false);

      /** Save everything to a file*/
      XMLTag* tag = sm->Save();
      tag->WriteToFile(targetXMLFilename);

      answer.xmlfilename = targetXMLFilename;
      answer.filenames =  tag->GetSubFilenames();

    }
    catch(const char *text)
    {
      ROS_ERROR("Failed to save an object: %s", text);
      return false;
    }
    catch(...)
    {
      ROS_ERROR("Failed to save an object");
      return false;
    }
    /** This messages should be passed to /cop/new_signatures */
    return true;
  }

  bool SaveCallBack(cop_save::Request& msg, cop_save::Response&  answer)
  {
    if(m_idList.find(msg.object_id) != m_idList.end())
    {
      std::string name = m_idList[msg.object_id];
      std::ostringstream os, oss;
      os << m_targetXMLFilename << name << "_model" << ".dxf";
      oss << m_targetXMLFilename << name << "_model_description" << ".xml";
      std::string dxfFilename = os.str();
      dxfwriter::WriteMesh (m_meshList[name], dxfFilename);
      write_shape_model(name, dxfFilename, oss.str(), answer);
    }
    return true;
  }

  void TriMeshCallback(triangle_mesh_msgs::TriangleMesh::ConstPtr meshIn)
  {
    std::string serviceName = "/annotator/ask_for_mesh";
    while(node->ok())
    {
      if(ros::service::waitForService(serviceName, 1000))
        break;
      else
      {
        ros::Rate rate(0.5);
        rate.sleep();
      }
    }
    ros::ServiceClient client = node->serviceClient<annotation_srvs::TriangleMeshToName>(serviceName, true);
    annotation_srvs::TriangleMeshToName query;
    query.request.mesh  = *meshIn;
    if(!client.call(query))
    {
       return;
    }

    Mesh_t meshOut;
    for(int i = 0; i < meshIn->triangles.size(); i++)
    {
      Polygon_t poly;

      Point3d_t p, q, r;
      p.x = meshIn->points[meshIn->triangles[i].i].x;
      p.y = meshIn->points[meshIn->triangles[i].i].y;
      p.z = meshIn->points[meshIn->triangles[i].i].z;
      poly.push_back(p);
      q.x = meshIn->points[meshIn->triangles[i].j].x;
      q.y = meshIn->points[meshIn->triangles[i].j].y;
      q.z = meshIn->points[meshIn->triangles[i].j].z;
      poly.push_back(q);
      r.x = meshIn->points[meshIn->triangles[i].k].x;
      r.y = meshIn->points[meshIn->triangles[i].k].y;
      r.z = meshIn->points[meshIn->triangles[i].k].z;
      poly.push_back(r);
      meshOut.push_back(std::pair<Polygon_t, Polygon_t>(poly, poly));
    }
    m_meshList[query.response.name] = meshOut;
    m_idList[8000000] = query.response.name;
  }

  std::map<std::string, Mesh_t> m_meshList;
  std::map<ObjectID_t, std::string> m_idList;

  std::string  m_targetXMLFilename;
};

using namespace std;
using terminal_tools::print_color;
using terminal_tools::print_error;
using terminal_tools::print_warn;
using terminal_tools::print_info;
using terminal_tools::print_debug;
using terminal_tools::print_value;
using terminal_tools::print_highlight;
using terminal_tools::TT_BRIGHT;
using terminal_tools::TT_RED;
using terminal_tools::TT_GREEN;
using terminal_tools::TT_BLUE;

/* ---[ */
int
  main (int argc, char** argv)
{

  // Parse the command line arguments for .vtk or .ply files

  std::string nodename ="vtk_to_dxf_srv";
  std::string topic_name = "/topicname";
  ros::init(argc, argv, nodename);
  ros::NodeHandle nh(nodename);

  ObjectSaverService service(&nh);
  /* TODO select a topic and */
  ros::Subscriber subs =
         nh.subscribe<triangle_mesh_msgs::TriangleMesh>(
                nh.resolveName(topic_name), 1000,
                 boost::bind(&ObjectSaverService::TriMeshCallback, &service, _1));
  std::string test = subs.getTopic();
  /**
    till here
  */
  ros::spin();
}
/* ]--- */
