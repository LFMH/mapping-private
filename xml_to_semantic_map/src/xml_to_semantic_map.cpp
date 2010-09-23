#include <cstdlib>
#include <vector>
#include <sstream>
#include <ros/ros.h>
#include "xml_to_semantic_map/xml_semantic_map_parser.h"
#include "mod_semantic_map/SemMap.h"

// Eigen
#include <Eigen/Core>
// if needed:
#include <Eigen/LU> // matrix inversion
#include <Eigen/Geometry> // cross product

int main(int argc, char **argv)
{
  // Getting parameters
  if (argc < 2)
  {
    std::cerr << "USAGE: " << argv[0] << " <xml-file> <publishing-rate>" << std::endl;
    return -1;
  }
  std::stringstream ss (argv[2]);
  double rate = 0;
  ss >> rate;

  // ROS stuff
  ros::init (argc, argv, "xml_to_semantic_map");
  ros::NodeHandle nh ("~");
  ros::Publisher semantic_map_pub = nh.advertise<mod_semantic_map::SemMap> ("semantic_map", 1);
  ros::Rate loop_rate (rate);

  // Spin at least once
  int count = 0;
  while ((ros::ok ()) && (count == 0 || rate != 0))
  {
    ++count;
    ROS_INFO("Publishing map from %s (rate: %g, count: %d)", argv[1], rate, count);

    // Create transformation between the coordinate frame of the XML and that of /map
    Eigen::Matrix4f map_frame = Eigen::Matrix4f::Identity ();
    ROS_INFO("Applying the following transformation to the map:");
    std::cerr << map_frame << std::endl;

    // Parse XML file
    SemanticMap map_xml;
    if (parseXML (argv[1], map_xml)) /// @NOTE: this sets the global smap variable declared in xml_semantic_map_parser.h
    {
      ROS_INFO("Loaded %ld candidates, %ld handles, %ld knobs and %ld walls", map_xml.candidates.size (), map_xml.handles.size (), map_xml.knobs.size (), map_xml.walls.size ());
      ros::Time ts = ros::Time::now ();

      // Creating the list of objects from the file
      int id_cnt = 0;
      mod_semantic_map::SemMap map_msg;
      //std::vector<mod_semantic_map::SemMapObject> objects;
      std::map<int, int> candidate_door;
      for (std::vector<Candidate>::iterator cit = map_xml.candidates.begin (); cit != map_xml.candidates.end (); cit++)
      {
        mod_semantic_map::SemMapObject obj;

        // adding door first
        obj.partOf = cit->id;
        obj.id = id_cnt++;
        obj.type = "door";
        obj.pose.resize (16);
        Eigen::Map<Eigen::Matrix4f> final_pose (&(obj.pose[0])); // map an eigen matrix onto the vector
        Eigen::Vector3f fp0 = Eigen::Map<Eigen::Vector3d> (cit->front.p0).cast<float> ();
        Eigen::Vector3f fp1 = Eigen::Map<Eigen::Vector3d> (cit->front.p1).cast<float> ();
        //Eigen::Vector3f fp2 = Eigen::Map<Eigen::Vector3d> (cit->front.p2).cast<float> ();
        Eigen::Vector3f fp3 = Eigen::Map<Eigen::Vector3d> (cit->front.p3).cast<float> ();
        Eigen::Vector3f bp0 = Eigen::Map<Eigen::Vector3d> (cit->back.p0).cast<float> ();
        //Eigen::Vector3f bp1 = Eigen::Map<Eigen::Vector3d> (cit->back.p1).cast<float> ();
        //Eigen::Vector3f bp2 = Eigen::Map<Eigen::Vector3d> (cit->back.p2).cast<float> ();
        //Eigen::Vector3f bp3 = Eigen::Map<Eigen::Vector3d> (cit->back.p3).cast<float> ();
        Eigen::Vector3f a = bp0 - fp0;
        Eigen::Vector3f b = fp3 - fp0;
        Eigen::Vector3f c = fp1 - fp0;
        obj.depth = 0.01;
        obj.width = b.norm ();
        obj.height = c.norm ();
        c /= obj.height;
        Eigen::Vector3f v = c.cross (b).normalized ();
        Eigen::Vector3f u = v.cross (c);
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity ();
        pose.block<3,1>(0,0) = u;
        pose.block<3,1>(0,1) = v;
        pose.block<3,1>(0,2) = c;// / obj.height;
        pose.block<3,1>(0,3) = fp0 + u*obj.width/2 + c*obj.height/2;
        final_pose.noalias() = (pose * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << obj.type << std::endl << final_pose.transpose () << std::endl;
        map_msg.objects.push_back (obj);

        // saving the door's id belonging to thei candiate
        candidate_door[obj.partOf] = obj.id;

        // now adding candidate
        obj.partOf = 0;
        obj.id = cit->id;
        obj.type = getTypeName (cit->type);
        obj.depth = a.norm ();
        pose.block<3,1>(0,3) += v*obj.depth/2;
        final_pose.noalias() = (pose * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << cit->name << std::endl << final_pose.transpose () << std::endl;
        map_msg.objects.push_back (obj);
      }
      for (std::vector<Handle>::iterator hit = map_xml.handles.begin (); hit != map_xml.handles.end (); hit++)
      {
        mod_semantic_map::SemMapObject obj;
        obj.partOf = candidate_door[hit->doorID];
        obj.id = hit->id;
        obj.type = "handle";
        obj.depth = hit->elongation[0];
        obj.width = hit->elongation[1];
        obj.height = hit->elongation[2];
        obj.pose.resize (16);
        Eigen::Map<Eigen::Matrix4f> final_pose (&(obj.pose[0])); // map an eigen matrix onto the vector
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity ();
        pose.block<3,1>(0,3) = Eigen::Map<Eigen::Vector3d> (map_xml.candidates[obj.partOf].front.p0).cast<float> () + Eigen::Map<Eigen::Vector3d> (hit->center).cast<float> ();
        final_pose.noalias() = (pose * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << obj.id << "/" << obj.partOf << std::endl << final_pose.transpose () << std::endl;
        map_msg.objects.push_back (obj);
      }
      for (std::vector<Knob>::iterator kit = map_xml.knobs.begin (); kit != map_xml.knobs.end (); kit++)
      {
        mod_semantic_map::SemMapObject obj;
        obj.partOf = candidate_door[kit->doorID];
        obj.id = kit->id;
        obj.type = "knob";
        obj.depth = kit->radius;
        obj.width = kit->radius;
        obj.height = kit->radius;
        obj.pose.resize (16);
        Eigen::Map<Eigen::Matrix4f> final_pose (&(obj.pose[0])); // map an eigen matrix onto the vector
        Eigen::Matrix4f pose = Eigen::Matrix4f::Identity ();
        pose.block<3,1>(0,3) = Eigen::Map<Eigen::Vector3d> (map_xml.candidates[obj.partOf].front.p0).cast<float> () + Eigen::Map<Eigen::Vector3d> (kit->center).cast<float> ();
        final_pose.noalias() = (pose * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << obj.id << "/" << obj.partOf << std::endl << final_pose.transpose () << std::endl;
        map_msg.objects.push_back (obj);
      }

      // Creating the map message and publishing it
      ROS_INFO ("Generated data for message in %g seconds.", (ros::Time::now () - ts).toSec ());
      map_msg.header.frame_id = "/map";
      map_msg.header.stamp = ros::Time::now ();
      semantic_map_pub.publish (map_msg);
    }
    else
      ROS_ERROR ("Could not read a valid semantic map from %s!", argv[1]);

    // Spin
    ros::spinOnce();
    if (rate)
      loop_rate.sleep();
  }

  return 0;
}
