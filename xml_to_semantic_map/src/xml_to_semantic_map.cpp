#include <cstdlib>
#include <vector>
#include <sstream>
#include <fstream>
#include <ros/ros.h>
#include "xml_to_semantic_map/xml_semantic_map_parser.h"
#include "mod_semantic_map/SemMap.h"
#include "mod_semantic_map/GenerateSemanticMapOWL.h"

// Eigen
#include <Eigen3/Core>
// if needed:
#include <Eigen3/LU> // matrix inversion
#include <Eigen3/Geometry> // cross product

int main(int argc, char **argv)
{
  // Getting parameters
  if (argc < 4)
  {
    std::cerr << "USAGE: " << argv[0] << " <xml-file> <publishing-rate> <owl-file>" << std::endl << "NOTE: owl file is written only once (attempts until successful), and if rate is 0 only one publish and owl conversion request is tried" << std::endl;
    return -1;
  }
  std::stringstream ss (argv[2]);
  double rate = 0;
  ss >> rate;
  std::string fileName (argv[3]);

  // ROS stuff
  ros::init (argc, argv, "xml_to_semantic_map");
  ros::NodeHandle nh ("~");
  ros::Publisher semantic_map_pub = nh.advertise<mod_semantic_map::SemMap> ("semantic_map", 1);
  ros::Rate loop_rate (rate);

  // Spin at least once
  int count = 0;
  bool owl_saved = false;
  while ((ros::ok ()) && (count == 0 || rate != 0))
  {
    ++count;
    ROS_INFO("Loading map from %s (rate: %g, count: %d)", argv[1], rate, count);

    // Create transformation between the coordinate frame of the XML and that of /map
    Eigen3::Matrix4f map_frame = Eigen3::Matrix4f::Identity ();
    ROS_INFO("Applying the following transformation to the map:");
    std::cerr << map_frame << std::endl;

    // Parse XML file
    SemanticMap map_xml;
    if (parseXML (argv[1], map_xml)) /// @NOTE: this sets the global smap variable declared in xml_semantic_map_parser.h
    {
      ROS_INFO("Loaded %ld candidates, %ld handles, %ld knobs and %ld planes", map_xml.candidates.size (), map_xml.handles.size (), map_xml.knobs.size (), map_xml.planes.size ());
      ros::Time ts = ros::Time::now ();

      // Creating the list of objects from the file
      int id_cnt = 1; /// @NOTE: the map itself has ID 0
      mod_semantic_map::SemMap map_msg;
      //std::vector<mod_semantic_map::SemMapObject> objects;
      std::map<int, int> candidate_door;
      std::vector<Eigen3::Vector3f> min_front_points;
      std::vector<Eigen3::Matrix4f> poses;
      for (std::vector<Candidate>::iterator cit = map_xml.candidates.begin (); cit != map_xml.candidates.end (); cit++)
      {
        // getting the corner points
        Eigen3::MatrixXf fp (3,4);
        fp.col(0) = Eigen3::Map<Eigen3::Vector3d> (cit->front.p0).cast<float> ();
        fp.col(1) = Eigen3::Map<Eigen3::Vector3d> (cit->front.p1).cast<float> ();
        fp.col(2) = Eigen3::Map<Eigen3::Vector3d> (cit->front.p2).cast<float> ();
        fp.col(3) = Eigen3::Map<Eigen3::Vector3d> (cit->front.p3).cast<float> ();
//        Eigen3::Vector3f fp0 = Eigen3::Map<Eigen3::Vector3d> (cit->front.p0).cast<float> ();
//        Eigen3::Vector3f fp1 = Eigen3::Map<Eigen3::Vector3d> (cit->front.p1).cast<float> ();
//        //Eigen3::Vector3f fp2 = Eigen3::Map<Eigen3::Vector3d> (cit->front.p2).cast<float> ();
//        Eigen3::Vector3f fp3 = Eigen3::Map<Eigen3::Vector3d> (cit->front.p3).cast<float> ();
        Eigen3::Vector3f bp0 = Eigen3::Map<Eigen3::Vector3d> (cit->back.p0).cast<float> ();
        //Eigen3::Vector3f bp1 = Eigen3::Map<Eigen3::Vector3d> (cit->back.p1).cast<float> ();
        //Eigen3::Vector3f bp2 = Eigen3::Map<Eigen3::Vector3d> (cit->back.p2).cast<float> ();
        //Eigen3::Vector3f bp3 = Eigen3::Map<Eigen3::Vector3d> (cit->back.p3).cast<float> ();

        // saving the row-wise minimum coordinate for future use of by the fixtures
        min_front_points.push_back (fp.col(0).array().min(fp.col(1).array()).min(fp.col(2).array()).min(fp.col(3).array()));
        //std::cerr << "points:" << std::endl << fp << std::endl << "min:" << std::endl << min_front_points[obj_door.id-1] << std::endl;

        // setting up door parameters first
        mod_semantic_map::SemMapObject obj_door;
        obj_door.partOf = cit->id;
        obj_door.id = id_cnt++;
        obj_door.type = "door";
        obj_door.pose.resize (16);
        Eigen3::Map<Eigen3::Matrix4f> final_pose (&(obj_door.pose[0])); // map an eigen matrix onto the vector
        Eigen3::Vector3f a = bp0 - fp.col(0);
        Eigen3::Vector3f b = fp.col(3) - fp.col(0);
        Eigen3::Vector3f c = fp.col(1) - fp.col(0);
        obj_door.depth = 0.01;
        obj_door.width = b.norm ();
        obj_door.height = c.norm ();
        c /= obj_door.height;
        Eigen3::Vector3f u = b.cross (c).normalized ();
        Eigen3::Vector3f v = c.cross (u);
        Eigen3::Matrix4f pose = Eigen3::Matrix4f::Identity ();
        pose.block<3,1>(0,0) = u;
        pose.block<3,1>(0,1) = v;
        pose.block<3,1>(0,2) = c;// / obj.height;
        pose.block<3,1>(0,3) = fp.col(0) + v*obj_door.width/2 + c*obj_door.height/2;
        final_pose.noalias() = (pose * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << obj.type << std::endl << final_pose.transpose () << std::endl;

        // saving the door's id belonging to the candidate
        candidate_door[obj_door.partOf] = obj_door.id;

        // saving the pose for future use of by the fixtures
        poses.push_back (pose);

        // now adjusting it to obtain candidate parameters
        mod_semantic_map::SemMapObject obj = obj_door;
        obj.partOf = 0;
        obj.id = cit->id;
        obj.type = getTypeName (cit->type);
        obj.depth = a.norm ();
        if (a.dot (u) > 0)
          pose.block<3,1>(0,3) += u*obj.depth/2;
        else
          pose.block<3,1>(0,3) -= u*obj.depth/2;
        final_pose.noalias() = (pose * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << cit->name << std::endl << final_pose.transpose () << std::endl;

        // adding candidate first then door to preserve hierarchy
        map_msg.objects.push_back (obj);
        map_msg.objects.push_back (obj_door);
      }
      for (std::vector<Handle>::iterator hit = map_xml.handles.begin (); hit != map_xml.handles.end (); hit++)
      {
        mod_semantic_map::SemMapObject obj;
        obj.partOf = candidate_door[hit->doorID];
        obj.id = hit->id;
        obj.type = "handle";
        Eigen3::Map<Eigen3::Vector3d> e (&(hit->elongation[0]));
        Eigen3::Vector3f d = (poses[obj.partOf-1].topLeftCorner<3,3> () * e.cast<float> ()).array ().abs ();
        //std::cerr << d.transpose () << std::endl;
        obj.depth = d[0];
        obj.width = d[1];
        obj.height = d[2];
        obj.pose.resize (16);
        Eigen3::Map<Eigen3::Matrix4f> final_pose (&(obj.pose[0])); // map an eigen matrix onto the vector
        //Eigen3::Matrix4f pose = Eigen3::Matrix4f::Identity ();
        //poses[obj.partOf-1].topLeftCorner<3,3>() = Eigen3::Matrix4f::Identity ();
        poses[obj.partOf-1].block<3,1>(0,3) = min_front_points[obj.partOf-1] + Eigen3::Map<Eigen3::Vector3d> (hit->center).cast<float> ();
        final_pose.noalias() = (poses[obj.partOf-1] * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << obj.id << "/" << obj.partOf << std::endl << final_pose.transpose () << std::endl;
        map_msg.objects.push_back (obj);
      }
      for (std::vector<Knob>::iterator kit = map_xml.knobs.begin (); kit != map_xml.knobs.end (); kit++)
      {
        mod_semantic_map::SemMapObject obj;
        obj.partOf = candidate_door[kit->doorID];
        obj.id = kit->id;
        obj.type = "knob";
        obj.depth = 2*kit->radius;
        obj.width = 2*kit->radius;
        obj.height = 2*kit->radius;
        obj.pose.resize (16);
        Eigen3::Map<Eigen3::Matrix4f> final_pose (&(obj.pose[0])); // map an eigen matrix onto the vector
        //Eigen3::Matrix4f pose = Eigen3::Matrix4f::Identity ();
        //poses[obj.partOf-1].topLeftCorner<3,3>() = Eigen3::Matrix4f::Identity ();
        poses[obj.partOf-1].block<3,1>(0,3) = min_front_points[obj.partOf-1] + Eigen3::Map<Eigen3::Vector3d> (kit->center).cast<float> ();
        final_pose.noalias() = (poses[obj.partOf-1] * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
        //std::cerr << obj.id << "/" << obj.partOf << std::endl << final_pose.transpose () << std::endl;
        map_msg.objects.push_back (obj);
      }
      for (std::vector<Plane>::iterator pit = map_xml.planes.begin (); pit != map_xml.planes.end (); pit++)
      {
        if (getTypeName (checkObjectClass (pit->id)) == "horizontal")
        {
          mod_semantic_map::SemMapObject obj;
          obj.partOf = 0;
          obj.id = pit->id;
          obj.type = "horizontal_plane";
          Eigen::Vector3f minD = Eigen::Map<Eigen::Vector3d> (pit->minD).cast<float> ();
          Eigen::Vector3f maxD = Eigen::Map<Eigen::Vector3d> (pit->maxD).cast<float> ();
          std::cerr << minD.transpose () << std::endl << maxD.transpose () << std::endl;
          obj.depth = maxD[0] - minD[0];
          obj.width = maxD[1] - minD[1];
          obj.height = 0.02;
          obj.pose.resize (16);
          Eigen::Map<Eigen::Matrix4f> final_pose (&(obj.pose[0])); // map an eigen matrix onto the vector
          Eigen::Matrix4f pose = Eigen::Matrix4f::Identity ();
          pose.block<3,1>(0,3) = minD + (maxD - minD)/2;
          final_pose.noalias() = (pose * map_frame).transpose (); /// @NOTE: the mapped vector holds the matrices transposed!
          std::cerr << obj.id << ": " << obj.depth << " " << obj.width << " " << obj.height << std::endl << final_pose.transpose () << std::endl;
          map_msg.objects.push_back (obj);
        }
      }
      ROS_INFO ("Generated data for message in %g seconds.", (ros::Time::now () - ts).toSec ());

      // Creating the map message and publishing it
      map_msg.header.frame_id = "/map";
      map_msg.header.stamp = ros::Time::now ();
      semantic_map_pub.publish (map_msg);
      ROS_INFO ("Published message on the %s topic.", semantic_map_pub.getTopic ().c_str ());

      // Requesting OWL conversion and saving it (if not succeeded already)
      if (!owl_saved)
      {
        mod_semantic_map::GenerateSemanticMapOWL srv;
        srv.request.map = map_msg;
        ros::ServiceClient client = nh.serviceClient<mod_semantic_map::GenerateSemanticMapOWL>("/generate_owl_map");
        if (client.call(srv))
        {
          ROS_INFO ("Received OWL file with %ld characters.", srv.response.owlmap.size ());
          std::ofstream fs;
          fs.open (fileName.c_str ());
          if (fs.is_open ())
          {
            fs << srv.response.owlmap;
            fs.close ();
            ROS_INFO ("OWL file written to %s", fileName.c_str ());
            owl_saved = true;
          }
          else
            ROS_ERROR ("Couldn't open %s for reading!", fileName.c_str ());
        }
        else
          ROS_ERROR("Failed to call service mod_semantic_map::GenerateSemanticMapOWL");
      }
      else
        ROS_WARN ("OWL file was already requested and written to %s ...", fileName.c_str ());
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
