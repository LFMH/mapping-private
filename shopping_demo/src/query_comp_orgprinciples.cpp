#include <string>
#include <iostream>

#include <unistd.h>

#include <ros/ros.h>
#include <json_prolog/prolog.h>
#include <shopping_demo/QueryBestObjLocation.h>

#include "rospack/rospack.h"


using namespace std;
using namespace json_prolog;

class QueryOrgPrinciples
{
protected:
  ros::NodeHandle nh_;
  ros::ServiceServer service_;

public:      
  QueryOrgPrinciples(ros::NodeHandle &nh) : nh_ (nh)
  {
    service_ = nh_.advertiseService("query", &QueryOrgPrinciples::queryCB, this);
    ROS_INFO ("[QueryOrgPrinciples] Advertising service on: %s", service_.getService ().c_str ());
  }

  
  bool queryCB (shopping_demo::QueryBestObjLocation::Request  &req,
		shopping_demo::QueryBestObjLocation::Response &res)
  {
    //classifiers: best_location_maxMaxWup and best_location_dtree
    //best_location_maxMaxWup may return multiple solutions (if they
    //have equal similarities)

    Prolog pl;
    //initialize visualization:
    pl.query("register_ros_package(mod_vis)");
    pl.query("use_module(library('mod_vis'))");
    pl.query("mod_vis:visualisation_canvas(C)");
    //wait until it is initialized
    sleep(5);
    
    string object = req.name_space+":'"+req.object_type+"'";
    string classifier = "best_location_maxMaxWup"; //best_location_dtree

    string query = "highlight_" + classifier + "(" + object + ", _)";
    string query2 = classifier + "(" + object + ", L)";
    ROS_INFO("Query highlight: %s", query.c_str());
    ROS_INFO("Query: %s", query2.c_str());

    pl.query(query);
    PrologQueryProxy bdgs =     pl.query(query2);
    string objectLocation = "";
    for(PrologQueryProxy::iterator it=bdgs.begin();
  	it != bdgs.end(); it++)
      {
  	PrologBindings bdg = *it;
  	//cout << "Found solution: " << (bool)(it == bdgs.end()) << endl;
	res.location.push_back(bdg["L"]);
  	ROS_INFO_STREAM("Location = "<< bdg["L"]);
	objectLocation = bdg["L"].toString();
      }
    //    sleep(10);    


// Logger
	rospack::ROSPack rp;
	char *p[] = {"rospack", "find", "shopping_demo"};
	rp.run(3, p);
 //ROS_INFO("ERROOORR %s",p.c_str());

  //rp.run(p);
	char folderName[300];
	strcpy(folderName, rp.getOutput().c_str());
	folderName[strlen(folderName)-1] = 0;

	ROS_INFO_STREAM("Foldername" << folderName);
	//display object images:
    string queryDisplayImages = "display_object_images_at_location('" + objectLocation + "','" + folderName +"')";
       //cout << queryDisplayImages << endl;
    pl.query(queryDisplayImages);

    return true;
  }
};


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "best_object_location");
  ros::NodeHandle n("~");
  QueryOrgPrinciples qop(n);
  ros::spin ();  
  return 0;
}
