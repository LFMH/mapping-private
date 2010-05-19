// #include <unistd.h>

#include <map>
#include <ctime>
#include <ros/node_handle.h>
#include <triangle_mesh/TriangleMesh.h>
#include <ias_table_msgs/TableWithObjects.h>
#include <ias_table_msgs/PrologReturn.h>
#include <ias_table_srvs/ias_table_clusters_service.h>
#include <ias_table_srvs/ias_reconstruct_object.h>
#include <point_cloud_mapping/cloud_io.h>
#include <point_cloud_mapping/geometry/areas.h>
#include <point_cloud_mapping/geometry/statistics.h>
#include <mapping_msgs/PolygonalMap.h>
#include <geometry_msgs/Polygon.h>

#include <position_string_rviz_plugin/PositionStringList.h>

// cloud_algos plugin stuff
#include <cloud_algos/rotational_estimation.h>
#include <cloud_algos/depth_image_triangulation.h>
#include <cloud_algos/mls_fit.h>
#include <cloud_algos/cylinder_fit_algo.h>
#include <cloud_algos/box_fit_algo.h>
#include <cloud_algos/box_fit2_algo.h>
#include <cloud_algos/cloud_algos.h>
#include <pluginlib/class_loader.h>

// COP/JLO stuff
#include <vision_srvs/srvjlo.h>
#include <vision_srvs/cop_call.h>
#include <vision_msgs/partial_lo.h>
#include <vision_msgs/cop_answer.h>
#include <vision_srvs/clip_polygon.h>

using namespace cloud_algos;

// we need this for a shared_ptr later on
struct dummy_deleter
{
  void operator()(void const *) const
  {
  }
};

/// holds a single object on a table, with timestamp, orig. pcd data and reconstructed representations
struct TableObject
{
  TableObject (): name(""),  sensor_type(""), object_type(""), object_color(""), 
                  object_geometric_type("cluster"), perception_method(""), lo_id (0) { }
  geometry_msgs::Point32 center;
  sensor_msgs::PointCloud point_cluster;
  geometry_msgs::Point32 minP;
  geometry_msgs::Point32 maxP;
  //unsigned long long lo_id;
  std::vector <double> coeffs;
  double score;
  std::vector<int> triangles;
  std::string name;
  std::string sensor_type;
  std::string object_type;
  std::string object_color;
  std::string object_geometric_type;
  std::string perception_method;
  boost::shared_ptr<const triangle_mesh::TriangleMesh> mesh;
  unsigned long long lo_id;
  // this number is _NOT_ unique per cluster! it is meant in a tracking sense!
  unsigned long long object_id;
};

/// holds a single snapshot of a table
struct TableStateInstance
{
  ros::Time time_instance;
  std::vector<TableObject*> objects;
};

/// this contains a "timeline" of table snapshots for one table
struct Table
 {
  unsigned new_flag;
  geometry_msgs::Point32 center;
  geometry_msgs::Polygon polygon;
  int color;
  std::vector<double> coeff; 
  std::vector<TableStateInstance*> inst;

  TableStateInstance *getCurrentInstance ()
  {

    if (inst.size() == 0)
    {
      ROS_ERROR ("TableStateInstance requested in empty list.");
      return new TableStateInstance ();
    }

    return inst.back ();
  }

  std::vector<TableStateInstance*> getLastInstances (unsigned int n)
  {
    std::vector<TableStateInstance*> ret;
    for (std::vector<TableStateInstance*>::reverse_iterator it = inst.rbegin (); it != inst.rend (); it++)
      {
        if (n != 0)
          {
            ret.push_back(*it);
            n--;
          }
        else
          break;
      }
    return ret;
  }

  TableStateInstance *getInstanceAtTime (ros::Time t)
  {
    TableStateInstance* ret = inst.back ();
    for (std::vector<TableStateInstance*>::reverse_iterator it = inst.rbegin (); it != inst.rend (); it++)
      if ((*it)->time_instance <= t)
        ret = *it;
      else
        break;
    return ret;
  }

};

/// this should be called dyn_obj_store
class TableMemory
{
  protected:
    ros::NodeHandle &nh_;

    // topic names
    std::string input_table_topic_;
    std::string input_cop_topic_;
    std::string output_cloud_topic_;
    std::string output_table_state_topic_;
    std::string cop_beliefstate_topic_;
    std::string output_cluster_name_topic_;

    // publishers and subscribers
    ros::Publisher cop_beliefstate_pub_;
    ros::Publisher cloud_pub_;
    ros::Publisher mem_state_pub_;
    ros::Publisher cluster_name_pub_;
    ros::Subscriber table_sub_;
    ros::Subscriber cop_sub_;
    ros::Publisher table_mesh_pub_;

    // service call related stuff
    ros::ServiceServer table_memory_clusters_service_;
    ros::ServiceClient table_reconstruct_clusters_client_;

    int cluster_name_counter_;
    int counter_;
    float color_probability_;
    std::string global_frame_;
    
    /// if this is true, we consider table positions and outlines to be fixed.
    /// they will not get updated
    bool fix_tables_;

    //insert lo_ids waiting for prolog update
    std::vector<unsigned long long> update_prolog_;
    //this number never resets in the life cycle of program
    unsigned long long object_unique_id_;
    unsigned long long object_id_counter_;
    //clock when the program started
    ros::Time first_stamp_;
    // THE structure... :D
    std::vector<Table> tables;
    /** @todo have a cupboards structure as well */

    // plugin loader
    pluginlib::ClassLoader<cloud_algos::CloudAlgo> *cl;

    typedef struct _NamedAlgorithm {
      ros::Publisher pub;
      std::string name;
      CloudAlgo * algorithm;
      _NamedAlgorithm (std::string n) : name(n) {};
    } NamedAlgorithm;

    std::vector<NamedAlgorithm> algorithm_pool;

    // this is a map<LO Id, vector[tableId, instId, objectId]> that maps a LO id to indices into our tables struct
    std::map <unsigned long long, std::vector<long> > lo_ids;
    TableObject *getObjectFromLOId (unsigned int id)
    {
      std::vector<long> idxs = lo_ids[id];
      return tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]];
    }

    ias_table_msgs::PrologReturn getPrologReturn(unsigned long long id)
    {
      ias_table_msgs::PrologReturn ret;
      std::map <unsigned long long, std::vector<long> >::iterator lo_ids_it;
      lo_ids_it = lo_ids.find(id);
      if (lo_ids_it != lo_ids.end())
      {
        std::vector<long>idxs=lo_ids_it->second;
        ret.table_id = idxs[0];
        ret.table_center =  tables[idxs[0]].center;
        ret.coeff =  tables[idxs[0]].coeff;
        ret.stamp =  tables[idxs[0]].inst[idxs[1]]->time_instance;
        ret.first_stamp =  first_stamp_;
        ret.object_center =  tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]]->center;
        ret.object_type =  tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]]->object_type;
        ret.object_color =  tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]]->object_color;
        //object's unique number
        ret.object_id = id;
        ret.object_geometric_type =  tables[idxs[0]].inst[idxs[1]]->objects[idxs[2]]->object_geometric_type;
      }
      else
      {
        ROS_ERROR("Id %lld not found!!!", id);
      }
      return ret;
     }

  public:
    TableMemory (ros::NodeHandle &anode) 
    : nh_(anode), cluster_name_counter_(0), counter_(0)
    , color_probability_(0.2), object_unique_id_(0), object_id_counter_(700000)
    {
      nh_.param ("fix_tables", fix_tables_, false);
      nh_.param ("input_table_topic", input_table_topic_, std::string("table_with_objects"));       // 15 degrees
      nh_.param ("input_cop_topic", input_cop_topic_, std::string("/tracking/out"));       // 15 degrees
      nh_.param ("output_cloud_topic", output_cloud_topic_, std::string("table_mem_state_point_clusters"));       // 15 degrees
      nh_.param ("output_cluster_name_topic", output_cluster_name_topic_, std::string("cluster_names"));       // 15 degrees
      nh_.param ("output_table_state_topic", output_table_state_topic_, std::string("table_mem_state"));       // 15 degrees
      nh_.param ("cop_beliefstate_topic", cop_beliefstate_topic_, std::string("table_mem_belief"));       // 15 degrees
      nh_.param ("/global_frame_id", global_frame_, std::string("/map"));
      first_stamp_ = ros::Time::now();
      table_sub_ = nh_.subscribe (input_table_topic_, 1, &TableMemory::table_cb, this);
//       cop_sub_ = nh_.subscribe (input_cop_topic_, 1, &TableMemory::cop_cb, this);
      cop_beliefstate_pub_ = nh_.advertise<ias_table_msgs::TableObject> (cop_beliefstate_topic_, 1);
      mem_state_pub_ = nh_.advertise<mapping_msgs::PolygonalMap> (output_table_state_topic_, 1);
      cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud> (output_cloud_topic_, 1);
      cluster_name_pub_ = nh_.advertise<position_string_rviz_plugin::PositionStringList> (output_cluster_name_topic_, 1);
      table_memory_clusters_service_ = nh_.advertiseService ("table_memory_clusters_service", &TableMemory::clusters_service, this);
      table_mesh_pub_ = nh_.advertise <RobustBoxEstimation::OutputType>("table_mesh", 5);
      algorithm_pool.push_back (NamedAlgorithm ("cloud_algos/MovingLeastSquares"));
      algorithm_pool.push_back (NamedAlgorithm ("cloud_algos/CylinderEstimation"));
      algorithm_pool.push_back (NamedAlgorithm ("cloud_algos/DepthImageTriangulation"));
      algorithm_pool.push_back (NamedAlgorithm ("cloud_algos/RobustBoxEstimation"));

      load_plugins ();
    }

    bool compare_table (Table& old_table, const ias_table_msgs::TableWithObjects::ConstPtr& new_table)
    {
      ros::ServiceClient polygon_clipper = nh_.serviceClient<vision_srvs::clip_polygon> ("/intersect_poly", true);
      if (polygon_clipper.exists() && old_table.polygon.points.size() > 2 &&new_table->table.points.size() > 2)
      {
        vision_srvs::clip_polygon::Request req;
        req.operation = req.INTERSECTION;
        req.poly1 = old_table.polygon;
        req.poly2 = new_table->table;
        vision_srvs::clip_polygon::Response resp;

        polygon_clipper.call (req, resp);
        std::vector<double> normal_z(3);
        normal_z[0] = 0.0;
        normal_z[1] = 0.0;
        normal_z[2] = 1.0;
        double area_old = cloud_geometry::areas::compute2DPolygonalArea (old_table.polygon, normal_z);
        double area_new = cloud_geometry::areas::compute2DPolygonalArea (new_table->table, normal_z);
        double area_clip = cloud_geometry::areas::compute2DPolygonalArea (resp.poly_clip, normal_z);
        ROS_INFO ("The 3 areas are: %f, %f, %f: [%f percent, %f percent]",
                  area_old, area_new, area_clip, area_clip/area_old, area_clip/area_new);
        if (area_clip/area_old > 0.5 || area_clip/area_new > 0.5)
          return true;
      }
      else
        ROS_WARN ("fix_tables is set to false, but could not find Polygon Clipper");
      

      // if center of new (invcomplete) table is within old
      // table bounds, it's the same
      geometry_msgs::Point32 center;
      center.x = new_table->table_min.x + (new_table->table_max.x - new_table->table_min.x) / 2.0;
      center.y = new_table->table_min.y + (new_table->table_max.y - new_table->table_min.y) / 2.0;
      center.z = new_table->table_min.z + (new_table->table_max.z - new_table->table_min.z) / 2.0;
      if (cloud_geometry::areas::isPointIn2DPolygon (center, old_table.polygon))
        return true;

      double x = center.x;
      double y = center.y;
      double z = center.z;

      std::cerr << "Table compare returns false. Center = <" << x << "," << y << "," << z << ">." << std::endl;
      for (unsigned int i = 0; i < old_table.polygon.points.size() ; i ++)
        std::cerr << "\t Polygon points " << i << " = <" << old_table.polygon.points[i].x
                  << "," << old_table.polygon.points[i].y
                  << "," << old_table.polygon.points[i].z
                  << ">." << std::endl;
      return false;
    }

    void
      update_table (int table_num, const ias_table_msgs::TableWithObjects::ConstPtr& new_table)
    {
      Table &old_table = tables[table_num];
      ROS_INFO ("Updating table with new TableInstance.");
      TableStateInstance *inst = new TableStateInstance ();
      for (unsigned int i = 0; i < new_table->objects.size(); i++)
      {
        TableObject *to = new TableObject ();

        to->point_cluster = new_table->objects[i].points;
        cloud_geometry::statistics::getMinMax (to->point_cluster, to->minP, to->maxP);

        to->center.x = to->minP.x + (to->maxP.x - to->minP.x) * 0.5;
        to->center.y = to->minP.y + (to->maxP.y - to->minP.y) * 0.5;
        to->center.z = to->minP.z + (to->maxP.z - to->minP.z) * 0.5;

        inst->objects.push_back (to);
      }
      inst->time_instance = new_table->header.stamp;
      old_table.inst.push_back (inst);
      
      // if we fix tables, we don't update the boundaries
      if (fix_tables_)
        return;

      ros::ServiceClient polygon_clipper = nh_.serviceClient<vision_srvs::clip_polygon> ("/intersect_poly", true);
      if (polygon_clipper.exists())
      {

        vision_srvs::clip_polygon::Request req;
        req.operation = req.UNION;
        req.poly1 = old_table.polygon;
        req.poly2 = new_table->table;
        double z_mean = 0.0;
        for (unsigned int poly_i = 0; poly_i < req.poly1.points.size(); poly_i++)
          z_mean += req.poly1.points.at(poly_i).z;
        for (unsigned int poly_i = 0; poly_i < req.poly2.points.size(); poly_i++)
          z_mean += req.poly2.points.at(poly_i).z;
        z_mean /= (req.poly2.points.size () + req.poly1.points.size());

        vision_srvs::clip_polygon::Response resp;

        polygon_clipper.call (req, resp);
        for (unsigned int poly_i = 0; poly_i < resp.poly_clip.points.size(); poly_i++)
          resp.poly_clip.points.at(poly_i).z = z_mean;
        std::vector<double> normal_z(3);
        normal_z[0] = 0.0;
        normal_z[1] = 0.0;
        normal_z[2] = 1.0;
        double area_old = cloud_geometry::areas::compute2DPolygonalArea (old_table.polygon, normal_z);
        double area_new = cloud_geometry::areas::compute2DPolygonalArea (new_table->table, normal_z);
        double area_clip = cloud_geometry::areas::compute2DPolygonalArea (resp.poly_clip, normal_z);
        ROS_INFO ("The 3 areas are: %f, %f, %f: [%f percent, %f percent]",
                  area_old, area_new, area_clip, area_clip/area_old, area_clip/area_new);
        old_table.polygon = resp.poly_clip;
      }

      // Create a PCD with normals out of the polygon and it's projection on the ground
      boost::shared_ptr<sensor_msgs::PointCloud> contour (new sensor_msgs::PointCloud);
      contour->header = new_table->header;
      contour->points = old_table.polygon.points;
      contour->points.insert (contour->points.end (), old_table.polygon.points.begin (), old_table.polygon.points.end ());
      contour->channels.resize (3);
      contour->channels[0].name = "nx";
      contour->channels[0].values.resize (2 * old_table.polygon.points.size());
      contour->channels[1].name = "ny";
      contour->channels[1].values.resize (2 * old_table.polygon.points.size());
      contour->channels[2].name = "nz";
      contour->channels[2].values.resize (2 * old_table.polygon.points.size());

      // Compute normals
      for (unsigned int i = 0; i < old_table.polygon.points.size(); i++)
      {
        // project one of the contours on the ground
        contour->points[i].z = 0.0;
        // compute edge vector
        int j = (i+1) % old_table.polygon.points.size();
        double x = old_table.polygon.points[i].x - old_table.polygon.points[j].x;
        double y = old_table.polygon.points[i].y - old_table.polygon.points[j].y;
        double length = sqrt(x*x + y*y);
        // fail-safe for the case the polygon has duplicate or weird points
        if (length == 0)
        {
          // this will not be considered by box fitting
          contour->channels[0].values[i] = contour->channels[0].values[i+old_table.polygon.points.size()] = 0;
          contour->channels[1].values[i] = contour->channels[1].values[i+old_table.polygon.points.size()] = 0;
          contour->channels[2].values[i] = contour->channels[2].values[i+old_table.polygon.points.size()] = 1;
        }
        else
        {
          // cross product with Z
          contour->channels[0].values[i] = contour->channels[0].values[i+old_table.polygon.points.size()] = -y/length;
          contour->channels[1].values[i] = contour->channels[1].values[i+old_table.polygon.points.size()] =  x/length;
          contour->channels[2].values[i] = contour->channels[2].values[i+old_table.polygon.points.size()] =  0;
        }
      }

      // Fit a rectangle to the double contour
      ROS_INFO("[update_table] Calling RobustBoxEstimation with a double contour of size %ld", contour->points.size ());
      CloudAlgo *alg_box = find_algorithm ("cloud_algos/RobustBoxEstimation");
      alg_box->pre();
      std::string process_answer_box = ((RobustBoxEstimation*)alg_box)->process (contour);
      ROS_INFO("[update_table] got response: %s", process_answer_box.c_str ());
      boost::shared_ptr<const RobustBoxEstimation::OutputType> table_mesh = ((RobustBoxEstimation*)alg_box)->output ();
      old_table.coeff = ((RobustBoxEstimation*)alg_box)->getCoeff ();
      ROS_INFO("[update_table] Publishing result as triangle_mesh::TriangleMesh on topic: table_mesh");
      table_mesh_pub_.publish (table_mesh);
      alg_box->post();
    }

    // service call from PROLOG
    bool
      clusters_service (ias_table_srvs::ias_table_clusters_service::Request &req,
                          ias_table_srvs::ias_table_clusters_service::Response &resp)
    {
      ROS_INFO("Tables to update: %ld", update_prolog_.size());
      for (unsigned int up = 0; up < update_prolog_.size(); up++)
        {
	  ias_table_msgs::PrologReturn pr =  getPrologReturn (update_prolog_[up]);
          resp.prolog_return.push_back(pr);
        }
      //TODO lock
      update_prolog_.clear();
      return true;
    }

    void cop_cb (const boost::shared_ptr<const vision_msgs::cop_answer> &msg)
    {
      ROS_INFO ("got answer from cop! (Errors: %s)\n", msg->error.c_str());
      for(unsigned int i = 0; i < msg->found_poses.size(); i++)
      {
        const vision_msgs::aposteriori_position &pos =  msg->found_poses [i];
        ROS_INFO ("Found Obj nr %d with prob %f at pos %d\n", (int)pos.objectId, pos.probability, (int)pos.position);
        //this here asumes that color classes are returned in FIFO fashion wrt to cop query (see cop_call function)!!!
        if(pos.probability >= color_probability_)
        {
          //TableObject * to = getObjectFromLOId (pos.position);
          for (unsigned int cls = 0; cls < pos.models.size (); cls++)
          {
            if(pos.models[cls].type.compare("ColorClass") == 0)
            {
              ROS_INFO("Object color is %s", pos.models[cls].sem_class.c_str());
              //possible returns: [red (or any other color), object type (Jug)]
              //color is a vector of strings in case object contains multiple color hypotheses
              //to->color.push_back(pos.models[cls].sem_class);
            }
          }
        }
      }
      ROS_INFO ("End!\n");
    }


    bool update_jlo (int table_num)
    {
      ros::ServiceClient jlo_client_ = nh_.serviceClient<vision_srvs::srvjlo> ("/located_object", true);

      // TODO:
      //if (!jlo_client_.exists ()) return false;

      for (unsigned int o_idx = 0; o_idx < tables[table_num].getCurrentInstance ()->objects.size (); o_idx++)
      {
        TableObject *o = tables[table_num].getCurrentInstance ()->objects [o_idx];
        geometry_msgs::Point32 extents;
        extents.x = o->center.x - o->minP.x;
        extents.y = o->center.y - o->minP.y;
        extents.z = o->center.z - o->minP.z;

        // create service client call to jlo
        vision_srvs::srvjlo call;
        call.request.command = "update";
        //world frame
        call.request.query.parent_id = 1;
        call.request.query.id = o->lo_id;

        //fill in pose
        int width = 4;
        for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
          {
            if(r == c)
              call.request.query.pose[r * width + c] = 1;
            else
              call.request.query.pose[r * width + c] = 0;
          }
        }

        call.request.query.pose[3]  = o->center.x;
        call.request.query.pose[7]  = o->center.y;
        call.request.query.pose[11] = o->center.z;

        //fill in covariance matrix: 0.9 * 1/2*max(cluster)-min(cluster) [m]
        width = 6;
        for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
          {
            if(r == c)
              call.request.query.cov [r * width + c] = 0.0;
            else
              call.request.query.cov [r * width + c] = 0;
          }
        }

        call.request.query.cov [0]  = extents.x * 0.9;
        call.request.query.cov [7]  = extents.y * 0.9;
        call.request.query.cov [13] = extents.z * 0.9;

        if (!jlo_client_.call(call))
        {
          ROS_ERROR ("Error in ROSjloComm: Update of pose information not psossible!\n");
          return false;
        }
        else if (call.response.error.length() > 0)
        {
          ROS_ERROR ("Error from jlo: %s!\n", call.response.error.c_str());
          return false;
        }

        ROS_INFO ("New Id: %lld (parent %lld)\n", (long long int)call.response.answer.id, (long long int)call.response.answer.parent_id);
        width = 4;
        for(int r = 0; r < width; r++)
        {
          for(int c = 0; c < width; c++)
          {
             printf("%f", call.response.answer.pose[r * width + c]);
          }
          printf("\n");
        }
        printf("\n");

        o->lo_id = call.response.answer.id;
      }
      return true;
    }

    void update_beliefstate_to_cop (int table_num)
    {
      for (unsigned int o_idx = 0; o_idx < tables[table_num].getCurrentInstance ()->objects.size (); o_idx++)
      {
        TableObject *o = tables[table_num].getCurrentInstance ()->objects [o_idx];
        ias_table_msgs::TableObject to;
        to.center = o->center;
        to.min_bound = o->minP;
        to.max_bound = o->maxP;
        to.object_unique_id = o->object_id;
        to.lo_id = o->lo_id;
        to.perception_method = o->perception_method;
        to.sensor_type = o->sensor_type;
        to.object_type = o->object_type;
        to.object_color = o->object_color;
        to.object_geometric_type = o->object_geometric_type;

        cop_beliefstate_pub_.publish (to);
      }
    }

    bool call_cop (int table_num)
    {
      ros::ServiceClient cop_client_ = nh_.serviceClient<vision_srvs::cop_call> ("/cop/in", true);


      for (unsigned int o_idx = 0; o_idx < tables[table_num].getCurrentInstance ()->objects.size (); o_idx++)
      {
        vision_srvs::cop_call call;
        call.request.outputtopic = input_cop_topic_;
        call.request.action_type = 768;   /** Refine*/
        call.request.number_of_objects = tables[table_num].getCurrentInstance ()->objects.size ();


        //TableObject *o = tables[table_num].getCurrentInstance ()->objects [o_idx];

        // save the indices in our vector of vector of vector so we can find a object
        // in our storage from the positionID (lo_id)
        std::vector<long> idxs (3);
        idxs[0] = table_num;
        idxs[1] = tables[table_num].inst.size()-1;
        idxs[2] = o_idx;
        //lo_ids [o->lo_id] = idxs;

        //update_prolog_.push_back(o->lo_id);

        vision_msgs::apriori_position pos;
        pos.probability = 1.0;
        //pos.positionId = o->lo_id;

        call.request.list_of_poses.push_back (pos);
        if(!cop_client_.call(call))
        {
          ROS_INFO("Error calling cop\n");
          return false;
        }
        else
        {
          ROS_INFO("Called cop \n");
        }
        /** TODO: consider the instead of mapping it over the lo id, which might change on a */
        /** something =   call.response.perception_primitive; */
      }

      return true;
    }

    bool update_table_instance_objects (int table_num)
    {
      for (unsigned int o_idx = 0; o_idx < tables[table_num].getCurrentInstance ()->objects.size (); o_idx++)
        {
          //TableObject *o = tables[table_num].getCurrentInstance ()->objects [o_idx];
          // save the indices in our vector of vector of vector so we can find a object
          // in our storage from the positionID (lo_id)
          std::vector<long> idxs (3);
          //table number
          idxs[0] = table_num;
          //instance number
          idxs[1] = tables[table_num].inst.size()-1;
          //object id
          idxs[2] = o_idx;
          //lo_ids [o->lo_id] = idxs;
          ROS_INFO("Pushing --------- object_unique_id_: %lld, table_num: %ld, inst num: %ld, o_idx: %ld", object_unique_id_, idxs[0], idxs[1], idxs[2]);
          lo_ids [object_unique_id_] = idxs;
          update_prolog_.push_back(object_unique_id_);
          object_unique_id_++;
        }
      return true;
    }


    /*!
     * \brief loads a given plugin
     * \param algo_name : name of algorithm as stated in plugins.xml file
     * \param algorithm : reference to pointer which will hold the loaded algorithm
    */
    bool load_algorithm (std::string algo_name, CloudAlgo*& algorithm, ros::Publisher &pub)
    {
      try
      {
        cl->loadLibraryForClass(algo_name);
        ROS_DEBUG("Loaded library with plugin %s inside", algo_name.c_str());
      }
      catch(pluginlib::PluginlibException &ex)
      {
        ROS_ERROR("Failed to load library with plugin %s inside. Exception: %s", algo_name.c_str(), ex.what());
      }

      if (cl->isClassLoaded(algo_name))
      {
        algorithm = cl->createClassInstance(algo_name);
        algorithm->init (nh_);
        pub = algorithm->createPublisher (nh_);
        ROS_INFO("Success. Could create CloudAlgo Class of type %s", algo_name.c_str ());
        return true;
      }
      else ROS_ERROR("Cannot create CloudAlgo Class of type %s", algo_name.c_str ());
      return false;
    }

    /*!
     * \brief loads plugins
    */
    void
      load_plugins ()
    {
      cl = new pluginlib::ClassLoader <cloud_algos::CloudAlgo> ("cloud_algos", "cloud_algos::CloudAlgo");

      ROS_INFO("ClassLoader instantiated");
      std::vector<std::string> plugins = cl->getDeclaredClasses();

      for (std::vector<std::string>::iterator it = plugins.begin(); it != plugins.end() ; ++it)
      {
        ROS_INFO("%s is in package %s and is of type %s", it->c_str(), cl->getClassPackage(*it).c_str(), cl->getClassType(*it).c_str());
        ROS_INFO("It does \"%s\"", cl->getClassDescription(*it).c_str());
      }

      for (unsigned int i = 0; i < algorithm_pool.size(); i++)
        load_algorithm (algorithm_pool[i].name, algorithm_pool[i].algorithm, algorithm_pool[i].pub);
    }

    /*!
     * \brief tries to match clusters to "track" them over time
    */
    void
      name_table_objects (int table_num)
    {
      Table &t = tables[table_num];
      ROS_INFO ("[name_table_objects] Table has %i objects.", (int)t.getCurrentInstance ()->objects.size());
      TableStateInstance *t_now = t.getCurrentInstance();
      std::vector<TableStateInstance*> t_temp = t.getLastInstances(2);
      if (t_temp.size() < 2)
        return;
      TableStateInstance *t_last = t_temp [1];

      for (int i = 0; i < (signed int) t_now->objects.size (); i++)
      {
        TableObject* to_now = t_now->objects.at (i);
        for (int j = 0; j < (signed int) t_last->objects.size (); j++)
        {
          TableObject* to_last = t_last->objects.at (j);
          geometry_msgs::Point32 diff;
          diff.x = to_now->center.x - to_last->center.x;
          diff.y = to_now->center.y - to_last->center.y;
          diff.z = to_now->center.z - to_last->center.z;
          double d = sqrt (diff.x*diff.x + diff.y*diff.y + diff.z*diff.z);

          if (d < 0.1)
          {
            to_now->name = to_last->name;
            to_now->lo_id = to_last->lo_id;
            to_now->object_id = to_last->object_id;
            break;
          }
        }
        if (to_now->name.length() == 0)
        {
          std::stringstream name;
          name << to_now->object_geometric_type << "_" << cluster_name_counter_++;
          to_now->name = name.str();
          to_now->object_id = object_id_counter_++;
        }
      }
    }

    CloudAlgo* find_algorithm (std::string name)
    {
      for (unsigned int i = 0; i < algorithm_pool.size(); i++)
        if (algorithm_pool[i].name == name)
          return algorithm_pool[i].algorithm;
      ROS_ERROR ("could not find algorithm \"%s\"", name.c_str ());
      return NULL;
    }
    
    ros::Publisher find_publisher (std::string name)
    {
      for (unsigned int i = 0; i < algorithm_pool.size(); i++)
        if (algorithm_pool[i].name == name)
          return algorithm_pool[i].pub;
      ROS_ERROR ("could not find publisher for algorithm \"%s\"", name.c_str ());
      return ros::Publisher();
    }

//    /// Function that dejan implements.. :D
//    IMPLEMENTME
//      cut_roi (image, std::vector<Point32> points)
//    {
//    }
//
//    /*!
//     * \brief employs different image detection algorithms for the current set of point clusters on table specified by table_num
//    */
//    void
//      camera_based_recognition (int table_num)
//    {
//      Table &t = tables[table_num];
//      // roi for table boundaries
//      Image roi_table = cut_roi (image_, t.polygon.points);
//      // CALL ALL table ALGORITHMS
//      for (int i = 1; i < (signed int) t.getCurrentInstance ()->objects.size (); i++)
//      {
//        TableObject* to = t.getCurrentInstance ()->objects.at (i);
//        if (to->point_cluster.points.size () == 0)
//        {
//          ROS_INFO ("[reconstruct_table_objects] Table object has 0 points.");
//          continue;
//        }
//        Image roi_cluster = cut_roi (image_, to.point_cluster.points);
//        // CALL ALL cluster ALGORITHMS 
//        // store object name etc in (TableObject to)
//      }
//      // Nico: use these results.. :D
//    }

    /*!
     * \brief employs different reconstruction algorithms for the current set of point clusters on table specified by table_num
    */
    void
      reconstruct_table_objects (int table_num)
    {
      //return;
      for (unsigned int i = 0; i < algorithm_pool.size(); i++)
        if (!cl->isClassLoaded(algorithm_pool[i].name))
        {
          ROS_WARN("%s Class not loaded, will not reconstruct table objects", algorithm_pool[i].name.c_str());
          return;
        }

      CloudAlgo * alg_triangulation = find_algorithm ("cloud_algos/DepthImageTriangulation");
      CloudAlgo * alg_rot_est = find_algorithm ("cloud_algos/CylinderEstimation");
      CloudAlgo * alg_mls = find_algorithm ("cloud_algos/MovingLeastSquares");
      CloudAlgo * alg_box = find_algorithm ("cloud_algos/RobustBoxEstimation");

      ros::Publisher pub_mls = find_publisher ("cloud_algos/MovingLeastSquares");
      ros::Publisher pub_rot = find_publisher ("cloud_algos/CylinderEstimation");
      ros::Publisher pub_tri = find_publisher ("cloud_algos/DepthImageTriangulation");
      ros::Publisher pub_box = find_publisher ("cloud_algos/RobustBoxEstimation");

      Table &t = tables[table_num];
//      table_reconstruct_clusters_client_ = nh_.serviceClient<ias_table_srvs::ias_reconstruct_object> ("ias_reconstruct_object", true);
      //if (table_reconstruct_clusters_client_.exists ())
      {
        ROS_INFO ("[reconstruct_table_objects] Table has %i objects.", (int)t.getCurrentInstance ()->objects.size());

        for (int i = 0; i < (signed int) t.getCurrentInstance ()->objects.size (); i++)
        {
          TableObject* to = t.getCurrentInstance ()->objects.at (i);
          if (to->point_cluster.points.size () == 0)
          {
            ROS_WARN ("[reconstruct_table_objects] Table object has 0 points.");
            continue;
          }

          boost::shared_ptr<const sensor_msgs::PointCloud> cluster = sensor_msgs::PointCloudConstPtr (&to->point_cluster, dummy_deleter());
          // call MLS
          std::vector<std::string> pre_mls = alg_mls->requires ();
          alg_mls->pre();
          std::cerr << "[reconstruct_table_objects] Calling MLS with a PCD with " <<
                        to->point_cluster.points.size () << " points." << std::endl;
          std::string process_answer_mls = ((MovingLeastSquares*)alg_mls)->process
                      (cluster);
          ROS_INFO("got response: %s", process_answer_mls.c_str ());
          boost::shared_ptr <const sensor_msgs::PointCloud> mls_cloud = (((MovingLeastSquares*)alg_mls)->output ());
          alg_mls->post();
          pub_mls.publish (mls_cloud);

          // call rotational estimation
          alg_rot_est->pre ();
          std::cerr << "[reconstruct_table_objects] Calling RotEst with a PCD with " <<
                        mls_cloud->points.size () << " points." << std::endl;
          std::string process_answer_rot = ((CylinderEstimation*)alg_rot_est)->process
                      (mls_cloud);
          ROS_INFO("got response: %s", process_answer_rot.c_str ());
          boost::shared_ptr<sensor_msgs::PointCloud> rot_inliers = ((CylinderEstimation*)alg_rot_est)->getInliers ();
          boost::shared_ptr<sensor_msgs::PointCloud> rot_outliers = ((CylinderEstimation*)alg_rot_est)->getOutliers ();
          boost::shared_ptr<const triangle_mesh::TriangleMesh> rot_mesh = ((CylinderEstimation*)alg_rot_est)->output ();
          alg_rot_est->post ();

          // call box estimation
          alg_box->pre();
          std::cerr << "[reconstruct_table_objects] Calling RobustBoxEstimation with a cluster with " <<
                        mls_cloud->points.size () << " points." << std::endl;
          std::string process_answer_box = ((RobustBoxEstimation*)alg_box)->process
                      (mls_cloud);
          std::vector <double> box_params;
          //((RobustBoxEstimation*)alg_box)->get_box_parameters (box_params);
          boost::shared_ptr<sensor_msgs::PointCloud> box_inliers = ((RobustBoxEstimation*)alg_box)->getInliers ();
          ROS_INFO("got response: %s", process_answer_box.c_str ());
          boost::shared_ptr<const triangle_mesh::TriangleMesh> box_mesh = ((RobustBoxEstimation*)alg_box)->output ();
          alg_box->post();

          if (box_inliers->points.size() > rot_inliers->points.size())
          {
            to->mesh = box_mesh;
            to->object_geometric_type = "box";
            pub_box.publish (box_mesh);
          }
	  else
          {
            to->mesh = rot_mesh;
            to->object_geometric_type = "cyl";
            pub_rot.publish (rot_mesh);
	  }
          
          // call triangulation on outliers
          alg_triangulation->pre();
//          std::cerr << "[reconstruct_table_objects] Calling Triangulation with the outliers from RotEst with " <<
//                        rot_outliers->points.size () << " points." << std::endl;
//          std::string process_answer_tri = ((DepthImageTriangulation*)alg_triangulation)->process
//                      (rot_outliers);
//          ROS_INFO("got response: %s", process_answer_tri.c_str ());
//          pub_tri.publish (((DepthImageTriangulation*)alg_triangulation)->output ());
          alg_triangulation->post();
//break;
        }
      }
    }

    /// this function should contain all parameters that might change during execution
    void update_parameters ()
    {
      nh_.param ("fix_tables", fix_tables_, false);
    }

    // incoming data...
    void
      table_cb (const ias_table_msgs::TableWithObjects::ConstPtr& table)
    {
      update_parameters ();
      int table_found = -1;
      ROS_INFO ("Looking for table in list of known tables.");
      for (int i = 0; i < (signed int) tables.size (); i++)
      {
        if (compare_table (tables[i], table))
        {
          // found same table earlier.. so we append a new table instance measurement
          ROS_INFO ("Table found.");
          update_table (i, table);
          table_found = i;
          break;
        }
      }
      if (fix_tables_ == false) // check if we are supposed to create new tables
      {
        if (table_found == -1)
        {
          std::vector<double> normal_z(3);
          normal_z[0] = 0.0;
          normal_z[1] = 0.0;
          normal_z[2] = 1.0;
          double area = cloud_geometry::areas::compute2DPolygonalArea (table->table, normal_z);
          if (area < 0.15)
          {
            ROS_INFO ("Table area too small.");
            return;
          }
  
          ROS_INFO ("Not found. Creating new table.");
          Table t;
          t.center.x = table->table_min.x + ((table->table_max.x - table->table_min.x) / 2.0);
          t.center.y = table->table_min.y + ((table->table_max.y - table->table_min.y) / 2.0);
          t.center.z = table->table_min.z + ((table->table_max.z - table->table_min.z) / 2.0);
          for (unsigned int i = 0; i < table->table.points.size(); i++)
            t.polygon.points.push_back (table->table.points.at(i));
  
          if (table->table.points.size() < 1)
            ROS_WARN ("Got degenerate polygon.");
          t.color = ((int)(rand()/(RAND_MAX + 1.0)) << 16) +
                    ((int)(rand()/(RAND_MAX + 1.0)) << 8) +
                    ((int)(rand()/(RAND_MAX + 1.0)));
          tables.push_back (t);
          table_found = tables.size () - 1;
          // also append the new (first) table instance measurement.
          update_table (table_found, table);
        }
      }
      if (table_found != -1)
      {
        reconstruct_table_objects (table_found);
        name_table_objects (table_found);
//        camera_based_recognition (table_found);
        publish_mem_state (table_found);
        update_table_instance_objects(table_found);
        update_jlo (table_found);
        update_beliefstate_to_cop (table_found);
        //call_cop (table_found);
        print_mem_stats (table_found);
      }
    }

    void
      publish_mem_state (int table_num)
    {
      // publish table boundaries
      mapping_msgs::PolygonalMap pmap;
      pmap.header.frame_id = global_frame_;

      pmap.chan.resize(1);
      pmap.chan[0].name = "rgb";
      for (unsigned int i = 0; i < tables.size(); i++)
      {
        geometry_msgs::Polygon p = tables[i].polygon;
        pmap.polygons.push_back (p);
        pmap.chan[0].values.push_back (tables[i].color);
      }
      mem_state_pub_.publish (pmap);

      // publish clusters point cloud
      sensor_msgs::PointCloud pc;
      pc.header.frame_id = global_frame_;
      for (unsigned int i = 0; i < tables.size(); i++)
      {
        for (unsigned int ob_i = 0; ob_i < tables[i].getCurrentInstance ()->objects.size(); ob_i++)
          for (unsigned int pc_i = 0; pc_i < tables[i].getCurrentInstance ()->objects[ob_i]->point_cluster.points.size(); pc_i++)
            pc.points.push_back (tables[i].getCurrentInstance ()->objects[ob_i]->point_cluster.points[pc_i]);
      }
      cloud_pub_.publish (pc);

      // publish cluster names
      position_string_rviz_plugin::PositionStringList names;
      names.header.frame_id = global_frame_;
      for (unsigned int i = 0; i < tables.size(); i++)
      {
        for (unsigned int ob_i = 0; ob_i < tables[i].getCurrentInstance ()->objects.size(); ob_i++)
        {
          std::stringstream cluster_label;
          cluster_label << tables[i].getCurrentInstance ()->objects[ob_i]->name << std::string("\non table ") << i;
          names.texts.push_back (cluster_label.str());
          names.poses.push_back (tables[i].getCurrentInstance ()->objects[ob_i]->center);
        }
      }
      cluster_name_pub_.publish (names);
    }

    void
      print_mem_stats (int table_num)
    {
      std::cerr << "Tables : " << tables.size () << std::endl;
      for (unsigned int t = 0; t < tables.size(); t++)
      {
        double x = tables[t].center.x;
        double y = tables[t].center.y;
        double z = tables[t].center.z;
        std::cerr << "\tTable " << t << " : " << tables[t].inst.size () << " instances. Center = <" << x << "," << y << "," << z << ">." << std::endl;

      }
      std::cerr << "\tCurrent Table : " << table_num << " (" << tables[table_num].getCurrentInstance ()->objects.size () << " objects)" << std::endl;
    }

    bool
      spin ()
    {
      while (ros::ok())
      {
        ros::spinOnce ();
        if (counter_ > 0)
          return true;
      }
      return true;
    }
};

int main (int argc, char* argv[])
{
  ros::init (argc, argv, "table_memory");

  ros::NodeHandle nh("~");
  TableMemory n(nh);
  ros::spin ();

  return (0);
}

