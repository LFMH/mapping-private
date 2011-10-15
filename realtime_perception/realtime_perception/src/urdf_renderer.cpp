//#define GL_GLEXT_PROTOTYPES

#include <GL/glew.h>
#include <GL/gl.h>
#include <GL/glu.h>
#include <GL/glx.h>
#undef Success  // <---- Screw Xlib for this

#include <pcl/common/time.h>
#include <ros/node_handle.h>

#include <realtime_perception/urdf_renderer.h>
#include <realtime_perception/shader_wrapper.h>

namespace realtime_perception
{
  URDFRenderer::URDFRenderer (std::string model_description, 
                              std::string tf_prefix,
                              std::string cam_frame,
                              std::string fixed_frame,
                              tf::TransformListener &tf)
    : model_description_(model_description)
    , tf_prefix_(tf_prefix)
    , camera_frame_ (cam_frame)
    , fixed_frame_(fixed_frame)
    , tf_(tf)
  {
    initURDFModel ();
    tf_.setExtrapolationLimit (ros::Duration (1.1));
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Loads URDF model from the parameter server and parses it. call loadURDFModel */
  void
    URDFRenderer::initURDFModel ()
  {
    urdf::Model model;
    if (!model.initString(model_description_))
    {
      ROS_ERROR ("URDF failed Model parse");
      return;
    }

    ROS_INFO ("URDF parsed OK");
    loadURDFModel (model);
    ROS_INFO ("URDF loaded OK");
  }


  /// /////////////////////////////////////////////////////////////////////////////
  /// @brief load URDF model description from string and create search operations data structures
  void URDFRenderer::loadURDFModel
    (urdf::Model &model)
  {
    typedef std::vector<boost::shared_ptr<urdf::Link> > V_Link;
    V_Link links;
    model.getLinks(links);

    V_Link::iterator it = links.begin();
    V_Link::iterator end = links.end();

    for (; it != end; ++it)
      process_link (*it);
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief Processes a single URDF link, creates renderable for it */
  void URDFRenderer::process_link (boost::shared_ptr<urdf::Link> link)
  {
    if (link->visual.get() == NULL || link->visual->geometry.get() == NULL)
      return;

    boost::shared_ptr<Renderable> r;
    if (link->visual->geometry->type == urdf::Geometry::BOX)
    {
      boost::shared_ptr<urdf::Box> box = boost::dynamic_pointer_cast<urdf::Box> (link->visual->geometry);
      r.reset (new RenderableBox (box->dim.x, box->dim.y, box->dim.z));
    }
    else if (link->visual->geometry->type == urdf::Geometry::CYLINDER)
    {
      boost::shared_ptr<urdf::Cylinder> cylinder = boost::dynamic_pointer_cast<urdf::Cylinder> (link->visual->geometry);
      r.reset (new RenderableCylinder (cylinder->radius, cylinder->length));
    }
    else if (link->visual->geometry->type == urdf::Geometry::SPHERE)
    {
      boost::shared_ptr<urdf::Sphere> sphere = boost::dynamic_pointer_cast<urdf::Sphere> (link->visual->geometry);
      r.reset (new RenderableSphere (sphere->radius));
    }
    else if (link->visual->geometry->type == urdf::Geometry::MESH)
    {
      boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh> (link->visual->geometry);
      std::string meshname (mesh->filename);
      r.reset (new RenderableMesh (meshname));
    }
    r->setLinkName (tf_prefix_+ "/" + link->name);
    urdf::Vector3 origin = link->visual->origin.position;
    urdf::Rotation rotation = link->visual->origin.rotation;
    r->link_offset = tf::Transform (
        tf::Quaternion (rotation.x, rotation.y, rotation.z, rotation.w).normalize (),
        tf::Vector3 (origin.x, origin.y, origin.z));
    if (link->visual && 
        (link->visual->material))
      r->color  = link->visual->material->color;
    renderables_.push_back (r); 
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief sets the camera frame link name */
  void URDFRenderer::setCameraFrame (std::string &cam_f) 
  {
    camera_frame_ = cam_f;
  }
  
  ////////////////////////////////////////////////////////////////////////////////
  /** \brief loops over all renderables and updates its transforms from TF */
  void URDFRenderer::transforms_changed ()
  {
    tf::StampedTransform t;

    std::vector<boost::shared_ptr<Renderable> >::const_iterator it = renderables_.begin ();
    for (; it != renderables_.end (); it++)
    {
      try
      {
        tf_.lookupTransform (fixed_frame_, (*it)->name, ros::Time (), t);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s",ex.what());
      }
      (*it)->link_to_fixed = tf::Transform (t.getRotation (), t.getOrigin ());
    }
  }

  ////////////////////////////////////////////////////////////////////////////////
  /** \brief loops over all renderables and renders them to canvas */
  void URDFRenderer::render ()
  {
    printf ("rendering frame");
    transforms_changed ();
      
    //static ShaderWrapper shader = ShaderWrapper::fromFiles ("package://realtime_perception/include/shaders/test1.vert", 
    //                                                        "package://realtime_perception/include/shaders/test1.frag");
    //TODO shader ();

    std::vector<boost::shared_ptr<Renderable> >::const_iterator it = renderables_.begin ();
    for (; it != renderables_.end (); it++)
      (*it)->render ();
  }

}

//
//
//
//
//      for (int i=0; i< links.size(); i++){
//        if (links[i]->visual.get() == NULL) continue;
//        if (links[i]->visual->geometry.get() == NULL) continue;
//        if (links[i]->visual->geometry->type == urdf::Geometry::MESH){
//
//          //todo: this should really be done by resource retriever
//          boost::shared_ptr<urdf::Mesh> mesh = boost::dynamic_pointer_cast<urdf::Mesh> (links[i]->visual->geometry);
//          std::string filename (mesh->filename);
//
//          if (filename.substr(filename.size() - 4 , 4) != ".stl" && filename.substr(filename.size() - 4 , 4) != ".dae") continue;
//          if (filename.substr(filename.size() - 4 , 4) == ".dae")
//            filename.replace(filename.size() - 4 , 4, ".stl");
//          ROS_INFO("adding link %d %s",i,links[i]->name.c_str());
//          filename.erase(0,25);
//          filename = description_path + filename;
//
//          boost::shared_ptr<CMeshO> mesh_ptr(new CMeshO);
//
//          if(vcg::tri::io::ImporterSTL<CMeshO>::Open(*mesh_ptr,filename.c_str())){
//            ROS_ERROR("could not load mesh %s", filename.c_str());
//            continue;
//          }
//
//          links_with_meshes.push_back(links[i]);
//          meshes[links[i]->name] = mesh_ptr;
//
//          tf::Vector3 origin(links[i]->visual->origin.position.x, links[i]->visual->origin.position.y, links[i]->visual->origin.position.z);
//          tf::Quaternion rotation(links[i]->visual->origin.rotation.x, links[i]->visual->origin.rotation.y, links[i]->visual->origin.rotation.z, links[i]->visual->origin.rotation.w);
//
//          offsets_[links[i]->name] = tf::Transform(rotation, origin);
//
//
//
//
//
//        }
//
//      }
//
//
//     initRobot();
//
//      //get camera intinsics
//
//    ROS_INFO("waiting for %s", camera_info_topic_.c_str());
//    cam_info_ = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(camera_info_topic_);
//    cameraframe_ = cam_info_->header.frame_id;
//
//    ROS_INFO("%s: robot model initialization done!", ros::this_node::getName().c_str());
//
//}
//
//////////////////////////////////////////////
//void RobotMeshModel::initRobot(){
//    std::vector<boost::shared_ptr<urdf::Link> > links ;
//
//    for (int i=0; i< links_with_meshes.size(); i++){
//
//     //draw
//    // update bounding box
//    boost::shared_ptr<CMeshO> mesh_ptr = meshes[links_with_meshes[i]->name];
//    vcg::tri::UpdateBounding<CMeshO>::Box(*mesh_ptr);
//
//    // update Normals
//    vcg::tri::UpdateNormals<CMeshO>::PerVertexNormalizedPerFace(*mesh_ptr);
//    vcg::tri::UpdateNormals<CMeshO>::PerFaceNormalized(*mesh_ptr);
//
//    // Initialize the opengl wrapper
//    boost::shared_ptr<vcg::GlTrimesh<CMeshO> > wrapper_ptr (new vcg::GlTrimesh<CMeshO>);
//    wrapper_ptr->m = mesh_ptr.get();
//    wrapper_ptr->Update();
//
//    mesh_wrappers[links_with_meshes[i]->name] = wrapper_ptr;
//
//
//    }
//
//};
//
//
//void RobotMeshModel::updateRobotLinks(const ros::Time time_stamp){
//
//  // get the current configuration of the robot links
//  if (current_time_stamp_ != time_stamp){
//    current_time_stamp_ = time_stamp;
//    tf::StampedTransform tf;
//    for (int i=0; i < links_with_meshes.size(); ++i){
//      if (!tf_.waitForTransform(links_with_meshes[i]->name, modelframe_, current_time_stamp_, ros::Duration(0.5))){
//        ROS_ERROR("could not get transform from %s to %s", links_with_meshes[i]->name.c_str(),modelframe_.c_str() );
//        continue;
//      }
//      tf_.lookupTransform( modelframe_, links_with_meshes[i]->name, current_time_stamp_, tf);
//
//          tf  *= offsets_[links_with_meshes[i]->name];;
//      robotLinks_[links_with_meshes[i]->name] = tf;
//
//    }
//
//  }
//}
//
//
//void RobotMeshModel::paintRobot(){
//
//
//  // draw the configuration of the robot links as  specified in robotLinks_
//    float d=1.0f; ///mesh.bbox.Diag();
//    vcg::glScale(d);
//    glDisable(GL_CULL_FACE);
//    glMatrixMode(GL_MODELVIEW);
//
//    for (int i=0; i < links_with_meshes.size(); ++i){
//      boost::shared_ptr<vcg::GlTrimesh<CMeshO> > wrapper_ptr  = mesh_wrappers[links_with_meshes[i]->name];
//
//      btScalar glTf[16];
//      robotLinks_[links_with_meshes[i]->name].getOpenGLMatrix(glTf);
//
//
//
//      glPushMatrix();
//
//      glMultMatrixd((GLdouble*)glTf);
//
//      wrapper_ptr->Draw<vcg::GLW::DMSmooth,   vcg::GLW::CMNone,vcg::GLW::TMNone> ();
//
//      glPopMatrix();
//    }
//}



//
//
/// camera stuff
//
//
//void RobotMeshModel::setCameraInfo(sensor_msgs::CameraInfoConstPtr cam_info){
//  cam_info_ = cam_info;
//}
//
//
//void RobotMeshModel::setCamera(){
//
//  //calc field of view for OpenGL Camera;
//
//    double FulstrumWidth, FulstrumHeight, left, right, top, bottom;
//
//    double near_clip = 0.1;
//    double farclip = 100;
//
//
//    double cx= cam_info_->P[2];
//    double cy = cam_info_->P[6];
//
//    double fx = cam_info_->P[0];
//    double fy = cam_info_->P[5];
//
//
//    FulstrumWidth = near_clip * cam_info_->width / fx;
//    FulstrumHeight = near_clip * cam_info_->height / fy;
//
//
//    left = FulstrumWidth * (- cx / cam_info_->width);
//    right = FulstrumWidth * ( 1.0 - cx / cam_info_->width);
//    top = FulstrumHeight * ( cy / cam_info_->height);
//    bottom = FulstrumHeight * ( -1.0 + cy / cam_info_->height);
//
//
//    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
//    glMatrixMode(GL_PROJECTION);
//    glLoadIdentity();
//
//    glFrustum(left,right, bottom, top, near_clip, farclip);
//    glMatrixMode(GL_MODELVIEW);
//
//}
//
//
//bool RobotMeshModel::setCameraPose(){
//
//
//  //set camera pose relative to robot links
//  if (!tf_.waitForTransform(modelframe_, cameraframe_, current_time_stamp_, ros::Duration(0.5))){
//      ROS_ERROR("setting cam pose: could not get transform from %s to %s", modelframe_.c_str(),cameraframe_.c_str() );
//      return false;
//   }
//  tf_.lookupTransform(modelframe_, cameraframe_, current_time_stamp_, cameraPose);
//
//
//  //get offset for stereo
//  double tx = -1.0 * (cam_info_->P[3] / cam_info_->P[0]);
//  tf::Vector3 origin = cameraPose.getOrigin() + cameraPose.getBasis() * tf::Vector3(tx, 0.0, 0.0);
//
//  tf::Vector3 look_at_position = origin + cameraPose.getBasis().getColumn(2);
//   glLoadIdentity();
//   tf::Vector3 up = -cameraPose.getBasis().getColumn(1);
//
//   gluLookAt(origin.getX() , origin.getY() , origin.getZ() ,
//       look_at_position.getX() , look_at_position.getY(), look_at_position.getZ(),
//       up.getX(), up.getY(), up.getZ());
//   return true;
//
//}
//
//
//




// REGEX BASED LINK / SEARCH OPERATIONS / TARGET FRAMES SETUP
//    for (sop_it = search_operations_.begin (); sop_it != search_operations_.end (); sop_it++)
//    {
//      boost::smatch res;
//      if (boost::regex_match(link->name, res, sop_it->re))
//      {
//        if (!sop_it->pub_topic_re.empty ())
//        {
//          std::string publish_topic = res.format (sop_it->pub_topic_re);
//          sop_it->pub_topic = publish_topic;
//          ROS_INFO ("Regex expanded publisher topic to: %s", publish_topic.c_str());
//          publishers_ [publish_topic] = pcl_ros::Publisher<pcl::PointXYZ>
//                                          (private_nh, publish_topic, max_queue_size_);
//        }
//        ROS_INFO ("Match: %s (%s)", link->name.c_str(), sop_it->re.str().c_str());
//        double r,p,y;
//
//        // handle special case of fixed links of drawers
//        boost::shared_ptr<urdf::Collision> c;
//        boost::regex re (".*_fixed_link");
//        if (boost::regex_match(link->name, re))
//        {
//          c = link->child_links[0]->collision;
//
//          // handle special case of search_expand* params for drawers
//          std::map <std::string, XmlRpc::XmlRpcValue>::const_iterator op_it = sop_it->params.find ("operation");
//          if (op_it != sop_it->params.end())
//          {
//            if (op_it->second == std::string ("fit_drawer"))
//            {
//              ROS_ERROR ("dealing with special case: %s, %s", link->name.c_str(), link->child_joints[0]->name.c_str());
//              sop_it->params["search_expand_distance"] = link->child_joints[0]->limits->upper
//                                                       - link->child_joints[0]->limits->lower;
//              sop_it->params["search_expand_axis"][0] = link->child_joints[0]->axis.x;
//              sop_it->params["search_expand_axis"][1] = link->child_joints[0]->axis.y;
//              sop_it->params["search_expand_axis"][2] = link->child_joints[0]->axis.z;
//              if (sop_it->params.count ("min_drawer_inliers") == 0)
//                sop_it->params["min_drawer_inliers"] = 50;
//            }
//            else if (op_it->second == std::string ("fit_door"))
//            {
//              ROS_ERROR ("dealing with special case: %s, %s", link->name.c_str(), link->child_joints[0]->name.c_str());
//              if (sop_it->params.count ("min_drawer_inliers") == 0)
//                sop_it->params["min_drawer_inliers"] = 50;
//            }
//          }
//        }
//        else
//          c = link->collision;
//
//        c->origin.rotation.getRPY (r,p,y);
//        if (r != 0.0 || p != 0.0 || y != 0.0)
//        {
//          // we have a rotation here, so we need to add this frame to target_frames_
//          TargetFrames tfr;
//          tfr.frame = tf_prefix_ + "/" + link->name;
//          tfr.op = (*sop_it);
//          tfr.translation = (urdf::Vector3());
//          tfr.link = (link);
//          target_frames_.push_back (tfr);
//        }
//        else
//        {
//          // regex matches this link... so let's walk it up...
//          std::string found_link;
//          urdf::Vector3 null_offset;
//          if (walk_back_links (link, link, *sop_it, stop_link_, null_offset))
//          {
//            // do something ?
//          }
//        }
//
//      }
//      else
//      {
//        ROS_INFO ("No match: %s (%s)", link->name.c_str(), sop_it->re.str().c_str());
//      }
//    }
