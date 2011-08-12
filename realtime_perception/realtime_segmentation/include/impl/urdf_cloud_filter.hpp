//template <typename PointT> inline void
//  realtime_perception::URDFCloudFilter<PointT>::principalComponent
//    (const Eigen::Matrix3f &covariance_matrix, const Eigen::Vector4f &point,
//     Eigen::Vector4f &principal_component)
//{
//  // Avoid getting hung on Eigen's optimizers
//  for (int i = 0; i < 3; ++i)
//    for (int j = 0; j < 3; ++j)
//      if (!std::isfinite (covariance_matrix (i, j)))
//      {
//        //ROS_WARN ("[pcl::solvePlaneParameteres] Covariance matrix has NaN/Inf values!");
//        principal_component.setConstant (std::numeric_limits<float>::quiet_NaN ());
//        return;
//      }
//
//  EIGEN_ALIGN16 Eigen::Vector3f eigen_values;
//  EIGEN_ALIGN16 Eigen::Matrix3f eigen_vectors;
//  pcl::eigen33 (covariance_matrix, eigen_vectors, eigen_values);
//
//  // The normalization is not necessary, since the eigenvectors from libeigen are already normalized
//  principal_component[0] = eigen_vectors (0, 2);
//  principal_component[1] = eigen_vectors (1, 2);
//  principal_component[2] = eigen_vectors (2, 2);
//}
//
//
//template <typename PointT> inline double
//  realtime_perception::URDFCloudFilter<PointT>::computePrincipalComponentAngle
//    (const PointCloud &cloud, Eigen::Vector4f &axis)
//{
//  if (cloud.points.size () == 0)
//    return 0.0;
//
//  // Placeholder for the 3x3 covariance matrix at each surface patch
//  EIGEN_ALIGN16 Eigen::Matrix3f covariance_matrix;
//  // 16-bytes aligned placeholder for the XYZ centroid of a surface patch
//  Eigen::Vector4f xyz_centroid;
//
//  // Estimate the XYZ centroid
//  compute3DCentroid (cloud, xyz_centroid);
//
//  // Compute the 3x3 covariance matrix
//  computeCovarianceMatrix (cloud, xyz_centroid, covariance_matrix);
//
//  // Get the principal axis and compare it with the given one
//  Eigen::Vector4f principal_axis;
//  principalComponent (covariance_matrix, xyz_centroid, principal_axis);
//
//  double angle = fabs (pcl::getAngle3D (principal_axis, axis));
//  return fmin (angle, (M_PI - angle));
//}


template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::perform_delete
    (typename std::vector<TargetFrames>::iterator &it,
     std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds,
     std::vector<int> &indices_to_be_deleted)
{
  // collide with the relevant regions of interest
  std::vector<int> indices;
  std::vector<int> indices_expand;

  // handle special case of fixed links of drawers
  boost::shared_ptr<urdf::Collision> c;
  boost::regex re (".*_fixed_link");
  if (boost::regex_match(it->link->name, re))
    c = it->link->child_links[0]->collision;
  else
    c = it->link->collision;

  switch (c->geometry->type)
  {
    case urdf::Geometry::BOX:
      box_collision_->check (c, it->translation, it->op.params,
          transformed_clouds[it->frame], remaining_indices_, indices, threshold_, indices_expand);
    break;
    default:
      ROS_WARN ("Link %s contains an unsupported collision model type.", it->link->name.c_str());
    break;
  }

  // OK, take care of deleting operations -- remembering delete candidates
  indices_to_be_deleted.insert (indices_to_be_deleted.end (), indices.begin (), indices.end ());
}



template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::perform_fit_door
    (typename std::vector<TargetFrames>::iterator &it,
     std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds)
{
//  // collide with the relevant regions of interest
//  ROS_INFO ("DOOR: colliding with link: %s, target_frame: %s",
//            it->link->name.c_str (), it->frame.c_str ());
//  std::vector<int> indices;
//  std::vector<int> indices_expand;
//
//  // handle special case of fixed links of doors
//  boost::shared_ptr<urdf::Collision> c;
//  boost::regex re (".*fixed_link");
//  if (boost::regex_match(it->link->name, re))
//    c = it->link->child_links[0]->collision;
//  else
//    c = it->link->collision;
//
//  switch (c->geometry->type)
//  {
//    case urdf::Geometry::BOX:
//      box_collision_->check (c, it->translation, it->op.params,
//          transformed_clouds[it->frame], remaining_indices_, indices, threshold_, indices_expand);
//    break;
//    default:
//      ROS_WARN ("Link %s contains an unsupported collision model type.", it->link->name.c_str());
//    break;
//  }
//
//  // get child joint
//  if (it->link->child_joints.size () == 0)
//  {
//    ROS_WARN ("door fixed_joint contains no child joints");
//    return;
//  }
//  boost::shared_ptr<urdf::Joint> joint = it->link->child_joints[0];
//  std::string joint_name = joint->name;
//
//  if (1)
//  {
//    boost::shared_ptr<urdf::Box> box = boost::static_pointer_cast <urdf::Box> (c->geometry);
//    double min_x = it->translation.x + c->origin.position.x - (box->dim.x*0.5);
//    double min_y = it->translation.y + c->origin.position.y - (box->dim.y*0.5);
//    double min_z = it->translation.z + c->origin.position.z - (box->dim.z*0.5);
//
//    double max_x = it->translation.x + c->origin.position.x + (box->dim.x*0.5);
//    double max_y = it->translation.y + c->origin.position.y + (box->dim.y*0.5);
//    double max_z = it->translation.z + c->origin.position.z + (box->dim.z*0.5);
//
//    double axis_x = it->op.params["search_expand_axis"][0];
//    double axis_y = it->op.params["search_expand_axis"][1];
//    double axis_z = it->op.params["search_expand_axis"][2];
//    double search_expand_distance = it->op.params["search_expand_distance"];
//
//    double min_x_expand, min_y_expand, min_z_expand;
//    double max_x_expand, max_y_expand, max_z_expand;
//
//    min_x_expand = (axis_x < 0.0)? min_x + axis_x * search_expand_distance: min_x;
//    min_y_expand = (axis_y < 0.0)? min_y + axis_y * search_expand_distance: min_y;
//    min_z_expand = (axis_z < 0.0)? min_z + axis_z * search_expand_distance: min_z;
//    max_x_expand = (axis_x > 0.0)? max_x + axis_x * search_expand_distance: max_x;
//    max_y_expand = (axis_y > 0.0)? max_y + axis_y * search_expand_distance: max_y;
//    max_z_expand = (axis_z > 0.0)? max_z + axis_z * search_expand_distance: max_z;
//
//    PointCloud door_box_points;
//    door_box_points.header = input_->header;
//    door_box_points.header.frame_id = it->frame;
//    PointT pt;
//
//    pt.x = min_x_expand; pt.y = min_y_expand; pt.z = min_z_expand;
//    door_box_points.points.push_back (pt);
//    pt.x = min_x_expand; pt.y = min_y_expand; pt.z = max_z_expand;
//    door_box_points.points.push_back (pt);
//    pt.x = min_x_expand; pt.y = max_y_expand; pt.z = min_z_expand;
//    door_box_points.points.push_back (pt);
//    pt.x = max_x_expand; pt.y = min_y_expand; pt.z = min_z_expand;
//    door_box_points.points.push_back (pt);
//    pt.x = max_x_expand; pt.y = min_y_expand; pt.z = max_z_expand;
//    door_box_points.points.push_back (pt);
//    pt.x = min_x_expand; pt.y = max_y_expand; pt.z = max_z_expand;
//    door_box_points.points.push_back (pt);
//    pt.x = max_x_expand; pt.y = max_y_expand; pt.z = min_z_expand;
//    door_box_points.points.push_back (pt);
//    pt.x = max_x_expand; pt.y = max_y_expand; pt.z = max_z_expand;
//    door_box_points.points.push_back (pt);
//
//    pt.x = joint->parent_to_joint_origin_transform.position.x;
//    pt.y = joint->parent_to_joint_origin_transform.position.y;
//    pt.z = joint->parent_to_joint_origin_transform.position.z + 0.5;
//    door_box_points.points.push_back (pt);
//    pt.x = joint->parent_to_joint_origin_transform.position.x;
//    pt.y = joint->parent_to_joint_origin_transform.position.y;
//    pt.z = joint->parent_to_joint_origin_transform.position.z + 0.55;
//    door_box_points.points.push_back (pt);
//    pt.x = joint->parent_to_joint_origin_transform.position.x;
//    pt.y = joint->parent_to_joint_origin_transform.position.y;
//    pt.z = joint->parent_to_joint_origin_transform.position.z + 0.6;
//    door_box_points.points.push_back (pt);
//
//    publishers_[it->op.pub_topic].publish (door_box_points);
//  }
//
//
//  if (indices_expand.size () > 0)
//  {
//    PointCloud door_points;
//
//    PointT pt;
//    pt.x = joint->parent_to_joint_origin_transform.position.x;
//    pt.y = joint->parent_to_joint_origin_transform.position.y;
//    pt.z = 0;
//
//    for (std::vector<int>::iterator it_idx = indices_expand.begin();
//         it_idx != indices_expand.end(); it_idx++)
//    {
//      door_points.points.push_back (input_->points[remaining_indices_[*it_idx]]);
//      door_points.points.push_back (pt);
//    }
//
//
//    publishers_[it->op.pub_topic].publish (door_points);
//    ROS_WARN ("published %i points on topic: %s", (int)door_points.points.size(), it->op.pub_topic.c_str ());
//
//    for (unsigned int i = 0; i < door_points.points.size(); i++)
//      door_points.points[i].z = 0;
//
//    Eigen::Vector4f axis;
//    axis[0] = (double) it->op.params["search_expand_axis"][0];
//    axis[1] = (double) it->op.params["search_expand_axis"][1];
//    axis[2] = (double) it->op.params["search_expand_axis"][2];
//    axis[3] = 0.0f;
//    double opening_angle = computePrincipalComponentAngle (door_points, axis);
//
//    if (opening_angle > 0.0872664626)
//    {
//      sensor_msgs::JointState js;
//      js.header.stamp = input_->header.stamp;
//      js.name.push_back (joint_name);
//      js.position.push_back (opening_angle);
//      joint_states_pub_.publish (js);
//      ROS_WARN ("published door joint state: %f", opening_angle);
//
//      return;
//    }
//  }
//  sensor_msgs::JointState js;
//  js.header.stamp = input_->header.stamp;
//  js.name.push_back (joint_name);
//  js.position.push_back (0.0);
//  joint_states_pub_.publish (js);
//  ROS_WARN ("published door joint state: %f", 0.0);
}


template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::perform_fit_drawer
    (typename std::vector<TargetFrames>::iterator &it,
     std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds)
{
//  // collide with the relevant regions of interest
//  ROS_INFO ("colliding with link: %s, target_frame: %s",
//            it->link->name.c_str (), it->frame.c_str ());
//  std::vector<int> indices;
//  std::vector<int> indices_expand;
//
//  // handle special case of fixed links of drawers
//  boost::shared_ptr<urdf::Collision> c;
//  boost::regex re (".*_fixed_link");
//  if (boost::regex_match(it->link->name, re))
//    c = it->link->child_links[0]->collision;
//  else
//    c = it->link->collision;
//
//  switch (c->geometry->type)
//  {
//    case urdf::Geometry::BOX:
//      box_collision_->check (c, it->translation, it->op.params,
//          transformed_clouds[it->frame], remaining_indices_, indices, threshold_, indices_expand);
//    break;
//    default:
//      ROS_WARN ("Link %s contains an unsupported collision model type.", it->link->name.c_str());
//    break;
//  }
//
//  // get child joint
//  if (it->link->child_joints.size () == 0)
//    return;
//  boost::shared_ptr<urdf::Joint> joint = it->link->child_joints[0];
//  std::string joint_name = joint->name;
//
//  if (indices_expand.size () > 0)
//  {
//    PointCloud drawer_door_points;
//    std::vector<int> original_indices;
//    original_indices.resize (indices_expand.size());
//    int count = 0;
//    for (std::vector<int>::iterator it_idx = indices_expand.begin();
//         it_idx != indices_expand.end(); it_idx++)
//      original_indices[count++] = remaining_indices_[*it_idx];
//    copyPointCloud (*input_, original_indices, drawer_door_points);
//
//    // find closed state of drawer..
//    boost::shared_ptr<urdf::Box> box_fixed = boost::dynamic_pointer_cast <urdf::Box> (it->link->child_links[0]->collision->geometry);
//
//    double closed_state = it->translation.x + c->origin.position.x + box_fixed->dim.x * 0.5;
//
//    // find current state..
//    std::vector<int> drawer_indices;
//    drawer_indices.insert (drawer_indices.end (), indices_expand.begin(), indices_expand.end());
//
//    ROS_WARN ("found drawer link, found %i inliers to expanded bounding box, attempting drawer fit.", (int)indices_expand.size());
//
//    // Create a shared plane model pointer directly
//    SampleConsensusModelPerpendicularPlanePtr model = boost::make_shared<pcl::SampleConsensusModelPerpendicularPlane<PointT> > (boost::make_shared<PointCloud> (drawer_door_points));
//
//    Eigen::Vector3f ax;
//    ax[0] = 1.0;
//    ax[1] = 0.0;
//    ax[2] = 0.0;
//    model->setAxis (ax);
//    model->setEpsAngle (10);
//
//    // Create the RANSAC object
//    pcl::RandomSampleConsensus<PointT> sac (model, 0.01);
//    sac.setMaxIterations (100);
//
//    // Algorithm tests
//    bool result = sac.computeModel ();
//    if (result)
//    {
//      std::vector<int> inliers;
//      sac.getInliers (inliers);
//
//      Eigen::VectorXf coeff;
//      sac.getModelCoefficients (coeff);
//
//      Eigen::VectorXf coeff_refined;
//      model->optimizeModelCoefficients (inliers, coeff, coeff_refined);
//
//      PointCloud proj_points;
//      model->projectPoints (inliers, coeff_refined, proj_points);
//
//      Eigen::Vector4f pt;
//      pt[0] = 0;
//      pt[1] = 0;
//      pt[2] = 0;
//      pt[3] = 1;
//
//      double open_state = fabs (coeff_refined.dot (pt));
//
//      Eigen::Vector4f coeff_4;
//      coeff_4[0] = coeff_refined[0];
//      coeff_4[1] = coeff_refined[1];
//      coeff_4[2] = coeff_refined[2];
//      coeff_4[3] = coeff_refined[3];
//      Eigen::Vector4f ax_4;
//      ax_4[0] = 1.0;
//      ax_4[1] = 0.0;
//      ax_4[2] = 0.0;
//      ax_4[3] = 0.0;
//      double angle = fabs (pcl::getAngle3D (coeff_4, ax_4));
//      angle = fmin (angle, (M_PI - angle));
//
//      if ((proj_points.points.size() < (unsigned int) indices.size())
//       || (proj_points.points.size() < (unsigned int) (int) it->op.params["min_drawer_inliers"])
//       || (open_state - closed_state > (double)       it->op.params["search_expand_distance"])
//       || (angle < 0.0872664626)
//       )
//      {
//        sensor_msgs::JointState js;
//        js.header.stamp = input_->header.stamp;
//        js.name.push_back (joint_name);
//        js.position.push_back (0.0);
//        joint_states_pub_.publish (js);
//
//        return;
//      }
//
//      publishers_[it->op.pub_topic].publish (proj_points);
//      sensor_msgs::JointState js;
//      js.header.stamp = input_->header.stamp;
//      js.name.push_back (joint_name);
//      js.position.push_back (open_state - closed_state);
//      joint_states_pub_.publish (js);
//      ROS_INFO ("published %i points on topic: %s", (int)proj_points.points.size(), it->op.pub_topic.c_str ());
//    }
//    else
//    {
//      sensor_msgs::JointState js;
//      js.header.stamp = input_->header.stamp;
//      js.name.push_back (joint_name);
//      js.position.push_back (0.0);
//      joint_states_pub_.publish (js);
//    }
//  }
//  else
//  {
//    sensor_msgs::JointState js;
//    js.header.stamp = input_->header.stamp;
//    js.name.push_back (joint_name);
//    js.position.push_back (0.0);
//    joint_states_pub_.publish (js);
//  }
}

template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::perform_segment_objects
    (typename std::vector<TargetFrames>::iterator &it,
     std::map <std::string, boost::shared_ptr <PointCloud> > &transformed_clouds, std::vector<int> &indices_expand)
{
  // collide with the relevant regions of interest
  ROS_INFO ("colliding with link: %s, target_frame: %s",
            it->link->name.c_str (), it->frame.c_str ());
  std::vector<int> indices;

  // handle special case of fixed links of drawers
  boost::shared_ptr<urdf::Collision> c;
  boost::regex re (".*_fixed_link");
  if (boost::regex_match(it->link->name, re))
    c = it->link->child_links[0]->collision;
  else
    c = it->link->collision;

  switch (c->geometry->type)
  {
    case urdf::Geometry::BOX:
      box_collision_->check (c, it->translation, it->op.params,
          transformed_clouds[it->frame], remaining_indices_, indices, threshold_, indices_expand);
    break;
    default:
      ROS_WARN ("Link %s contains an unsupported collision model type.", it->link->name.c_str());
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Filter the cloud and publish the results. Internal method. */
template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::filter (const PointCloud2::ConstPtr &input, const IndicesConstPtr &indices, PointCloud2 &output)
{
    PointCloud cloud_xyz;
    pcl::fromROSMsg (*input, cloud_xyz);
    output.header.frame_id = input->header.frame_id;
    
    ROS_INFO ("number of target frames: %i", (int)target_frames_.size());
    typename std::vector<TargetFrames>::iterator it;
    std::map <std::string, boost::shared_ptr <PointCloud> > transformed_clouds;
    remaining_indices_.clear ();
    std::vector<int> indices_to_be_deleted;
    std::string last_op ("");
    for (it = target_frames_.begin (); it !=target_frames_.end (); it ++)
    {
      if (it->op.params.count ("operation") == 0)
      {
        ROS_WARN ("Filter operation for frame %s not specified", it->frame.c_str ());
        continue;
      }

      // check if we already have a cached pcd converted to the right frame..
      if (transformed_clouds.count (it->frame) == 0)
      {
        // transform point cloud
        PointCloud pcd;
	ros::Time time = cloud_xyz.header.stamp;
	ROS_INFO("wait_for_tf: %f", wait_for_tf_);
	bool found_transform = tf_listener_.waitForTransform(it->frame, cloud_xyz.header.frame_id, time, ros::Duration(wait_for_tf_));
	if (found_transform)
	  {
	    if (pcl_ros::transformPointCloud (it->frame, cloud_xyz, pcd, tf_listener_))
	      {
		pcd.header.frame_id = it->frame;
		ROS_WARN ("Created new transformed point cloud in frame %s.", it->frame.c_str());
		
		// cache the transformed point cloud
		transformed_clouds[it->frame] = boost::make_shared <PointCloud> (pcd);
		
		typename std::map<std::string, boost::shared_ptr<PointCloud> >::iterator it_map;
		for (it_map = transformed_clouds.begin (); it_map != transformed_clouds.end (); it_map++)
		  ROS_WARN ("Map contains: %s / %s (%f).", it_map->first.c_str (), it_map->second->header.frame_id.c_str(), it_map->second->points[0].x);
	      }
	    else
	      return;
	  }
	else
	  {
	    ROS_ERROR("No transform found");
	    return;
	  }
      }


      // perform the actual operations
      if (std::string(it->op.params["operation"]) == "delete")
      {
        perform_delete (it, transformed_clouds, indices_to_be_deleted);
        last_op = "delete";
      }
      else
      {
        if (last_op == "delete")
        {
          // since the delete operations came first, it means we can now delete them..
          if (indices_to_be_deleted.size () != 0) // .. if we haven't already done so
          {
            ROS_INFO ("deleting points...");
            std::vector <bool> stay (cloud_xyz.points.size (), true);
            remaining_indices_.clear ();
            for (unsigned int i = 0; i < indices_to_be_deleted.size (); i++)
              stay[indices_to_be_deleted[i]] = false;
            for (unsigned int i = 0; i < stay.size (); i++)
              if (stay[i])
                remaining_indices_.push_back (i);
            PointCloud out;
            out.header = input->header;
            for (std::vector<int>::iterator it_idx = remaining_indices_.begin();
                 it_idx != remaining_indices_.end(); it_idx++)
              out.points.push_back (cloud_xyz.points[*it_idx]);
            pcl::toROSMsg (out, output);


            ROS_INFO ("%i points of %i left.", (int) remaining_indices_.size(), (int)cloud_xyz.points.size());
            indices_to_be_deleted.clear ();
          }
          last_op = "deleted";
        }
        //else
        {
          if (std::string(it->op.params["operation"]) == "segment_objects")
          {
            ROS_INFO ("segmenting objects...");
            std::vector<int> indices_expand;
            perform_segment_objects (it, transformed_clouds, indices_expand);
            // publish points in expanded search region
            if (indices_expand.size () > 0)
            {
              ROS_INFO ("successfully segmented objects...");
              PointCloud output;
              std::vector<int> original_indices;
              original_indices.resize (indices_expand.size());
              int count = 0;
              for (std::vector<int>::iterator it_idx = indices_expand.begin();
                   it_idx != indices_expand.end(); it_idx++)
                original_indices[count++] = remaining_indices_[*it_idx];
              copyPointCloud (cloud_xyz, original_indices, output);
              // Publish a Boost shared ptr const data
              publishers_[it->op.pub_topic].publish (output);
              ROS_INFO ("published on topic: %s", it->op.pub_topic.c_str ());
            }
            else
              ROS_INFO ("failed to segment any objects...");
          }
          else if (std::string(it->op.params["operation"]) == "fit_drawer")
            perform_fit_drawer (it, transformed_clouds);
          else if (std::string(it->op.params["operation"]) == "fit_door")
            perform_fit_door (it, transformed_clouds);
        }
      }
    }

    //DEPRECATED pub_output_.publish (cloud_xyzrgb);
    //return true;
}

/// @brief recursively walk up from the given link until we hit one of the
/// @brief target_links (and return which one); cancel if we encounter a frame similar to stop.

template <typename PointT> bool
  realtime_perception::URDFCloudFilter<PointT>::walk_back_links
    (const boost::shared_ptr<urdf::Link> l,
     const boost::shared_ptr<urdf::Link> original_l,
     const SearchOperation sop,
     const std::string stop,
           urdf::Vector3 &offset)
{
  // stop if the link is invalid
  if (!l)
    return false;

  std::string frame = l->name;

  //if (frame == stop) // check if it matches our always-stop link
  //  return false;

  double r=0, p=0, y=0;
  if (l->parent_joint)
  {
    l->parent_joint->parent_to_joint_origin_transform.rotation.getRPY (r, p, y);
  }

  // make sure that the whole subtree is rotation-free
  if (  frame == stop
     || ( l->parent_joint
       && ( l->parent_joint->type == urdf::Joint::UNKNOWN
          || l->parent_joint->type == urdf::Joint::REVOLUTE
          || l->parent_joint->type == urdf::Joint::CONTINUOUS
          || l->parent_joint->type == urdf::Joint::FLOATING
          || l->parent_joint->type == urdf::Joint::PLANAR
          || r != 0.0 || p != 0.0 || y != 0.0
          )
        )
     )
  {
    TargetFrames tfr;
    tfr.frame = tf_prefix_ + "/" + frame;
    tfr.op = (sop);
    tfr.translation = (offset);
    tfr.link = (original_l);
    target_frames_.push_back (tfr);
    return true;
  }

  if (l->parent_joint)
  {
    offset.x += l->parent_joint->parent_to_joint_origin_transform.position.x;
    offset.y += l->parent_joint->parent_to_joint_origin_transform.position.y;
    offset.z += l->parent_joint->parent_to_joint_origin_transform.position.z;
  }

  return walk_back_links (l->getParent (), original_l, sop, stop, offset);
}

/// /////////////////////////////////////////////////////////////////////////////
/// @brief load the model description into a string and parse relevant_roi parameters.
template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::loadParams (ros::NodeHandle private_nh)
{
  // read parameters
  private_nh.getParam ("stop_link", stop_link_);
  private_nh.getParam ("threshold", threshold_);
  private_nh.getParam ("tf_prefix", tf_prefix_);
  private_nh.getParam ("use_indices", use_indices_);
  private_nh.getParam ("model_description", description_param_);
  
  // parse relevant ROIs parameters
  XmlRpc::XmlRpcValue v;
  private_nh.param("relevant_rois", v, v);
  private_nh.param("wait_for_tf", wait_for_tf_, 0.5);

  publishers_.clear ();
  search_operations_.clear ();
  target_frames_.clear ();

  // create joint state publisher
  joint_states_pub_ = private_nh.advertise <sensor_msgs::JointState> (tf_prefix_ + "/joint_states", 5);

  // read regions of interest
  for(int i =0; i < v.size(); i++)
  {
    NODELET_DEBUG ("Reading %i th relevant_roi entry", i);
    try {
      // find the regex param and assign it to our regex object
      XmlRpc::XmlRpcValue temp = v[i];
      std::string sre;

      if (temp.hasMember ("regex"))
        sre = (std::string)temp["regex"];
      else
      {
        ROS_WARN ("Didn't find regex param for this ROI.");
        continue;
      }

      SearchOperation sop;
      try {
        sop.re.assign (sre);
      }
      catch (boost::regex_error& e)
      {
	      ROS_ERROR ("\"%s\" is not a valid regex: %s", sre.c_str(), e.what());
        continue;
      }

      // copy the rest of the parameters
      // special treatment for "regex" and "publish" fields.
      XmlRpc::XmlRpcValue::ValueStruct::iterator it;
      for (it = v[i].begin (); it != v[i].end (); it++)
        if (std::string(it->first) == "publish")
          sop.pub_topic_re = (std::string)it->second;
        else
          if (std::string(it->first) != "regex")
            sop.params[std::string(it->first)] = it->second;

      search_operations_.push_back (sop);
    }
    catch (XmlRpc::XmlRpcException& e)  // Pokemon vs additional include file...
    { /*nothing*/
      ROS_ERROR ("Cought Exception while parsing relevant_rois parameter: %s", e.getMessage ().c_str());
    }
  }

  NODELET_DEBUG ("Successfully read %i search operations", (int) search_operations_.size());

  // read URDF model
  std::string content;

  if (!private_nh.getParam(description_param_, content))
  {
    std::string loc;
    if (private_nh.searchParam(description_param_, loc))
    {
      private_nh.getParam(loc, content);
    }
    else
    {
      ROS_ERROR ("Parameter [%s] does not exist, and was not found by searchParam()",
          description_param_.c_str());
      return;
    }
  }

  if (content.empty())
  {
    ROS_ERROR ("URDF is empty");
    return;
  }

  if ( content == model_description_ )
  {
    return;
  }
  // finally, set the model description so we can later parse it.
  model_description_ = content;
}


/// /////////////////////////////////////////////////////////////////////////////
/// @brief load URDF model description from string and create search operations data structures
template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::loadURDFModel
    (TiXmlElement* root_element, urdf::Model &descr, ros::NodeHandle private_nh, bool visual, bool collision)
{

  typedef std::vector<boost::shared_ptr<urdf::Link> > V_Link;
  V_Link links;
  descr.getLinks(links);

  V_Link::iterator it = links.begin();
  V_Link::iterator end = links.end();

  for (; it != end; ++it)
  {
    const boost::shared_ptr<urdf::Link> link = *it;

    typename std::vector<SearchOperation>::iterator sop_it;
    for (sop_it = search_operations_.begin (); sop_it != search_operations_.end (); sop_it++)
    {
      boost::smatch res;
      if (boost::regex_match(link->name, res, sop_it->re))
      {
        if (!sop_it->pub_topic_re.empty ())
        {
          std::string publish_topic = res.format (sop_it->pub_topic_re);
          sop_it->pub_topic = publish_topic;
          ROS_INFO ("Regex expanded publisher topic to: %s", publish_topic.c_str());
          publishers_ [publish_topic] = pcl_ros::Publisher<pcl::PointXYZ>
                                          (private_nh, publish_topic, max_queue_size_);
        }
        ROS_INFO ("Match: %s (%s)", link->name.c_str(), sop_it->re.str().c_str());
        double r,p,y;

        // handle special case of fixed links of drawers
        boost::shared_ptr<urdf::Collision> c;
        boost::regex re (".*_fixed_link");
        if (boost::regex_match(link->name, re))
        {
          c = link->child_links[0]->collision;

          // handle special case of search_expand* params for drawers
          std::map <std::string, XmlRpc::XmlRpcValue>::const_iterator op_it = sop_it->params.find ("operation");
          if (op_it != sop_it->params.end())
          {
            if (op_it->second == std::string ("fit_drawer"))
            {
              ROS_ERROR ("dealing with special case: %s, %s", link->name.c_str(), link->child_joints[0]->name.c_str());
              sop_it->params["search_expand_distance"] = link->child_joints[0]->limits->upper
                                                       - link->child_joints[0]->limits->lower;
              sop_it->params["search_expand_axis"][0] = link->child_joints[0]->axis.x;
              sop_it->params["search_expand_axis"][1] = link->child_joints[0]->axis.y;
              sop_it->params["search_expand_axis"][2] = link->child_joints[0]->axis.z;
              if (sop_it->params.count ("min_drawer_inliers") == 0)
                sop_it->params["min_drawer_inliers"] = 50;
            }
            else if (op_it->second == std::string ("fit_door"))
            {
              ROS_ERROR ("dealing with special case: %s, %s", link->name.c_str(), link->child_joints[0]->name.c_str());
              if (sop_it->params.count ("min_drawer_inliers") == 0)
                sop_it->params["min_drawer_inliers"] = 50;
            }
          }
        }
        else
          c = link->collision;

        c->origin.rotation.getRPY (r,p,y);
        if (r != 0.0 || p != 0.0 || y != 0.0)
        {
          // we have a rotation here, so we need to add this frame to target_frames_
          TargetFrames tfr;
          tfr.frame = tf_prefix_ + "/" + link->name;
          tfr.op = (*sop_it);
          tfr.translation = (urdf::Vector3());
          tfr.link = (link);
          target_frames_.push_back (tfr);
        }
        else
        {
          // regex matches this link... so let's walk it up...
          std::string found_link;
          urdf::Vector3 null_offset;
          if (walk_back_links (link, link, *sop_it, stop_link_, null_offset))
          {
            // do something ?
          }
        }

      }
      else
      {
        ROS_INFO ("No match: %s (%s)", link->name.c_str(), sop_it->re.str().c_str());
      }
    }
  }

  std::sort (target_frames_.begin (), target_frames_.end ());

  // print some debug info about the search operations
  for (unsigned int i = 0; i < target_frames_.size (); i++)
  {
    TargetFrames tfr = target_frames_[i];
    std::cerr << tfr.frame << std::endl << "   - ";
    std::map<std::string, XmlRpc::XmlRpcValue>::iterator it;
    for (it = tfr.op.params.begin (); it != tfr.op.params.end (); it++)
      std::cerr << tfr.op.re.str() << " - " << it->first << " -> " << it->second << std::endl;
    std::cerr << tfr.link->name << std::endl;
    std::cerr << std::endl;
  }
}


////////////////////////////////////////////////////////////////////////////////
/** \brief Nodelet initialization routine. */
template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::onInit ()
{
  pcl_ros::Filter::onInit ();
  ros::NodeHandle private_nh = getMTPrivateNodeHandle ();
  //DEPRECATED pub_output_ = pcl_ros::Publisher<pcl::PointXYZRGB> (private_nh, "output", max_queue_size_);

  loadParams (private_nh);

  // load collision tests
  box_collision_      = boost::shared_ptr<typename realtime_perception::CloudBoxCollision<PointT> >
                          (new realtime_perception::CloudBoxCollision<PointT> ());

  initURDFModel (private_nh);
}

////////////////////////////////////////////////////////////////////////////////
/** \brief Loads URDF model from the parameter server */
template <typename PointT> void
  realtime_perception::URDFCloudFilter<PointT>::initURDFModel (ros::NodeHandle private_nh)
{
  TiXmlDocument doc;
  doc.Parse(model_description_.c_str());
  if (!doc.RootElement())
  {
    ROS_ERROR ("URDF failed XML parse");
    return;
  }

  urdf::Model descr;
  if (!descr.initXml(doc.RootElement()))
  {
    ROS_ERROR ("URDF failed Model parse");
    return;
  }

  ROS_INFO ("URDF parsed OK");
  loadURDFModel (doc.RootElement(), descr, private_nh);
  ROS_INFO ("URDF loaded OK");
}


