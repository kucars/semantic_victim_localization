#include "victim_localization/view_generator_ig.h"

view_generator_IG::view_generator_IG():
  vis_sphere_counter_(0),
  vis_marker_array_prev_size_(0)
{
  // Read parameters

   ros::param::param("~view_generator_pos_res_x", res_x_, 1.0);
   ros::param::param("~view_generator_pos_res_y", res_y_, 1.0);
   ros::param::param("~view_generator_pos_res_z", res_z_, 1.0);
   ros::param::param("~view_generator_pos_res_yaw", res_yaw_, M_PI/3.0); // (pi/3)

   ros::param::param("~object_bounds_x_min", obj_bounds_x_min_,-1.0);
   ros::param::param("~object_bounds_x_max", obj_bounds_x_max_, 1.0);
   ros::param::param("~object_bounds_y_min", obj_bounds_y_min_,-1.0);
   ros::param::param("~object_bounds_y_max", obj_bounds_y_max_, 1.0);
   ros::param::param("~object_bounds_z_min", obj_bounds_z_min_, 0.0);
   ros::param::param("~object_bounds_z_max", obj_bounds_z_max_, 1.0);

   ros::param::param("~nav_bounds_x_min", nav_bounds_x_min_,-10.0);
   ros::param::param("~nav_bounds_x_max", nav_bounds_x_max_, 10.0);
   ros::param::param("~nav_bounds_y_min", nav_bounds_y_min_,-10.0);
   ros::param::param("~nav_bounds_y_max", nav_bounds_y_max_, 10.0);
   ros::param::param("~nav_bounds_z_min", nav_bounds_z_min_, 0.5);
   ros::param::param("~nav_bounds_z_max", nav_bounds_z_max_, 5.0);

   ros::param::param("~uav_fixed_height", uav_fixed_height, 1.0);
   ros::param::param("~extensionRange", extensionRange_, 1.0);
   ros::param::param("~bounding_box_x", boundingbox_x_, 0.2);
   ros::param::param("~bounding_box_y", boundingbox_y_, 0.2);
   ros::param::param("~bounding_box_z", boundingbox_z_, 0.2);
   ros::param::param("~overshoot", dOvershoot_, 0.25);

   ros::NodeHandle nh_;

     pub_view_marker_array_ = nh_.advertise<visualization_msgs::MarkerArray>("generated_pose_marker_array", 10);
     pub_view_drone_marker_ = nh_.advertise<visualization_msgs::Marker>("drone_marker", 10);

}

bool view_generator_IG::isInsideBounds(Pose p)
{
  if (p.position.x < nav_bounds_x_min_ || p.position.x > nav_bounds_x_max_ ||
      p.position.y < nav_bounds_y_min_ || p.position.y > nav_bounds_y_max_ ||
      p.position.z < nav_bounds_z_min_ || p.position.z > nav_bounds_z_max_)
  {
    return false;
  }

  return true;
}

bool view_generator_IG::isSafe(Pose p)
{
  double resl= manager_->octree_->getResolution();
  double box_size= obj_bounds_x_max_;
  double Occupied_threshold= 0.8;

  // check that the point in sampled at vacant area of the 2D occlusion map
   octomap::OcTreeKey key_= occlusion_map->m_octree->coordToKey(p.position.x,p.position.y,p.position.z);

 // if (occlusion_map->Get2DMapValue(key_) != 0)
 // { std::cout << "rejected by occluson_map" << std::endl;
  //  return false;
 // }


  // check that the box (of size 1m^3) around the sampled point has not occlusion

  for (double i=p.position.x - (box_size/2) ; i< p.position.x + (box_size/2) ; i=i+resl){
    for (double j=p.position.y - (box_size/2) ; j< p.position.y + (box_size/2) ; j=j+resl){
      for (double k=p.position.z - (box_size/2) ; k< p.position.z + (box_size/2) ; k=k+resl){
        octomap::OcTreeNode* node = manager_->octree_->search(i,j,k);
        if (node==NULL) continue; // if the point does not exist in Octomap, skip it...
        if ((node->getOccupancy() >= 0.8)) return false;
      }
    }
   }

  return true;
}


bool view_generator_IG::isValidViewpoint(Pose p)
{
  if (!isInsideBounds(p) ){
    //std::cout << "rejectedbyvalidity" << std::endl;
    return false;
}

  if (!isSafe(p)){
    //std::cout << "rejectedbySafety" << std::endl;
    return false;
}

  if (manager_ == NULL) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
    //return true;
  }


  // Check for collision of new connection plus some overshoot distance.
  boundingbox_[0]=boundingbox_x_;
  boundingbox_[1]=boundingbox_y_;
  boundingbox_[2]=boundingbox_z_;

  Eigen::Vector3d origin(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
  Eigen::Vector3d direction(p.position.x - origin[0], p.position.y - origin[1],
      p.position.z - origin[2]);
  if (direction.norm() > extensionRange_)
  {
    direction = extensionRange_ * direction.normalized();
  }

  volumetric_mapping::OctomapManager::CellStatus cellStatus;
  //std::cout << "Pose: "<< p << " NewPose: " << direction + origin + direction.normalized() * dOvershoot_ << std::endl;
  cellStatus = manager_->getLineStatusBoundingBox(
        origin,
        direction + origin + direction.normalized() * dOvershoot_,
        boundingbox_);
  //std::cout << "status is: " << cellStatus << std::endl;
  if (cellStatus == volumetric_mapping::OctomapManager::CellStatus::kFree)// || cellStatus == volumetric_mapping::OctomapManager::CellStatus::kUnknown)
    return true;

  return false;
}


void view_generator_IG::visualize(std::vector<Pose> valid_poses, std::vector<Pose> invalid_poses, Pose selected_pose)
{
  if (pub_view_marker_array_.getNumSubscribers() == 0)
    return;

  visualization_msgs::MarkerArray pose_array;

  // Valid markers
  for (int i=0; i<valid_poses.size(); i++)
  {
    visualization_msgs::Marker pose_marker = visualizeCreateArrowMarker(i, valid_poses[i], true);
    pose_array.markers.push_back(pose_marker);
  }

  // Invalid markers
  int offset = valid_poses.size();
  for (int i=0; i<invalid_poses.size(); i++)
  {
    visualization_msgs::Marker pose_marker = visualizeCreateArrowMarker(i+offset, invalid_poses[i], false);
    pose_array.markers.push_back(pose_marker);
  }

  // Delete old markers
  for (int i=valid_poses.size() + invalid_poses.size(); i<vis_marker_array_prev_size_; i++)
  {
    visualization_msgs::Marker pose_marker = visualizeDeleteArrowMarker(i);
    pose_array.markers.push_back(pose_marker);
  }

  vis_marker_array_prev_size_ = valid_poses.size() + invalid_poses.size();

  //Change the color for the selected Marker in the Valid Marker

  visualizeSelectedArrowMarker(selected_pose,pose_array);

  // Publish markers
  pub_view_marker_array_.publish(pose_array);

}

visualization_msgs::Marker view_generator_IG::visualizeDeleteArrowMarker(int id)
{
  visualization_msgs::Marker pose_marker;

  pose_marker.header.frame_id = "world";
  pose_marker.header.stamp = ros::Time::now();
  pose_marker.id = id;

  //pose_marker.action = visualization_msgs::Marker::DELETE;
  pose_marker.type = visualization_msgs::Marker::ARROW;
  pose_marker.pose = geometry_msgs::Pose();
  return pose_marker;
}

visualization_msgs::Marker view_generator_IG::visualizeCreateArrowMarker(int id, Pose pose, bool valid, double max_z, double min_z)
{
  visualization_msgs::Marker pose_marker;

  pose_marker.header.frame_id = "world";
  pose_marker.header.stamp = ros::Time::now();
  pose_marker.id = id;
  pose_marker.type = visualization_msgs::Marker::ARROW;
  pose_marker.pose = pose;

  // Style
  pose_marker.scale.x = 0.5;
  pose_marker.scale.y = 0.1;
  pose_marker.scale.z = 0.1;

  if( valid )
  {
    pose_marker.color.r = 0;
    pose_marker.color.g = 1;
    pose_marker.color.b = 0;
    pose_marker.color.a = 1.0;
  }
  else
  {
    pose_marker.color.r = 1.0;
    pose_marker.color.g = 0;
    pose_marker.color.b = 0;
    pose_marker.color.a = 1.0;
  }

  return pose_marker;
}


void view_generator_IG::visualizeSelectedArrowMarker(Pose selected_pose, visualization_msgs::MarkerArray &All_poses)
{

  for (int i=0; i<All_poses.markers.size();i++)
    if (ComparePoses(All_poses.markers[i].pose,selected_pose))
    {
        All_poses.markers[i].color.r=0;
        All_poses.markers[i].color.g=0;
        All_poses.markers[i].color.b=1.0;
        All_poses.markers[i].color.a=1.0;
    }
}


bool view_generator_IG::ComparePoses(Pose Pose1, Pose Pose2)
{
  if (float(
        (Pose1.position.x-Pose2.position.x) +
        (Pose1.position.y-Pose2.position.y) +
        (Pose1.position.z-Pose2.position.z) +
        (pose_conversion::getYawFromQuaternion(Pose1.orientation) -pose_conversion::getYawFromQuaternion (Pose2.orientation))
         )==0.0) return true;
      return false;
}



void view_generator_IG::visualizeDrawSphere(Pose p, double r)
{
  if (pub_view_drone_marker_.getNumSubscribers() == 0)
    return;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "world";
  marker.header.stamp = ros::Time();
  marker.id = vis_sphere_counter_;
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = p;
  marker.scale.x = r;
  marker.scale.y = r;
  marker.scale.z = r;
  marker.color.a = 0.3;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  pub_view_drone_marker_.publish( marker );

  vis_sphere_counter_++;
}

void view_generator_IG::setCurrentPose(Pose p)
{
  current_pose_ = p;
}

void view_generator_IG::setOctomapManager(volumetric_mapping::OctomapManager *manager)
{
  manager_ = manager;
}

void view_generator_IG::generateViews(bool generate_at_current_location)
{
  std::vector<Pose> initial_poses;
  generated_poses.clear();
  rejected_poses.clear();

  double currX = current_pose_.position.x;
  double currY = current_pose_.position.y;
  double currZ = current_pose_.position.z;  // the
  double currYaw = pose_conversion::getYawFromQuaternion(current_pose_.orientation);


  //Generating 3-D state lattice as z-axis movement is restrained (fixed)
   int cnt=0;
   for (int i_x=-1; i_x<=1; i_x++)
    {
      for (int i_y=-1; i_y<=1; i_y++)
      {
       // for (int i_z=-5; i_z<=5; i_z++)
        //{
          for (int i_yaw=-1; i_yaw<=1; i_yaw++)
          {
            // Do not generate any viewpoints in current location

            if (!generate_at_current_location && i_x==0 && i_y==0)
            continue;

            if (i_x==0 && i_y==0 && i_yaw==0)
            continue;

            Pose p;
            p.position.x = currX + res_x_*i_x*cos(currYaw) + res_y_*i_y*sin(currYaw);
            p.position.y = currY - res_x_*i_x*sin(currYaw) + res_y_*i_y*cos(currYaw);
            p.position.z = 1.0 ;//+ res_z_*i_z; // z-axis movement is fixed

            p.orientation = pose_conversion::getQuaternionFromYaw(currYaw + res_yaw_*i_yaw);
            initial_poses.push_back(p);
           // std::cout << "Pose Number " << cnt << std::endl;
           //std::cout << p << "\n";
           // cnt++;
          }
       // }
      }
   }


    for (int i=0; i<initial_poses.size(); i++)
    {
      if ( isValidViewpoint(initial_poses[i]) )
      {
        generated_poses.push_back(initial_poses[i]);
      }
      else
      {
        rejected_poses.push_back(initial_poses[i]);
      }
    }

    std::cout << "[ViewGenerator] Generated " << generated_poses.size() << " poses (" << rejected_poses.size() << " rejected)" << std::endl;

}

void view_generator_IG::generateViews()
{
  generateViews(false);
}


void view_generator_IG::setHistory(nbv_history* h)
{
  nbv_history_ = h;
}

void view_generator_IG::setOcclusionMap(Volumetric_Map* Occ)
{
  occlusion_map = Occ;
}



