#include "victim_localization/view_generator_ig.h"

view_generator_IG::view_generator_IG():
  vis_sphere_counter_(0),
  vis_marker_array_prev_size_(0),
  generator_type(Generator::NN_Generator)
{
  // Read parameters

  ros::param::param("~view_generator_nn_pos_res_x", res_x_, 1.0);
  ros::param::param("~view_generator_nn_pos_res_y", res_y_, 1.0);
  ros::param::param("~view_generator_nn_pos_res_z", res_z_, 1.0);
  ros::param::param("~view_generator_nn_pos_res_yaw", res_yaw_, M_PI/4.0); // (pi/3)

  ros::param::param("~object_bounds_x_min", obj_bounds_x_min_,-1.0);
  ros::param::param("~object_bounds_x_max", obj_bounds_x_max_, 2.0);
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
  ros::param::param("~bounding_box_x", boundingbox_x_, 0.5);
  ros::param::param("~bounding_box_y", boundingbox_y_, 0.5);
  ros::param::param("~bounding_box_z", boundingbox_z_, 0.5);
  ros::param::param("~overshoot", dOvershoot_, 0.25);

  if (!ros::param::get(ros::this_node::getName() + "/nav_type", nav_type)) {
    ROS_WARN( "navigation type is unknown for the view generator, default value is set to",
              std::string(ros::this_node::getName() + "/nav_type"));
  }

  start_x_=-res_x_;
  start_y_=-res_y_;
  end_x_=res_x_;
  end_y_=res_y_;

  ros::NodeHandle nh_;
  visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/ViewPoints_visualisation"));
  visualTools->loadMarkerPub();

  visualTools->deleteAllMarkers();
  visualTools->enableBatchPublishing();
}

bool view_generator_IG::isInsideBounds(geometry_msgs::Pose p)
{
  if (p.position.x < nav_bounds_x_min_ || p.position.x > nav_bounds_x_max_ ||
      p.position.y < nav_bounds_y_min_ || p.position.y > nav_bounds_y_max_ ||
      p.position.z < nav_bounds_z_min_ || p.position.z > nav_bounds_z_max_)
  {
    return false;
  }

  return true;
}

bool view_generator_IG::isSafe(geometry_msgs::Pose p)
{
//  double resl= manager_->getResolution();
//  double box_size= obj_bounds_x_max_;
//  double Occupied_threshold= 0.8;

//  // check that the box (of size 1m^3) around the sampled point has not occlusion

//  for (double i=p.position.x - (box_size/2) ; i< p.position.x + (box_size/2) ; i=i+resl){
//    for (double j=p.position.y - (box_size/2) ; j< p.position.y + (box_size/2) ; j=j+resl){
//      for (double k=p.position.z - (box_size/2) ; k< p.position.z + (box_size/2) ; k=k+resl){
//         if (manager_->getCellStatusPoint(Eigen::Vector3d (i,j,k))!=1) continue; // if the cell is not free , return false
//         std::cout << manager_->getCellStatusPoint(Eigen::Vector3d (i,j,k))<< std::endl;
//         return false;
//      }
//    }
//  }
//  return true;

  Eigen::Vector3d loc(p.position.x, p.position.y,p.position.z);
  Eigen::Vector3d box_size(0.5,0.5,0.5);
if (generator_type!=Generator::NN_Adaptive_Generator){
 if( manager_->getCellStatusBoundingBox(loc,box_size)==0) return true;}
else if (manager_->getCellStatusBoundingBox(loc,box_size)==0) return true;
 return false;
}   // Alternative: it is also possible to search the 2D occlusion map for a square of 1m x 1m


bool view_generator_IG::isValidViewpoint(geometry_msgs::Pose p , bool check_safety)
{
  if (!isInsideBounds(p) ){
   std::cout << "rejectedbyvalidity" << std::endl;
    return false;
  }

//if (generator_type!=Generator::Frontier_Generator)
//{
  if (!isSafe(p)){
    std::cout << "rejectedbySafety" << std::endl;
    return false;
  }
//}
//else
//{
//  std::cout << cc.blue << "Avoid safe checking, this is done by the reactive planner..\n" << cc.reset;
//}

if (manager_ == NULL) {
    ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
    return false;
  }

if (nav_type==0) // line collision checking only done for straight line navigation. Reactive planner follows a different approach (search space)
  if (isCollide(p)){
    std::cout << "rejectedbycollision" << std::endl;
  return false;
}

return true;
}

bool view_generator_IG::isCollide(geometry_msgs::Pose p) {

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
  if (cellStatus == volumetric_mapping::OctomapManager::CellStatus::kFree)
    return false;

  return true;
}

void view_generator_IG::visualize(std::vector<geometry_msgs::Pose> valid_poses, std::vector<geometry_msgs::Pose> invalid_poses, geometry_msgs::Pose selected_pose)
{

  {
    visualTools->deleteAllMarkers(); //reset the markers

    for (int i=0; i< valid_poses.size(); i++)
    {
    visualTools->publishArrow(valid_poses[i],rviz_visual_tools::GREEN,rviz_visual_tools::XXLARGE,0.5);
    }

    for (int i=0; i< invalid_poses.size(); i++)
    {
     visualTools->publishArrow(invalid_poses[i],rviz_visual_tools::RED,rviz_visual_tools::XXLARGE,0.5);
    }
    visualTools->publishArrow(selected_pose,rviz_visual_tools::BLUE,rviz_visual_tools::XXXLARGE,0.5);

    visualTools->trigger();
  }

}


void view_generator_IG::visualizeAllpose(std::vector<geometry_msgs::Pose> valid_poses, std::vector<geometry_msgs::Pose> invalid_poses)
{

    visualTools->deleteAllMarkers(); //reset the markers

    for (int i=0; i< valid_poses.size(); i++)
    {
    visualTools->publishArrow(valid_poses[i],rviz_visual_tools::GREEN,rviz_visual_tools::XXLARGE,0.5);
    }

    for (int i=0; i< invalid_poses.size(); i++)
    {
     visualTools->publishArrow(invalid_poses[i],rviz_visual_tools::RED,rviz_visual_tools::XXLARGE,0.5);
    }

    visualTools->trigger();
}

void view_generator_IG::visualizeSelectedpose(geometry_msgs::Pose selected_pose)
{
    visualTools->publishArrow(selected_pose,rviz_visual_tools::BLUE,rviz_visual_tools::XXXLARGE,0.5);
    visualTools->trigger();
}

bool view_generator_IG::ComparePoses(geometry_msgs::Pose Pose1, geometry_msgs::Pose Pose2)
{
  if (float(
        (Pose1.position.x-Pose2.position.x) +
        (Pose1.position.y-Pose2.position.y) +
        (Pose1.position.z-Pose2.position.z) +
        (pose_conversion::getYawFromQuaternion(Pose1.orientation) -pose_conversion::getYawFromQuaternion (Pose2.orientation))
        )==0.0) return true;
  return false;
}

void view_generator_IG::setCurrentPose(geometry_msgs::Pose p)
{
  current_pose_ = p;
}

void view_generator_IG::setOctomapManager(volumetric_mapping::OctomapManager *manager)
{
  manager_ = manager;
}

void view_generator_IG::generateViews(bool generate_at_current_location)
{
  std::vector<geometry_msgs::Pose> initial_poses;
  generated_poses.clear();
  rejected_poses.clear();

  double currX = current_pose_.position.x;
  double currY = current_pose_.position.y;
  double currZ = current_pose_.position.z;
  double currYaw = pose_conversion::getYawFromQuaternion(current_pose_.orientation);


  //Generating 3-D state lattice as z-axis movement is restrained (fixed)
  for (double i_x=start_x_; i_x<=end_x_; i_x=i_x+res_x_)
  {
    for (double i_y=start_y_; i_y<=end_y_; i_y=i_y+res_y_)
    {
      for (double i_yaw=-M_PI; i_yaw<M_PI; i_yaw+=res_yaw_)
      {
        // Do not generate any viewpoints in current location
        if (!generate_at_current_location && i_x==0 && i_y==0)
          continue;

        if (i_x==0 && i_y==0 && i_yaw==0)
          continue;

        geometry_msgs::Pose p;
        p.position.x = currX + i_x*cos(currYaw) + i_y*sin(currYaw);
        p.position.y = currY - i_x*sin(currYaw) + i_y*cos(currYaw);
        p.position.z = uav_fixed_height ;//+ res_z_*i_z; // z-axis movement is fixed

        p.orientation = pose_conversion::getQuaternionFromYaw(currYaw + i_yaw);
        initial_poses.push_back(p);
      }
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

  nav_type = 0; // set navigation type as straight line for NN_view_generator

}

void view_generator_IG::generateViews()
{
  generateViews(true);
}


void view_generator_IG::setHistory(nbv_history* h)
{
  nbv_history_ = h;
}

void view_generator_IG::setOcclusionMap(Volumetric_Map* Occ)
{
  occlusion_map = Occ;
}


void view_generator_IG::setvictimmap(grid_map::GridMap map,std::string layer_name)
{ victim_map=map;
  map_layer=layer_name;
}

void view_generator_IG::setCostMapROS(costmap_2d::Costmap2DROS *CostMapROS_)
{
  costmap_ros_=CostMapROS_;
}

std::string view_generator_IG::getMethodName()
{
  return "IG_Base";
}
