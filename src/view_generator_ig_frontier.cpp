#include "victim_localization/view_generator_ig_frontier.h"

view_generator_ig_frontier::view_generator_ig_frontier():
view_generator_IG()
{
  ros::param::param<double>("~frontier_reached_with_distance", dist_for_frontier_reached , 10);
  ros::param::param<bool>("~use_inflated_obstacles", use_inflated_obs_ , true);
  ros::param::param<double>("frontier_yaw", frontier_yaw_res_,M_PI/4.0);
  ros::param::param("~object_bounds_x_max", obj_bounds_x_max_, 0.5);

  visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/Frontier_visualisation"));
  visualTools->loadMarkerPub();

  visualTools->deleteAllMarkers();
  visualTools->enableBatchPublishing();
}


 std::vector<geometry_msgs::Pose> view_generator_ig_frontier::FindFrontiers()
{
  //set up the costmap

  setupMapData();

  // list of all frontiers in the victim map
  std::vector<Index> allFrontiers;
  std::vector<geometry_msgs::Pose> allFrontierPose;
  // check for all cells in the victim map whether or not they are frontier cells
  for (grid_map::GridMapIterator iterator(victim_map);
       !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    if(isFrontier(index)){
     allFrontiers.push_back(index);
    }
  }

  std::vector<geometry_msgs::Pose> FinalFrontier;
  for(unsigned int i = 0; i < allFrontiers.size(); ++i){
     //if(!isFrontierReached(allFrontiers[i])){
      Position P;
      geometry_msgs::Pose FrontierPose;

      victim_map.getPosition(allFrontiers[i],P);
      FrontierPose.position.x =P[0];
      FrontierPose.position.y=P[1];
      FrontierPose.position.z=uav_fixed_height;



      FinalFrontier = setYawtoViewpoint(FrontierPose);
      allFrontierPose.insert(allFrontierPose.end(),FinalFrontier.begin(),FinalFrontier.end());

      FinalFrontier.clear();
      }
  //}
      return allFrontierPose;
}

bool view_generator_ig_frontier::isFrontierReached(Index point){

  tf::Stamped<tf::Pose> robotPose;
  if(!costmap_ros_->getRobotPose(robotPose)){
    ROS_WARN("[View_Generator_IG_Frontier]: Failed to get RobotPose");
  }
  geometry_msgs::PoseStamped robotPoseMsg;
  tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

  Position p;
  victim_map.getPosition(point,p);

  double dx = robotPoseMsg.pose.position.x - p[0];
  double dy = robotPoseMsg.pose.position.y - p[1];

  if ( (dx*dx) + (dy*dy) < (dist_for_frontier_reached*dist_for_frontier_reached)) {
   // ROS_DEBUG("[View_Generator_IG_Frontier]: frontier is within the squared range of: %f", dist_for_frontier_reached);
    return true;
  }
  return false;
}


bool view_generator_ig_frontier::isFrontier(Index point){
  if(isFreeFrontiers(point)){
    // check for unkonwn in the victim map
    Index adjacentPoints[8];
    getAdjacentPoints(point,adjacentPoints);

    for(int i = 0; i < 8; ++i){
      if(isValid(adjacentPoints[i])){
        if(victim_map.at(map_layer,adjacentPoints[i]) == 0.5){  //0.5 indicate unknown cell
          int no_inf_count = 0;
          Index noInfPoints[8];
          getAdjacentPoints(adjacentPoints[i],noInfPoints);
          for(int j = 0; j < 8; j++){
            if( isValid(noInfPoints[j]) && victim_map.at(map_layer,noInfPoints[i]) == 0.5){
              ++no_inf_count;
              if(no_inf_count > 2){
                return true;
                setupMapData();
              }
            }
          }
        }
      }
    }
  }

  return false;
}

bool view_generator_ig_frontier::isFreeFrontiers(Index point){

  Position p;
  int costmap_index;
  victim_map.getPosition(point,p);
  if (victim_map.at(map_layer,point)>=0.5 ) return false;
  unsigned int mx,my;
  costmap_->worldToMap(p[0],p[1],mx,my);
  costmap_index=costmap_->getIndex(mx,my);
  return checkValidity(costmap_index);


 // std::cout << "map_position" << "(" << p[0] << "," << p[1] << ")" << std::endl;
  //std::cout << "cost_position" << "(" << t1 << "," << t2 << ")" << std::endl;
  //std::cout << "difference" << "(" << p[0]-t1 << "," << p[1]-t2 << ")" << std::endl;


  // take into consideration the marginal difference in the resolution
//  double diff_resol= fabs(victim_map.getResolution()-costmap_->getResolution())/2;

//  double actual_pos_x,actual_pos_y;
//  costmap_->mapToWorld(mx,my,actual_pos_x,actual_pos_y);

//  double margin_diff_x=  floor( (p[0]-actual_pos_x) * 10.00 + 0.5 ) / 10.00;
//  double margin_diff_y=  floor( (p[1]-actual_pos_y) * 10.00 + 0.5 ) / 10.00;


//  return false;
}


inline void view_generator_ig_frontier::getAdjacentPoints(Index point, Index points[]){

  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
  points[4] = upleft(point);
  points[5] = upright(point);
  points[6] = downright(point);
  points[7] = downleft(point);
}

inline Index view_generator_ig_frontier::left(Index point){
  // only go left if no index error and if current point is not already on the left boundary
 return Index(point[0]-1,point[1]);
}
inline Index view_generator_ig_frontier::upleft(Index point){
  return Index(point[0]-1,point[1]+1);
}
inline Index view_generator_ig_frontier::up(Index point){
 return Index(point[0],point[1]+1);
}
inline Index view_generator_ig_frontier::upright(Index point){
 return Index(point[0]+1,point[1]+1);
}
inline Index view_generator_ig_frontier::right(Index point){
  return Index(point[0]+1,point[1]);
}
inline Index view_generator_ig_frontier::downright(Index point){
  return Index(point[0]+1,point[1]-1);
}
inline Index view_generator_ig_frontier::down(Index point){
  return Index(point[0],point[1]-1);
}
inline Index view_generator_ig_frontier::downleft(Index point){
  return Index(point[0]-1,point[1]-1);
}

bool view_generator_ig_frontier::isValid(Index point){
  if ((point[0] >= 0) && (point[0]  < victim_map.getSize()(0)))
      if ((point[1] >= 0) && (point[1]  < victim_map.getSize()(0)))
        return true;

   return false;
}

void view_generator_ig_frontier::setupMapData()
{
  costmap_ = costmap_ros_->getCostmap();

//  if ((this->map_width_ != costmap_->getSizeInCellsX()) || (this->map_height_ != costmap_->getSizeInCellsY())){
//    map_width_ = costmap_->getSizeInCellsX();
//    map_height_ = costmap_->getSizeInCellsY();
//    num_map_cells_ = map_width_ * map_height_;

//    // initialize exploration_trans_array_, obstacle_trans_array_, goalMap and frontier_map_array_
//    exploration_trans_array_.reset(new unsigned int[num_map_cells_]);
//    obstacle_trans_array_.reset(new unsigned int[num_map_cells_]);
//    is_goal_array_.reset(new bool[num_map_cells_]);
//    frontier_map_array_.reset(new int[num_map_cells_]);
//    clearFrontiers();
//    resetMaps();
//  }
  occupancy_grid_array_ = costmap_->getCharMap();
}

//void view_generator_ig_frontier::clearFrontiers()
//{
//  std::fill_n(frontier_map_array_.get(), num_map_cells_, 0);
//}

//void view_generator_ig_frontier::resetMaps(){
//  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
//  std::fill_n(obstacle_trans_array_.get(), num_map_cells_, UINT_MAX);
//  std::fill_n(is_goal_array_.get(), num_map_cells_, false);
//}


std::vector<geometry_msgs::Pose> view_generator_ig_frontier::setYawtoViewpoint(geometry_msgs::Pose vp)
{
  geometry_msgs::Pose temp_Frontier;
  std::vector<geometry_msgs::Pose> Frontier_with_yaws;
 // double currYaw = pose_conversion::getYawFromQuaternion(current_pose_.orientation);

  for (double i_yaw=0; i_yaw<=2*M_PI; i_yaw+=frontier_yaw_res_)
  {
   temp_Frontier=vp;
   temp_Frontier.orientation=pose_conversion::getQuaternionFromYaw(i_yaw);
   Frontier_with_yaws.push_back(temp_Frontier);
  }
   return Frontier_with_yaws;
}

void view_generator_ig_frontier::generateViews()
{
  std::vector<geometry_msgs::Pose> initial_poses;
  generated_poses.clear();
  rejected_poses.clear();

  initial_poses=FindFrontiers();

  for (int i=0; i<initial_poses.size(); i++)
  {
    if ( isValidViewpoint(initial_poses[i],false) )
    {
      generated_poses.push_back(initial_poses[i]);
    }
    else
    {
      rejected_poses.push_back(initial_poses[i]);
    }
  }
  std::cout << "[ViewGenerator] Generated " << generated_poses.size() << " poses (" << rejected_poses.size() << " rejected)" << std::endl;

  //ros::Rate(0.01).sleep();
}

void view_generator_ig_frontier::visualize(std::vector<geometry_msgs::Pose> valid_poses,
                                           std::vector<geometry_msgs::Pose> invalid_poses,
                                           geometry_msgs::Pose selected_pose)
{
  visualTools->deleteAllMarkers(); //reset the markers

  for (int i=0; i< valid_poses.size(); i++)
  {
  visualTools->publishArrow(valid_poses[i],rviz_visual_tools::GREEN,rviz_visual_tools::LARGE,0.4);
  }

  for (int i=0; i< invalid_poses.size(); i++)
  {
   visualTools->publishArrow(invalid_poses[i],rviz_visual_tools::RED,rviz_visual_tools::LARGE,0.4);
  }
  visualTools->publishArrow(selected_pose,rviz_visual_tools::BLUE,rviz_visual_tools::LARGE,0.7);

  visualTools->trigger();
}


void view_generator_ig_frontier::visualize()
{
  visualTools->deleteAllMarkers(); //reset the markers

  for (int i=0; i< generated_poses.size(); i++)
  {
  visualTools->publishArrow(generated_poses[i],rviz_visual_tools::GREEN,rviz_visual_tools::LARGE,0.4);
  }

  for (int i=0; i< rejected_poses.size(); i++)
  {
   visualTools->publishArrow(rejected_poses[i],rviz_visual_tools::RED,rviz_visual_tools::LARGE,0.4);
  }
  visualTools->trigger();
}

bool view_generator_ig_frontier::checkValidity(int costmap_index){

if(IsValid(costmap_index)){
  // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free


  if(use_inflated_obs_){
    if(occupancy_grid_array_[costmap_index] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
      return true;
    }
  } else {
    if(occupancy_grid_array_[costmap_index] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
      return true;
    }
  }
}
return false;
}

std::string view_generator_ig_frontier::getMethodName()
{
  return "frontier";
}


