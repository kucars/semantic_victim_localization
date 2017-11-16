#include "victim_localization/view_generator_ig_frontier.h"

view_generator_ig_frontier::view_generator_ig_frontier()
{

  ros::param::param<double>("~frontier_reached_with_distance", dist_for_frontier_reached , 0.25);
  ros::param::param<bool>("~use_inflated_obstacles", use_inflated_obs_ , true);

}


bool view_generator_ig_frontier::FindFrontiers(std::vector<geometry_msgs::PoseStamped>
                                               &frontiers, std::vector<geometry_msgs::PoseStamped>
                                               &noFrontiers)
{
  // list of all frontiers in the occupancy grid
  std::vector<int> allFrontiers;

  // check for all cells in the occupancy grid whether or not they are frontier cells
  for(unsigned int i = 0; i < num_map_cells_; ++i){
    if(isFrontier(i)){
      allFrontiers.push_back(i);
    }
  }

  for(unsigned int i = 0; i < allFrontiers.size(); ++i){
    if(!isFrontierReached(allFrontiers[i])){
      geometry_msgs::PoseStamped finalFrontier;
      double wx,wy;
      unsigned int mx,my;
      costmap_->indexToCells(allFrontiers[i], mx, my);
      costmap_->mapToWorld(mx,my,wx,wy);
      std::string global_frame = costmap_ros_->getGlobalFrameID();
      finalFrontier.header.frame_id = global_frame;
      finalFrontier.pose.position.x = wx;
      finalFrontier.pose.position.y = wy;
      finalFrontier.pose.position.z = 0.0;

      double yaw = getYawToUnknown(costmap_->getIndex(mx,my));

      //if(frontier_is_valid){

      finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);

      frontiers.push_back(finalFrontier);
    }
    //}
  }

  return (frontiers.size() > 0);
}

bool view_generator_ig_frontier::isFrontierReached(int point){

  tf::Stamped<tf::Pose> robotPose;
  if(!costmap_ros_->getRobotPose(robotPose)){
    ROS_WARN("[View_Generator_IG_Frontier]: Failed to get RobotPose");
  }
  geometry_msgs::PoseStamped robotPoseMsg;
  tf::poseStampedTFToMsg(robotPose, robotPoseMsg);

  unsigned int fx,fy;
  double wfx,wfy;
  costmap_->indexToCells(point,fx,fy);
  costmap_->mapToWorld(fx,fy,wfx,wfy);

  double dx = robotPoseMsg.pose.position.x - wfx;
  double dy = robotPoseMsg.pose.position.y - wfy;

  if ( (dx*dx) + (dy*dy) < (p_dist_for_goal_reached_*p_dist_for_goal_reached_)) {
    ROS_DEBUG("[View_Generator_IG_Frontier]: frontier is within the squared range of: %f", p_dist_for_goal_reached_);
    return true;
  }
  return false;

}

bool view_generator_ig_frontier::isFrontier(int point){
  if(isFreeFrontiers(point)){

    int adjacentPoints[8];
    getAdjacentPoints(point,adjacentPoints);

    for(int i = 0; i < 8; ++i){
      if(isValid(adjacentPoints[i])){
        if(occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){

          int no_inf_count = 0;
          int noInfPoints[8];
          getAdjacentPoints(adjacentPoints[i],noInfPoints);
          for(int j = 0; j < 8; j++){


            if( isValid(noInfPoints[j]) && occupancy_grid_array_[noInfPoints[j]] == costmap_2d::NO_INFORMATION){
              ++no_inf_count;

              if(no_inf_count > 2){
                return true;
              }
            }
          }
        }
      }
    }
  }

  return false;
}

bool view_generator_ig_frontier::isFreeFrontiers(int point){

  if(isValid(point)){
    // if a point is not inscribed_inflated_obstacle, leathal_obstacle or no_information, its free


    if(use_inflated_obs_){
      if(occupancy_grid_array_[point] < costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        return true;
      }
    } else {
      if(occupancy_grid_array_[point] <= costmap_2d::INSCRIBED_INFLATED_OBSTACLE){
        return true;
      }
    }
  }
  return false;
}


inline void view_generator_ig_frontier::getAdjacentPoints(int point, int points[]){

  points[0] = left(point);
  points[1] = up(point);
  points[2] = right(point);
  points[3] = down(point);
  points[4] = upleft(point);
  points[5] = upright(point);
  points[6] = downright(point);
  points[7] = downleft(point);

}

inline int view_generator_ig_frontier::left(int point){
  // only go left if no index error and if current point is not already on the left boundary
  if((point % map_width_ != 0)){
    return point-1;
  }
  return -1;
}
inline int view_generator_ig_frontier::upleft(int point){
  if((point % map_width_ != 0) && (point >= (int)map_width_)){
    return point-1-map_width_;
  }
  return -1;

}
inline int view_generator_ig_frontier::up(int point){
  if(point >= (int)map_width_){
    return point-map_width_;
  }
  return -1;
}
inline int view_generator_ig_frontier::upright(int point){
  if((point >= (int)map_width_) && ((point + 1) % (int)map_width_ != 0)){
    return point-map_width_+1;
  }
  return -1;
}
inline int view_generator_ig_frontier::right(int point){
  if((point + 1) % map_width_ != 0){
    return point+1;
  }
  return -1;

}
inline int view_generator_ig_frontier::downright(int point){
  if(((point + 1) % map_width_ != 0) && ((point/map_width_) < (map_height_-1))){
    return point+map_width_+1;
  }
  return -1;

}
inline int view_generator_ig_frontier::down(int point){
  if((point/map_width_) < (map_height_-1)){
    return point+map_width_;
  }
  return -1;

}
inline int view_generator_ig_frontier::downleft(int point){
  if(((point/map_width_) < (map_height_-1)) && (point % map_width_ != 0)){
    return point+map_width_-1;
  }
  return -1;
}

inline bool view_generator_ig_frontier::isValid(int point){
  return (point>=0);
}

double view_generator_ig_frontier::getYawToUnknown(int point)
{
  int adjacentPoints[8];
  getAdjacentPoints(point,adjacentPoints);

  int max_obs_idx = 0;
  unsigned int max_obs_dist = 0;

  for(int i = 0; i < 8; ++i){
    if(isValid(adjacentPoints[i])){
      if(occupancy_grid_array_[adjacentPoints[i]] == costmap_2d::NO_INFORMATION){
        if(obstacle_trans_array_[adjacentPoints[i]] > max_obs_dist){
          max_obs_idx = i;
          max_obs_dist = obstacle_trans_array_[adjacentPoints[i]];
        }
      }
    }
  }
  int orientationPoint = adjacentPoints[max_obs_idx];
  unsigned int sx,sy,gx,gy;
  costmap_->indexToCells((unsigned int)point,sx,sy);
  costmap_->indexToCells((unsigned int)orientationPoint,gx,gy);
  int x = gx-sx;
  int y = gy-sy;
  double yaw = std::atan2(y,x);

  return yaw;
}

void view_generator_ig_frontier::setupMapData()
{
  costmap_ = costmap_ros_->getCostmap();

  if ((this->map_width_ != costmap_->getSizeInCellsX()) || (this->map_height_ != costmap_->getSizeInCellsY())){
    map_width_ = costmap_->getSizeInCellsX();
    map_height_ = costmap_->getSizeInCellsY();
    num_map_cells_ = map_width_ * map_height_;

    // initialize exploration_trans_array_, obstacle_trans_array_, goalMap and frontier_map_array_
    exploration_trans_array_.reset(new unsigned int[num_map_cells_]);
    obstacle_trans_array_.reset(new unsigned int[num_map_cells_]);
    is_goal_array_.reset(new bool[num_map_cells_]);
    frontier_map_array_.reset(new int[num_map_cells_]);
    clearFrontiers();
    resetMaps();
  }
  occupancy_grid_array_ = costmap_->getCharMap();
}

void view_generator_ig_frontier::clearFrontiers()
{
  std::fill_n(frontier_map_array_.get(), num_map_cells_, 0);
}

void view_generator_ig_frontier::resetMaps(){
  std::fill_n(exploration_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(obstacle_trans_array_.get(), num_map_cells_, UINT_MAX);
  std::fill_n(is_goal_array_.get(), num_map_cells_, false);
}

void view_generator_ig_frontier::setCostMapROS(costmap_2d::Costmap2DROS *CostMapROS_)
{
  costmap_ros_=CostMapROS_;
}

