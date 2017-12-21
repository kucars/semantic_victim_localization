#include "victim_localization/straightline.h"


  straightLine::straightLine(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, volumetric_mapping::OctomapManager *manager):
  nh_(nh),
  nh_private_(nh_private),
  manager_(manager),
  navigationBase()
{
  ros::param::param("~uav_fixed_height", uav_fixed_height, 1.0);
  ros::param::param("~extensionRange", extensionRange_, 1.0);
  ros::param::param("~bounding_box_x", boundingbox_x_, 0.2);
  ros::param::param("~bounding_box_y", boundingbox_y_, 0.2);
  ros::param::param("~bounding_box_z", boundingbox_z_, 0.2);
  ros::param::param("~overshoot", dOvershoot_, 0.25);
}

bool straightLine::GeneratePath(geometry_msgs::Pose end, nav_msgs::Path &Path)
{
 if (getDistance(current_pose_,end)< d_close ) // terminate if the end pose is at the robot current pose
   return false;

 if (manager_ == NULL) {
   ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
   //return true;
 }

 // Check for collision between start and end along the connection plus some overshoot distance.
 boundingbox_[0]=boundingbox_x_;
 boundingbox_[1]=boundingbox_y_;
 boundingbox_[2]=boundingbox_z_;

 Eigen::Vector3d origin(current_pose_.position.x, current_pose_.position.y, current_pose_.position.z);
 Eigen::Vector3d direction(end.position.x - origin[0], end.position.y - origin[1],
     end.position.z - origin[2]);
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
 {
   return true;

   geometry_msgs::PoseStamped ps_;
   ps_.header.frame_id="world";
   ps_.header.stamp=ros::Time::now();
   ps_.pose=end;
   Path.poses.push_back(ps_);
 }
 return false;
}

std::string straightLine::methodName(void)
{
 methodName_="straightLine";
 return methodName_;
}
