#include "victim_localization/reactive_path_planner.h"


ReactivePathPlanner::ReactivePathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, volumetric_mapping::OctomapManager *manager):
nh_(nh),
nh_private_(nh_private),
manager_(manager),
navigationBase()
{
}

bool ReactivePathPlanner::GeneratePath(geometry_msgs::Pose end, nav_msgs::Path &Path)
{
 if (getDistance(current_pose_,end)< d_close) // terminate if the end pose is at the robot current pose
   return false;

 std::cout << "calling the service" << std::endl;

 if (manager_ == NULL) {
   ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
   return false;
 }

 ROS_INFO("Executing the Planner Loop...");

 if(reactivePlannerServer->PathGeneration(current_pose_,end,Path))
 {
   return true;
 }
 else
 {
   ROS_INFO("No Path Found or planner not ready!");
  return false;
 }

}

bool ReactivePathPlanner::GeneratePath(geometry_msgs::Pose end,std::vector<geometry_msgs::Pose> &Path)
{
 if (getDistance(current_pose_,end)< d_close) // terminate if the end pose is at the robot current pose
   return false;

 if (manager_ == NULL) {
   ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
   return false;
 }

 ROS_INFO("Executing the Reactive Planner");

 if(reactivePlannerServer->PathGeneration(current_pose_,end,Path))
 {
   return true;
 }
 else
 {
  std::cout << cc.red << "Planner Failed to generate planne... Skipping Navigation \n" << cc.reset;
  return false;
 }
}
std::string ReactivePathPlanner::methodName(void)
{
 methodName_="ReactivePathPlanner";
 return methodName_;
}

void ReactivePathPlanner::start()

{
  //initiate the reactive planner service
  reactivePlannerServer= new ReactivePlannerServer(nh_,nh_private_,manager_);
}

void ReactivePathPlanner::SetDynamicGridSize(double x, double y, double z)
{
  reactivePlannerServer->SetDynamicGridSize(x,y,z);
}

void ReactivePathPlanner::SetOriginPose(double x, double y, double z)
{
  reactivePlannerServer->SetOriginPose(x,y,z);
}
