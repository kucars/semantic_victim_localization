#include "victim_localization/reactive_path_planner.h"


ReactivePathPlanner::ReactivePathPlanner()
{
  navigationBase();
}

bool ReactivePathPlanner::GeneratePath(geometry_msgs::Pose end, nav_msgs::Path &Path)
{
 if (getDistance(current_pose_,end)< d_close) // terminate if the end pose is at the robot current pose
   return false;

 if (manager_ == NULL) {
   ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
   //return true;
 }

 ROS_INFO("Executing the Planner Loop");

 planningService.request.header.stamp = ros::Time::now();
 planningService.request.header.seq = 1;
 planningService.request.header.frame_id = "world";
 planningService.request.start = current_pose_;
 planningService.request.end   = end;
 planningService.request.grid_start = current_pose_;

 if(ros::service::call("sspp_planner", planningService))
 {

   ROS_INFO("Path Found");
   for (int i = 0; i < planningService.response.path.size(); i++)
   {
     /*std::cout<<"Path x:"<<planningService.response.path[i].position.x
             <<" y:"<<planningService.response.path[i].position.y
             <<" z:"<<planningService.response.path[i].position.z
             <<"\n";
     */
     tf::Pose pose;
     tf::poseMsgToTF(planningService.response.path[i], pose);
     double yaw = tf::getYaw(pose.getRotation());
     tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), yaw);
     Path.poses[i].pose=planningService.response.path[i];
   }


   //Path.insert(Path.end(),planningService.response.path.begin(),
                               // planningService.response.path.end());

   return true;
 }
 else
 {
   ROS_INFO("No Path Found or planner not ready!");
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

  std::cout << "plannner ready to run it" << std::endl;
  reactivePlannerServer= new ReactivePlannerServer(nh_,nh_private_,manager_);
  std::cout << "plannner runne4d..." << std::endl;
}



