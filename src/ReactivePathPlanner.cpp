#include "victim_localization/ReactivePathPlanner.h"


ReactivePathPlanner::ReactivePathPlanner()
{
  navigationBase();
}

bool ReactivePathPlanner::GeneratePath(geometry_msgs::Pose end, std::vector<geometry_msgs::Pose> &Path)
{
  std::cout << "i am here... correct\n";
 if (getDistance(current_pose_,end)< d_close) // terminate if the end pose is at the robot current pose
   return false;

 std::cout << "no problem... correct\n";

 if (manager_ == NULL) {
   ROS_ERROR_THROTTLE(1, "Planner not set up: No octomap available!");
   //return true;
 }

 std::cout << "rquest service... correct\n";


 planningService.request.header.stamp = ros::Time::now();
 planningService.request.header.seq = 1;
 planningService.request.header.frame_id = "world";
 planningService.request.start = current_pose_;
 planningService.request.end   = end;
 planningService.request.grid_start = current_pose_;

 std::cout << "here1\n";

 if(ros::service::call("sspp_planner", planningService))
 {
   std::cout << "here2\n";

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
   }
   std::cout << "here3\n";

   Path.insert(Path.end(),planningService.response.path.begin(),
                                planningService.response.path.end());
   return true;
 }
 else
 {
   ROS_INFO("No Path Found or planner not ready!");
   return false;
 }
 std::cout << "here4\n";

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

