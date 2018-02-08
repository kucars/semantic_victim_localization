#include "victim_localization/navigation_base.h"

navigationBase::navigationBase()
{
  ros::param::param("~close_distance", d_close, 0.1);
  ros::param::param("~nav_bounds_x_min", nav_bounds_x_min_,-10.0);
  ros::param::param("~nav_bounds_x_max", nav_bounds_x_max_, 10.0);
  ros::param::param("~nav_bounds_y_min", nav_bounds_y_min_,-10.0);
  ros::param::param("~nav_bounds_y_max", nav_bounds_y_max_, 10.0);
  ros::param::param("~nav_bounds_z_min", nav_bounds_z_min_, 0.5);
  ros::param::param("~nav_bounds_z_max", nav_bounds_z_max_, 5.0);
  ros::param::param("~nav_bounds_z_max", nav_bounds_z_max_, 5.0);
  ros::param::param("~uav_fixed_height", uav_fixed_height, 1.0);

}

void navigationBase::setCurrentPose(geometry_msgs::Pose p)
{
  current_pose_ = p;
}


double navigationBase::getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return sqrt(
    (p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) +
    (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) +
    (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z) );
}

bool navigationBase::GeneratePath(geometry_msgs::Pose end, nav_msgs::Path &Path)
{
  std::cout << "[Warning:] " << cc.red << " Navigation method is the Base & no implementation is provided\n" << cc.reset;
  return false;
}

bool navigationBase::GeneratePath(geometry_msgs::Pose end, std::vector<geometry_msgs::Pose> &Path)
{
  std::cout << "[Warning:] " << cc.red << " Navigation method is the Base & no implementation is provided\n" << cc.reset;
  return false;
}

std::string navigationBase::methodName(void)
{
  methodName_="Navigation_Base";
  return methodName_;
}

std::vector<geometry_msgs::Pose>
    navigationBase::Path_discretization(geometry_msgs::Pose start,
                                         geometry_msgs::Pose end,
                                         double step_size)
   {
     double step_size_x,step_size_y,Diff_x,Diff_y;
     double iteration;
     double x_sign,y_sign;

     std::vector<geometry_msgs::Pose> Formed_Path;
     //double path_angle = atan2(end.position.y - start.position.y,end.position.x - start.position.x);
     double path_angle = pose_conversion::getYawFromQuaternion(end.orientation);

     Diff_x= end.position.x - start.position.x;
     Diff_y= end.position.y - start.position.y;

     if (fabs(Diff_x) <= step_size && fabs(Diff_y) <= step_size)
     {
       ROS_INFO ("the two poses are close so no Path discretization is needed");
       Formed_Path.push_back(start);
       Formed_Path.push_back(end);
       return Formed_Path;
     }

    else if (fabs(Diff_x) >= fabs(Diff_y))
    {
      step_size_x=step_size;
      iteration=fabs(Diff_x)/step_size_x;

      step_size_y= fabs(Diff_y)/iteration;
    }

    else if (fabs(Diff_x) < fabs(Diff_y))
    {
      step_size_y=step_size;
      iteration=fabs(Diff_y)/step_size_y;

      step_size_x=fabs(Diff_x)/iteration;
    }

   //********** Start Path Discretization *************//

     (Diff_x>=0) ? x_sign=1 : x_sign=-1;
     (Diff_y>=0) ? y_sign=1 : y_sign=-1;

     geometry_msgs::Pose temp_pose=start;

     temp_pose.orientation= pose_conversion::getQuaternionFromYaw(path_angle);

     for (double i=0; i< iteration-1; i++)
     {
      temp_pose.position.x+= x_sign * step_size_x;
      temp_pose.position.y+= y_sign * step_size_y;
      Formed_Path.push_back(temp_pose);
     }

     // pass the end pose orientation to the path
     temp_pose.orientation= end.orientation;
     Formed_Path.push_back(temp_pose);

   return Formed_Path;

   }

std::vector<geometry_msgs::Pose>
    navigationBase::Path_discretizationtoPath(geometry_msgs::Pose start,
                                         geometry_msgs::Pose end,
                                         double step_size)
{
  double step_size_x,step_size_y,step_size_yaw,step_size_yaw_start,step_size_yaw_end,Diff_x,Diff_y,Diff_yaw,Diff_yaw_start_to_path,Diff_yaw_path_to_end;
  double iteration , iteration_start, iteration_end;
  double x_sign,y_sign,yaw_sign,yaw_start_sign,yaw_end_sign;

  std::vector<geometry_msgs::Pose> Formed_Path;
  double path_angle = atan2(end.position.y - start.position.y,end.position.x - start.position.x);

  Diff_x= end.position.x - start.position.x;
  Diff_y= end.position.y - start.position.y;
  Diff_yaw= pose_conversion::getYawFromQuaternion(end.orientation) -
            pose_conversion::getYawFromQuaternion (start.orientation);

  Diff_yaw_start_to_path= path_angle - pose_conversion::getYawFromQuaternion(start.orientation) -
            pose_conversion::getYawFromQuaternion (start.orientation);

  Diff_yaw_path_to_end= pose_conversion::getYawFromQuaternion(end.orientation) - path_angle;


  if (fabs(Diff_x) <= 0.2 && fabs(Diff_y) <= 0.2)
  {
    ROS_INFO ("the two poses are close so no Path discretization is needed");
    Formed_Path.push_back(start);
    Formed_Path.push_back(end);
    return Formed_Path;
  }

 else if (fabs(Diff_x) >= fabs(Diff_y))
 {
   step_size_x=step_size;
   iteration=fabs(Diff_x)/step_size_x;

   step_size_y= fabs(Diff_y)/iteration;

   step_size_yaw= fabs(Diff_yaw)/iteration;
 }

 else if (fabs(Diff_x) < fabs(Diff_y))
 {
   step_size_y=step_size;
   iteration=fabs(Diff_y)/step_size_y;

   step_size_x= fabs(Diff_x)/iteration;
   step_size_yaw= fabs(Diff_yaw)/iteration;
 }

//********** Start Path Discretization *************//

  // three stages are used.... Discretize start angle to path angle, discretize along the path , discretize from path angle to end
  (Diff_x>=0) ? x_sign=1 : x_sign=-1;
  (Diff_y>=0) ? y_sign=1 : y_sign=-1;
  (Diff_yaw>=0) ? yaw_sign=1 : yaw_sign=-1;
  (Diff_yaw_start_to_path>=0) ? yaw_start_sign=1 : yaw_start_sign=-1;
  (Diff_yaw_path_to_end>=0) ? yaw_end_sign=1 : yaw_end_sign=-1;



  geometry_msgs::Pose temp_pose=start;

// Discretize the start angle to the Path angle


     iteration_start= fabs(Diff_yaw_start_to_path)/step_size;
     step_size_yaw_start= fabs(Diff_yaw_start_to_path)/iteration_start;

     for (double i=0; i< iteration_start-1; i++)
     {
      temp_pose.orientation= pose_conversion::getQuaternionFromYaw(
                             (pose_conversion::getYawFromQuaternion(temp_pose.orientation))
                             + yaw_start_sign * step_size_yaw_start);
     Formed_Path.push_back(temp_pose);
     }



  Formed_Path.push_back(temp_pose);

// Discretize over the path
  temp_pose.orientation=pose_conversion::getQuaternionFromYaw(path_angle);
  Formed_Path.push_back(temp_pose);

  for (double i=0; i< iteration-1; i++)
  {
   temp_pose.position.x+= x_sign * step_size_x;
   temp_pose.position.y+= y_sign * step_size_y;
   temp_pose.orientation= pose_conversion::getQuaternionFromYaw(
                             (pose_conversion::getYawFromQuaternion(temp_pose.orientation))
                             + yaw_sign * step_size_yaw);
   temp_pose.orientation= end.orientation;
   Formed_Path.push_back(temp_pose);
  }


  // Discretize the path angle to the end angle

     iteration_end= fabs(Diff_yaw_path_to_end)/step_size;
     step_size_yaw_end= Diff_yaw_path_to_end/iteration_end;

     for (double i=0; i< iteration_end-1; i++)
     {
      temp_pose.orientation= pose_conversion::getQuaternionFromYaw(
                             (pose_conversion::getYawFromQuaternion(temp_pose.orientation))
                             + yaw_end_sign * step_size_yaw_end);
     Formed_Path.push_back(temp_pose);
     }


  // Path Discretization is Done

return Formed_Path;

}

