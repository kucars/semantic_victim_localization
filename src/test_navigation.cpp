#include "victim_localization/test_navigation.h"

test_navigation::test_navigation(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private ):
  nh_(nh),
  nh_private_(nh_private),
  Selectpath_(false)
{
  visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/Path_points"));
  visualTools->loadMarkerPub();

  visualTools->deleteAllMarkers();
  visualTools->enableBatchPublishing();

  ROS_INFO("test_navigation: Begin!");
}

int index_=0;

void test_navigation::Configuration()
{
  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);
  Volumetric_Map_ = new Volumetric_Map(manager_);

  CostMapROS_ = new costmap_2d::Costmap2DROS ("costmap",tf_);
  CostMapROS_->start();
  Volumetric_Map_->SetCostMapRos(CostMapROS_);

  planner_ = new ReactivePathPlanner(nh_,nh_private_,manager_);
  planner_->start();

  drone_communicator_ = new vehicle_communicator(nh_,nh_private_,manager_);
}


void test_navigation::state_machine()
{
  ROS_INFO("test_navigation... started");
  state = NavigationState::STARTING_ROBOT;

  ros::Rate loop_rate(2);
  while (ros::ok()){

    switch(state)
    {

    case NavigationState::STARTING_ROBOT:

      if (!drone_communicator_->GetStatus()) break;
      state = NavigationState::STARTING_ROBOT_COMPLETE;
      //this->GetTestPath3();
      this->GetTestPath();
      if (Selectpath_)
      {
        state = NavigationState::NAVIGATION_PATH;
        break;
      }
      else
      {
        state = NavigationState::NAVIGATION_WAYPOINT;
        break;
      }
    case(NavigationState::NAVIGATION_WAYPOINT):
      ROS_INFO("Move to Position [ %f %f %f]",path_.poses[index_].pose.position.x,
      path_.poses[index_].pose.position.y,path_.poses[index_].pose.position.z);

      planner_->setCurrentPose(drone_communicator_->GetPose());

      //if (!drone_communicator_->Execute_waypoint(path_.poses[0].pose)) break;
      Volumetric_Map_->GetActiveOctomapSize(grid_size_x,grid_size_y);
      planner_->reactivePlannerServer->SetDynamicGridSize(grid_size_x,grid_size_y,0);

      Volumetric_Map_->GetActiveOrigin(grid_origin_x,grid_origin_y);
      planner_->reactivePlannerServer->SetOriginPose((grid_origin_x),(grid_origin_y),round(drone_communicator_->GetPose().position.z));

      path_to_waypoint.poses.clear();  // Initially clear the path

      if (planner_->GeneratePath(path_.poses[index_].pose,path_to_waypoint)) {
        printf("path found...\n");

        drone_communicator_->Execute_path(path_to_waypoint);
        std::cout << "navigating to point index: " << index_ << std::endl;
      }
      state = NavigationState::WAITING_FOR_WAYPOINT;
      break;

    case(NavigationState::WAITING_FOR_WAYPOINT):
      if (!drone_communicator_->GetStatus()) break;
       ROS_INFO("WAYPOINT.... REACHED");
       if (index_==0)
       {
       state= NavigationState::IDEL;
      break;
       }
       else
       {
         state=NavigationState::NAVIGATION_WAYPOINT;
        index_=index_+1;
        break;
       }

    case(NavigationState::NAVIGATION_PATH):
      visualTools->deleteAllMarkers();
       number_of_path = Final_paths.size()-1;
       std::cout << "path_size " << number_of_path << std::endl;
       Path_size = Final_paths[index_].size()-1;

     // ROS_INFO("Move to path started with [ %f %f %f] and end with  [ %f %f %f] "
     //          ,(Final_paths[index_][0]).position.x,(Final_paths.[index_][0]).position.y,
     //           (Final_paths.[0][Path_size]).position.z,(Final_paths.[index_][Path_size]).position.x,
     //           (Final_paths.[index_][Path_size]).position.y,(Final_paths.[index_][Path_size]).position.z);

       std::cout << "index path " << index_ << std::endl;
      if (!drone_communicator_->Execute_path(Final_paths[index_])) break;
      std::cout << "Done with index path " << index_ << std::endl;


      visualTools->publishPath(Final_paths[index_],rviz_visual_tools::RED,rviz_visual_tools::LARGE,"path_");
      visualTools->trigger();

      std::cout << "the path is published.....\n";

      state = NavigationState::WAITING_FOR_PATH;
      break;

    case(NavigationState::WAITING_FOR_PATH):
      std::cout << "waiting for the drone to do its job.\n";
      if (!drone_communicator_->GetStatus()) break;
      ROS_INFO("PATH.... REACHED");
       if (index_==number_of_path)
       {
       state= NavigationState::IDEL;
      break;
       }
       else
       {
       state=NavigationState::NAVIGATION_PATH;
        index_=index_+1;
        break;
       }
    case(NavigationState::IDEL):
      ROS_INFO("The navigation test is done");
      break;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}

void test_navigation::executeplanner()
{
  this->GetTestPath();

  for(int i=0;i<path_.poses.size();i++)
  {
    printf("Staring the Planner in test navigation node... \n");

    //thread_1 = std::thread(&test_navigation::PublishCurrentPose,this,vehicle_->getPose()); // keep publishing the drone current

    double grid_size_x, grid_size_y;                                                                        // location until the Planner service is done
    Volumetric_Map_->GetActiveOctomapSize(grid_size_x,grid_size_y);
    planner_->reactivePlannerServer->SetDynamicGridSize(grid_size_x,grid_size_y,0);
    nav_msgs::Path path;
    planner_->setCurrentPose(drone_communicator_->GetPose());
    if (planner_->GeneratePath(path_.poses[i].pose,path)) {
      printf("path found...\n");
      drone_communicator_->Execute_path(path);
    }

    else
    {
      std::cout << "no path found for setpoint " << i <<  "let check the other setpoint..."  << std::endl;
    }

  }
}


void test_navigation::GetTestPath()
{
  //thread_1 = std::thread(&test_navigation::PublishCurrentPose,this,vehicle_->getPose()); // keep publishing the drone current

  geometry_msgs::PoseStamped Setpoint_1;
  Setpoint_1.pose.position.x= -4.7;
  Setpoint_1.pose.position.y = 1.3;
  Setpoint_1.pose.position.z = 1;
  Setpoint_1.pose.orientation =  pose_conversion::getQuaternionFromYaw(0.0);

  geometry_msgs::PoseStamped Setpoint_2;
  Setpoint_2.pose.position.x= -1.5;
  Setpoint_2.pose.position.y = 0;
  Setpoint_2.pose.position.z = 1;
  Setpoint_2.pose.orientation =  pose_conversion::getQuaternionFromYaw(0.0);

  geometry_msgs::PoseStamped Setpoint_3;
  Setpoint_3.pose.position.x= -2;
  Setpoint_3.pose.position.y = -2.7;
  Setpoint_3.pose.position.z = 1;
  Setpoint_3.pose.orientation =  pose_conversion::getQuaternionFromYaw(2.0);
  path_.poses.push_back(Setpoint_1);
  path_.poses.push_back(Setpoint_2);
  path_.poses.push_back(Setpoint_3);
  path_.poses.push_back(Setpoint_1);
  path_.poses.push_back(Setpoint_2);
  path_.poses.push_back(Setpoint_3);
}

void test_navigation::GetTestPath2()
{
  //thread_1 = std::thread(&test_navigation::PublishCurrentPose,this,vehicle_->getPose()); // keep publishing the drone current
  double step_size=0.2;

  geometry_msgs::Pose Setpoint_1;
  Setpoint_1.position.x= -1.5;
  Setpoint_1.position.y = 0;
  Setpoint_1.position.z = 1;
  Setpoint_1.orientation =  pose_conversion::getQuaternionFromYaw(0.0);
  collection_of_poses.push_back(Setpoint_1);
  while(Setpoint_1.position.x>-5)
  {
    Setpoint_1.position.x-=step_size;
    collection_of_poses.push_back(Setpoint_1);
  }


  geometry_msgs::PoseStamped Setpoint_;
  Setpoint_.pose.position.x= -1.5;
  Setpoint_.pose.position.y = 0;
  Setpoint_.pose.position.z = 1;
  Setpoint_.pose.orientation =  pose_conversion::getQuaternionFromYaw(0.0);
  path_.poses.push_back(Setpoint_);
  std::cout << "poses are \n";
  while(Setpoint_.pose.position.x>-5)
  {
    Setpoint_.pose.position.x-=step_size;
    path_.poses.push_back(Setpoint_);
    std::cout << Setpoint_ << "\n";
  }

}

void test_navigation::GetTestPath3()
{
  //thread_1 = std::thread(&test_navigation::PublishCurrentPose,this,vehicle_->getPose()); // keep publishing the drone current
  double step_size=0.1;

  std::vector<geometry_msgs::Pose> selected_poses;

  geometry_msgs::Pose selected_pose;
  selected_pose.position.x=-4.7;
  selected_pose.position.y=1.3;
  selected_pose.position.z=1;
  selected_pose.orientation =  pose_conversion::getQuaternionFromYaw(0.0);


  selected_poses.push_back(selected_pose);

  selected_pose.position.x=-4.42;
  selected_pose.position.y=2.48;
  selected_pose.position.z=1;
  selected_pose.orientation =  pose_conversion::getQuaternionFromYaw(-M_PI/2);

  selected_poses.push_back(selected_pose);

  selected_pose.position.x=-4.42;
  selected_pose.position.y=-2.650972;
  selected_pose.position.z=1;
  selected_pose.orientation =  pose_conversion::getQuaternionFromYaw(0);

  selected_poses.push_back(selected_pose);

  selected_pose.position.x= 4.42;
  selected_pose.position.y=-2.650972;
  selected_pose.position.z=1;
  selected_pose.orientation =  pose_conversion::getQuaternionFromYaw(M_PI/2);


  selected_poses.push_back(selected_pose);

  selected_pose.position.x= 4.5;
  selected_pose.position.y=0;
  selected_pose.position.z=1;
  selected_pose.orientation =  pose_conversion::getQuaternionFromYaw(M_PI);


  selected_poses.push_back(selected_pose);

  //*******generate path **************//
  for (int i=0; i<selected_poses.size()-1; i++)
  {
    std::vector<geometry_msgs::Pose> obtained_path;
    obtained_path = planner_->Path_discretization(selected_poses[i],selected_poses[i+1],step_size);
    Final_paths.push_back(obtained_path);

   // for (int j=0; j<Final_paths[i].size() ; j++)
    //std::cout << "path index " << i << " has values" << Final_paths[i][j] << std::endl;
  }

   std::cout << "Done Generating paths\n";
}


std::vector<geometry_msgs::Pose>
    test_navigation::Path_discretization(geometry_msgs::Pose start,
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

      step_size_y= Diff_y/iteration;

      step_size_yaw= Diff_yaw/iteration;
    }

    else if (fabs(Diff_x) < fabs(Diff_y))
    {
      step_size_y=step_size;
      iteration=fabs(Diff_y)/step_size_y;

      step_size_x= Diff_x/iteration;
      step_size_yaw= Diff_yaw/iteration;
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


//     iteration_start= fabs(Diff_yaw_start_to_path)/step_size;
//     step_size_yaw_start= Diff_yaw_start_to_path/iteration_start;

//     for (double i=0; i< iteration_start-1; i++)
//     {
//      temp_pose.orientation= pose_conversion::getQuaternionFromYaw(
//                             (pose_conversion::getYawFromQuaternion(temp_pose.orientation))
//                             + yaw_start_sign * step_size_yaw_start);
//     Formed_Path.push_back(temp_pose);
//     }



   // Formed_Path.push_back(temp_pose);

  // Discretize over the path
   //  temp_pose.orientation=pose_conversion::getQuaternionFromYaw(path_angle);
  //   Formed_Path.push_back(temp_pose);

     for (double i=0; i< iteration-1; i++)
     {
      temp_pose.position.x+= x_sign * step_size_x;
      temp_pose.position.y+= y_sign * step_size_y;
//      temp_pose.orientation= pose_conversion::getQuaternionFromYaw(
//                             (pose_conversion::getYawFromQuaternion(temp_pose.orientation))
//                             + yaw_sign * step_size_yaw);
//    temp_pose.orientation= end.orientation;
      Formed_Path.push_back(temp_pose);
     }


     // Discretize the path angle to the end angle

//     iteration_end= fabs(Diff_yaw_path_to_end)/step_size;
//     step_size_yaw_end= Diff_yaw_path_to_end/iteration_end;

//     for (double i=0; i< iteration_end-1; i++)
//     {
//      temp_pose.orientation= pose_conversion::getQuaternionFromYaw(
//                             (pose_conversion::getYawFromQuaternion(temp_pose.orientation))
//                             + yaw_end_sign * step_size_yaw_end);
//     Formed_Path.push_back(temp_pose);
//     }


     // Path Discretization is Done

   return Formed_Path;

   }


int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "test_navigation");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //Create an object of class test_navigation
  test_navigation *test_;
  test_ = new test_navigation(nh,nh_private);
  test_->Configuration();
  test_->state_machine();
  ros::spin();

  return 0;
}
