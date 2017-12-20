#include "victim_localization/test_navigation.h"

test_navigation::test_navigation(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private ):
  nh_(nh),
  nh_private_(nh_private),
  Selectpath_(false)
{
  ROS_INFO("test_navigation: Begin!");
}

void test_navigation::Configuration()
{
  manager_ = new volumetric_mapping::OctomapManager(nh_, nh_private_);
  Volumetric_Map_ = new Volumetric_Map(manager_);

  CostMapROS_ = new costmap_2d::Costmap2DROS ("costmap",tf_);
  CostMapROS_->start();
  Volumetric_Map_->SetCostMapRos(CostMapROS_);

  planner_ = new ReactivePathPlanner(nh_,nh_private_,manager_);
  planner_->start();

  drone_communicator_ = new drone_communicator(nh_,nh_private_,manager_);
}


void test_navigation::state_machine()
{
  ROS_INFO("test_navigation... started");
  state = NavigationState::STARTING_ROBOT;

  ros::Rate loop_rate(30);
  while (ros::ok()){

    switch(state)
    {

    case NavigationState::STARTING_ROBOT:

      if (!drone_communicator_->GetStatus()) break;
      state = NavigationState::STARTING_ROBOT_COMPLETE;
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
      ROS_INFO("Move to Position [ %f %f %f]",path_.poses[0].pose.position.x,
      path_.poses[0].pose.position.y,path_.poses[0].pose.position.z);

      planner_->setCurrentPose(drone_communicator_->GetPose());

      //if (!drone_communicator_->Execute_waypoint(path_.poses[0].pose)) break;
      Volumetric_Map_->GetActiveOctomapSize(grid_size_x,grid_size_y);
      planner_->reactivePlannerServer->SetDynamicGridSize(grid_size_x,grid_size_y,0);

      Volumetric_Map_->GetActiveOrigin(grid_origin_x,grid_origin_y);
      planner_->reactivePlannerServer->SetOriginPose(grid_origin_x,grid_origin_x,round(drone_communicator_->GetPose().position.z));


      if (planner_->GeneratePath(path_.poses[0].pose,path_to_waypoint)) {
        printf("path found...\n");
        drone_communicator_->Execute_path(path_to_waypoint);
      }
      state = NavigationState::WAITING_FOR_WAYPOINT;
      break;

    case(NavigationState::WAITING_FOR_WAYPOINT):
      if (!drone_communicator_->GetStatus()) break;
       ROS_INFO("WAYPOINT.... REACHED");
       state= NavigationState::IDEL;
      break;

    case(NavigationState::NAVIGATION_PATH):
       Num_points = path_.poses.size()-1;
      ROS_INFO("Move to path started with [ %f %f %f] and end with  [ %f %f %f] "
               ,path_.poses[0].pose.position.x,path_.poses[0].pose.position.y,
                path_.poses[0].pose.position.z,path_.poses[Num_points].pose.position.x,
                path_.poses[Num_points].pose.position.y,path_.poses[Num_points].pose.position.z);

      if (!drone_communicator_->Execute_path(path_)) break;

      state = NavigationState::WAITING_FOR_PATH;
      break;

    case(NavigationState::WAITING_FOR_PATH):
      if (!drone_communicator_->GetStatus()) break;
       ROS_INFO("PATH.... REACHED");
       state= NavigationState::IDEL;
      break;

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
  Setpoint_1.pose.position.x= -2.5;
  Setpoint_1.pose.position.y = 1;
  Setpoint_1.pose.position.z = 1;
  Setpoint_1.pose.orientation =  pose_conversion::getQuaternionFromYaw(0.0);

  geometry_msgs::PoseStamped Setpoint_2;
  Setpoint_2.pose.position.x= -4;
  Setpoint_2.pose.position.y = 0;
  Setpoint_2.pose.position.z = 1;
  Setpoint_2.pose.orientation =  pose_conversion::getQuaternionFromYaw(0.0);

  geometry_msgs::PoseStamped Setpoint_3;
  Setpoint_3.pose.position.x= -4.7;
  Setpoint_3.pose.position.y = -2.7;
  Setpoint_3.pose.position.z = 1;
  Setpoint_3.pose.orientation =  pose_conversion::getQuaternionFromYaw(2.0);
  path_.poses.push_back(Setpoint_1);
  path_.poses.push_back(Setpoint_2);
  path_.poses.push_back(Setpoint_3);
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

