#include "victim_localization/test_navigation.h"

test_navigation::test_navigation(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private ):
  nh_(nh),
  nh_private_(nh_private)
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

  vehicle_ = new VehicleControlIris();
  planner_ = new ReactivePathPlanner();
  planner_->setConfiguration(nh_,nh_private_,manager_);
  planner_->start();
}

void test_navigation::Takeoff()
{

  double x, y, z, yaw;
  ros::param::param<double>("~uav_pose_x" ,x, -1.5);
  ros::param::param<double>("~uav_pose_y" , y, 0.0);
  ros::param::param<double>("~uav_pose_z" , z, 1.0);
  ros::param::param<double>("~uav_pose_yaw" , yaw, 0.0);

  vehicle_->setWaypoint(x, y, z, yaw);

  printf("Starting vehicle\n");
  vehicle_->start(x,y,z);
  printf("Moving vehicle\n");
  vehicle_->moveVehicle(1.0);
  printf("Done moving\n");

  ros::Rate loop_rate(10);
  while (ros::ok())
  {
    vehicle_->moveVehicle(1);

    if (manager_->getMapSize().norm() <= 0.0)

      continue;

    if( vehicle_->isReady() && manager_->getMapSize().norm() > 0.0)
    {
      vehicle_->rotateOnTheSpot(x,y,z);
      break;
    }
  }
  ros::spinOnce();
  loop_rate.sleep();
}


void test_navigation::executeplanner()
{
   setpoints();

  for(int i=0;i<Setpoints_.size();i++)
  {
    printf("Staring the Planner in test navigation node... \n");

    thread_1 = std::thread(&test_navigation::PublishCurrentPose,this,vehicle_->getPose()); // keep publishing the drone current

    double grid_size_x, grid_size_y;                                                                        // location until the Planner service is done
    Volumetric_Map_->GetActiveOctomapSize(grid_size_x,grid_size_y);
    planner_->reactivePlannerServer->SetDynamicGridSize(grid_size_x,grid_size_y,0);

    std::vector <geometry_msgs::Pose> path;
    planner_->setCurrentPose(vehicle_->getPose());
    if (planner_->GeneratePath(Setpoints_[i],path)) {
      printf("path found...\n");
      thread_1.detach(); // detached the thread.

      for(int j=0;j<path.size();j++)
      {
        vehicle_->setWaypoint(path[j]);
        vehicle_->moveVehicle(1.0);
      }
      thread_1 = std::thread(&test_navigation::PublishCurrentPose,this,vehicle_->getPose()); // keep publishing the drone current pose
    }

    else
    {
      std::cout << "no path found let check the other setpoint... " << std::endl;
      vehicle_->moveVehicle(1.0);
    }

  }
}



void test_navigation::setpoints()
{
  geometry_msgs::Pose Setpoint_1;
  Setpoint_1.position.x= -4;
  Setpoint_1.position.y = 0;
  Setpoint_1.position.z = 1;
  Setpoint_1.orientation =  pose_conversion::getQuaternionFromYaw(2.0);

  geometry_msgs::Pose Setpoint_2;
  Setpoint_2.position.x= -4;
  Setpoint_2.position.y = 1;
  Setpoint_2.position.z = 1;
  Setpoint_2.orientation =  pose_conversion::getQuaternionFromYaw(0.0);

  geometry_msgs::Pose Setpoint_3;
  Setpoint_3.position.x= -1;
  Setpoint_3.position.y = 0;
  Setpoint_3.position.z = 0;
  Setpoint_3.orientation =  pose_conversion::getQuaternionFromYaw(2.0);
  Setpoints_.push_back(Setpoint_1);
  Setpoints_.push_back(Setpoint_2);
  Setpoints_.push_back(Setpoint_3);
}

void test_navigation::PublishCurrentPose(geometry_msgs::Pose p)
{
  while (ros::ok())
  {
    std::cout << "keep publishing ... \n";
    geometry_msgs::PoseStamped ps;
    ps.header.frame_id = "world";
    ps.header.stamp = ros::Time::now();
    ps.pose = p;
    //std::cout << "pose to go to: " << p << "\n";
    vehicle_->pub_setpoint.publish(ps);
    ros::spinOnce();
    ros::Rate(10).sleep();
  }
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
  test_->Takeoff();
  test_->executeplanner();

  return 0;
}
