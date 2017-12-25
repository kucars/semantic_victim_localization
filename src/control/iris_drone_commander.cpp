#include "control/iris_drone_commander.h"

iris_drone_commander::iris_drone_commander(const ros::NodeHandle &nh, const ros::NodeHandle &nhPrivate):
nh_(nh),
nh_private_(nhPrivate)
{
 vehicle_ = new VehicleControlIris();
 command_status = false;

 // Parameters
 std::string topic_ps;
 std::string topic_service_rotate;
 std::string topic_service_waypoint;
 std::string topic_service_path;
 std::string topic_command_status;

 ros::param::param("~topic_ps", topic_ps, std::string("iris/mavros/setpoint_position/local"));

 // communication topic for service
 ros::param::param("~topic_service_rotate", topic_service_rotate, std::string("iris/rotate"));
 ros::param::param("~topic_service_waypoint", topic_service_waypoint, std::string("iris/waypoint"));
 ros::param::param("~topic_service_path", topic_service_path, std::string("iris/path"));
 ros::param::param("~topic_service_command_status", topic_command_status, std::string("iris/command_status"));

 localPosePub  = nh_.advertise<geometry_msgs::PoseStamped>(topic_ps, 10);

 service_rotation = nh_private_.advertiseService(topic_service_rotate,&iris_drone_commander::execute_rotation,this);
 service_waypoint = nh_private_.advertiseService(topic_service_waypoint,&iris_drone_commander::execute_waypoint,this);
 service_path = nh_private_.advertiseService(topic_service_path,&iris_drone_commander::execute_path,this);
 service_status = nh_private_.advertiseService(topic_command_status,&iris_drone_commander::check_status,this);

 ROS_INFO("start the iris drone commander...");
}


void iris_drone_commander::start(){

  ROS_INFO("test_NBZ: Starting vehicle. Waiting for current position information.");

  state = Command::STARTING_DRONE;
  ros::Rate rate(30);

  while(ros::ok())
  {
  switch(state)
  {
  case Command::STARTING_DRONE:
    if( vehicle_->isReady()){
     state = Command::TAKEOFF;
     break;
    }
    break;
  case Command::TAKEOFF:
     this->Takeoff();
     state = Command::HOVER_IF_NO_WAYPOINT_RECEIVED;
    break;
  case Command::ROTATE:
    std::cout << "execute_four_waypoints"<< std::endl;
     vehicle_->Evaluate4Points(start_x,start_y,start_z);
     state = Command::HOVER_IF_NO_WAYPOINT_RECEIVED;
     command_status = true; // done
     std::cout << "done rotatting" << std::endl;
    break;

  case Command::SEND_WAYPOINT:
     vehicle_->moveVehicle(1);
     state = Command::HOVER_IF_NO_WAYPOINT_RECEIVED;
     command_status= true; // done executing the waypoint
    break;

  case Command::SEND_PATH:
     vehicle_->FollowPath();
     state = Command::HOVER_IF_NO_WAYPOINT_RECEIVED;
     command_status = true;  // done following the path
    break;


  case Command::HOVER_IF_NO_WAYPOINT_RECEIVED:
      hoverPose.header.stamp = ros::Time::now();
      hoverPose.header.frame_id = "world";
      hoverPose.pose = vehicle_->getlastSP();
      localPosePub.publish(hoverPose);

    ROS_INFO_THROTTLE(1.0,"   - HoverPose is: [%f %f %f]",hoverPose.pose.position.x,hoverPose.pose.position.y,hoverPose.pose.position.z);
    break;
  }
  ros::spinOnce();
  rate.sleep();
}
}


void iris_drone_commander::Takeoff()
{
  ros::param::param<double>("~uav_pose_x" ,start_x, -2);
  ros::param::param<double>("~uav_pose_y" , start_y, 0.0);
  ros::param::param<double>("~uav_pose_z" , start_z, 1.0);
  ros::param::param<double>("~uav_pose_yaw" , start_yaw, 3.14);

  vehicle_->setWaypoint(start_x, start_y, start_z, start_yaw);

  ROS_INFO("Starting vehicle");
  vehicle_->start(start_x, start_y, start_z, start_yaw);
  ROS_INFO("Moving vehicle");
  vehicle_->moveVehicle(1.0);
  ROS_INFO("Done moving");
}

bool iris_drone_commander::execute_rotation(victim_localization::rotate_action::Request &request,
                                         victim_localization::rotate_action::Response &respond)
{
  if (state!=Command::HOVER_IF_NO_WAYPOINT_RECEIVED)
    return false;
  else
  {
    state=Command::ROTATE;
    return true;
  }
}

bool iris_drone_commander::execute_waypoint(victim_localization::waypoint_action::Request &request,
                                            victim_localization::waypoint_action::Response &respond)
{
  command_status = false;
  vehicle_->setWaypoint(request.waypoint);
  state=Command::SEND_WAYPOINT;
  return true;
}

bool iris_drone_commander::execute_path(victim_localization::path_action::Request &request, victim_localization::path_action::Response &respond)
{
  command_status = false;
  vehicle_->setPath(request.path);
  state=Command::SEND_PATH;
  return true;
}

bool iris_drone_commander::check_status(victim_localization::status_action::Request &request, victim_localization::status_action::Response &respond)
{
 // std::cout << "the status is called " << command_status << std::endl;
 respond.resp =command_status;
 return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "iris_commander");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  iris_drone_commander *iris_drone_commander_;
  iris_drone_commander_ = new iris_drone_commander(nh,nh_private);
  iris_drone_commander_->start();
  return 0;
}
