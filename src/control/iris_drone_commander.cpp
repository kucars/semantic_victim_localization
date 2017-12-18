#include "control/iris_drone_commander.h"

iris_drone_commander::iris_drone_commander(const ros::NodeHandle &nh, const ros::NodeHandle &nhPrivate):
nh_(nh),
nh_private_(nhPrivate)
{
 vehicle_ = new VehicleControlIris();
 localPosePub  = nh_.advertise<geometry_msgs::PoseStamped>("iris/mavros/setpoint_position/local", 10);
 ROS_INFO("start the iris drone commander...");
}


void iris_drone_commander::start(){

  ROS_INFO("test_NBZ: Starting vehicle. Waiting for current position information.");

  state = Command::STARTING_DRONE;
  ros::Rate rate(10);

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
     state = Command::READY_FOR_WAYPOINT;
    break;

  case Command::READY_FOR_WAYPOINT:
    if(ros::Time::now() - (vehicle_->setpoint_last_received) > ros::Duration(0.5))
    {
      ROS_INFO_THROTTLE(1.0,"READY_FOR_WAYPOINTS ---> Hovering");
      // Don't change gover pose, just header info
      hoverPose.header.stamp = ros::Time::now();
      hoverPose.header.frame_id = "world";
      currentGoal = hoverPose;
    }
    else
    {
      ROS_INFO_THROTTLE(1.0,"READY_FOR_WAYPOINTS ---> Navigating to WayPoint");
      goalPose = vehicle_->setpoint_last_pose;
      goalPose.header.stamp = ros::Time::now();
      goalPose.header.frame_id = "world";
      currentGoal = goalPose;
    }
    ROS_INFO_THROTTLE(1.0,"   - Current Goal is: [%f %f %f]",currentGoal.pose.position.x,currentGoal.pose.position.y,currentGoal.pose.position.z);
    localPosePub.publish(currentGoal);
    break;
  }
  ros::spinOnce();
  rate.sleep();
}
}


void iris_drone_commander::Takeoff()
{

  double x, y, z, yaw;
  ros::param::param<double>("~uav_pose_x" ,x, -1.5);
  ros::param::param<double>("~uav_pose_y" , y, 0.0);
  ros::param::param<double>("~uav_pose_z" , z, 1.0);
  ros::param::param<double>("~uav_pose_yaw" , yaw, 0.0);

  vehicle_->setWaypoint(x, y, z, yaw);

  ROS_INFO("Starting vehicle");
  vehicle_->start(x,y,z);
  ROS_INFO("Moving vehicle");
  vehicle_->moveVehicle(1.0);
  ROS_INFO("Done moving");

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
