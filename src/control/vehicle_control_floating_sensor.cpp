#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "control/vehicle_control_floating_sensor.h"
#include "victim_localization/common.h"


VehicleControlFloatingSensor::VehicleControlFloatingSensor():
  VehicleControlBase() // Call super class constructor
{
  ros::NodeHandle ros_node;

  // Callbacks
  sub_pose  = ros_node.subscribe("/floating_sensor/poseStamped", 1, &VehicleControlFloatingSensor::callbackPose, this);
  pub_twist = ros_node.advertise<geometry_msgs::Twist>("/floating_sensor/set_twist", 10);
  pub_pose  = ros_node.advertise<geometry_msgs::PoseStamped>("/floating_sensor/set_pose", 10);

  getPose();
}


// Update global position of UGV
void VehicleControlFloatingSensor::callbackPose(const geometry_msgs::PoseStamped& msg)
{
  // Save pose and velocities
  vehicle_current_pose_ = msg.pose;
  setpoint_last_received = ros::Time::now();


  if (std::isnan(vehicle_current_pose_.position.x) || std::isnan(vehicle_current_pose_.orientation.x))
  {
    std::cout << cc.red << "Current vehicle position not found..." << cc.reset;
    is_ready_ = false;
  }
  else
  {
    is_ready_ = true;
  }
}


bool VehicleControlFloatingSensor::isReady()
{
  return is_ready_;
}

bool VehicleControlFloatingSensor::isSationary(double threshold_sensitivity)
{
  // Doesn't check if we're stationary since the sensor teleports
  // Instead it checks if the sensor is in the desired location

  return isNear(vehicle_current_pose_, setpoint_, threshold_sensitivity );
}

void VehicleControlFloatingSensor::moveVehicle(double threshold_sensitivity)
{
//  geometry_msgs::PoseStamped temp_target;
 // temp_target.pose = setpoint_;

//  //ros::Time wait=ros::Time::now();

//  while (ros::Time::now() - wait < ros::Duration(1))
//   {
//   std::cout << "waiting...floating sensor." <<  (ros::Time::now()-wait).toSec() << std::endl;
//   pub_pose.publish(temp_target);
//   //ros::spinOnce();
//   ros::Rate(5).sleep();
//   }

//  ros::spinOnce();
//  ros::Rate(1).sleep();

//  return;

//  if (speed_ < 0)
//  {
    // Teleport sensor instantly
    geometry_msgs::PoseStamped setpoint_stamped;
    setpoint_stamped.pose = setpoint_;

    pub_pose.publish(setpoint_stamped);

    // Wait for sensor to reach destination
    while (ros::ok() && !isNear(setpoint_, vehicle_current_pose_, 1) )
    {
      ros::Rate(100).sleep();
      ros::spinOnce();
    }

//    return;
//  }

//  //Compute new velocities
//  //updateTwist();

//  double rate = 50;
//  geometry_msgs::PoseStamped temp_target_;
//  temp_target_.pose = vehicle_current_pose_;

//  std::cout << "time to target...." << time_to_target_ << std::endl;
//    for (int i=0; i<time_to_target_*rate; i++)
//  {
//    temp_target_.pose.position.x += twist_.linear.x/rate;
//    temp_target_.pose.position.y += twist_.linear.y/rate;
//    temp_target_.pose.position.z += twist_.linear.z/rate;

//    pub_pose.publish(temp_target_);

//    ros::Rate(rate).sleep();
//    ros::spinOnce();
//}
  /*
  // Publish speed so vehicle will move
  pub_twist.publish(twist_);
  std::cout << twist_ << "\n";
  // Sleep until we arrive at destination
  ros::Duration(time_to_target_).sleep();
  // Stop vehicle
  twist_.linear.x = 0;
  twist_.linear.y = 0;
  twist_.linear.z = 0;
  twist_.angular.x = 0;
  twist_.angular.y = 0;
  twist_.angular.z = 0;
  pub_twist.publish(twist_);
  */

  // Done, publish setpoint to make sure we're in target location
  pub_pose.publish(setpoint_stamped);
}


void VehicleControlFloatingSensor::setSpeed(double speed)
{
  speed_ = speed;
}

void VehicleControlFloatingSensor::setPath(nav_msgs::Path path)
{
  setpath_.clear();
  for (int i=0; i<path.poses.size(); i++)
  {
    setpath_.push_back(path.poses[i].pose);
  }
}

void VehicleControlFloatingSensor::setPath(std::vector<geometry_msgs::Pose> path)
{
  setpath_ = path;
}

void VehicleControlFloatingSensor::FollowPath(double threshold_sensitivity)
{
   for (int i=0;i<setpath_.size();i++)
   {
     setWaypoint(setpath_[i]);
     moveVehicle(threshold_sensitivity);
   }
}


void VehicleControlFloatingSensor::setWaypoint(double x, double y, double z, double yaw)
{
  // Position
  setpoint_.position.x = x;
  setpoint_.position.y = y;
  setpoint_.position.z = z;

  // Orientation
  setpoint_.orientation = pose_conversion::getQuaternionFromYaw(yaw);
}

void VehicleControlFloatingSensor::setWaypoint(double x, double y, double z, geometry_msgs::Quaternion orientation_)
{
  geometry_msgs::Pose setpoint_world;

  // Position
  setpoint_world.position.x = x;
  setpoint_world.position.y = y;
  setpoint_world.position.z = z;

  // Orientation
  setpoint_world.orientation = orientation_;

}

void VehicleControlFloatingSensor::setWaypoint(geometry_msgs::Pose p)
{
  setpoint_ = p;
}


void VehicleControlFloatingSensor::setWaypointIncrement(double x, double y, double z, double yaw)
{
  // Position
  setpoint_.position.x = vehicle_current_pose_.position.x + x;
  setpoint_.position.y = vehicle_current_pose_.position.y + y;
  setpoint_.position.z = vehicle_current_pose_.position.z + z;

  // Orientation
  double yaw_current =  pose_conversion::getYawFromQuaternion(vehicle_current_pose_.orientation);
  setpoint_.orientation =  pose_conversion::getQuaternionFromYaw(yaw_current + yaw);
}


void VehicleControlFloatingSensor::start(double x, double y, double z, double yaw)
{
  ros::Rate rate(10);
  while(ros::ok() && !is_ready_)
  {
    std::cout << cc.yellow << "Vehicle (floating sensor) not ready!\n" << cc.reset;

    ros::spinOnce();
    rate.sleep();
  }

  // Take off
  setWaypoint(x,y,z,yaw);


  std::cout << cc.green << "Done taking off\n" << cc.reset;

  is_ready_ = true;
}

void VehicleControlFloatingSensor::start()
{
  printf("starting...\n");
  ros::Rate rate(10);
  while(ros::ok() && !is_ready_)
  {
    std::cout << cc.yellow << "Iris not ready!\n" << cc.reset;
    ros::spinOnce();
    rate.sleep();
  }

  is_ready_ = true;
}

void VehicleControlFloatingSensor::Evaluate4Points(double x, double y, double z){
  geometry_msgs::Pose p;

  ROS_INFO("Perfroming -- 4Points Evaluation");

  // generate four points
  std::vector <geometry_msgs::Pose> FourPoints;
  p.position.x= x+0.5;
  p.position.y= y;
  p.position.z = z;
  p.orientation= pose_conversion::getQuaternionFromYaw(M_PI);
  FourPoints.push_back(p);
  p.position.x= x+0.5;
  p.position.y= y;
  p.position.z = z;
  p.orientation= pose_conversion::getQuaternionFromYaw((M_PI/2));
  FourPoints.push_back(p);
  p.position.x= x;
  p.position.y= y-0.5;
  p.orientation = pose_conversion::getQuaternionFromYaw((M_PI/2));
  FourPoints.push_back(p);
  p.position.x= x;
  p.position.y= y-0.5;
  p.orientation = pose_conversion::getQuaternionFromYaw(0);
  FourPoints.push_back(p);
  p.position.x= x-0.5;
  p.position.y= y;
  p.orientation = pose_conversion::getQuaternionFromYaw(0);
  FourPoints.push_back(p);
  p.position.x= x-0.5;
  p.position.y= y;
  p.orientation = pose_conversion::getQuaternionFromYaw((-M_PI/2));
  FourPoints.push_back(p);
  p.position.x= x;
  p.position.y= y+0.5;
  p.orientation = pose_conversion::getQuaternionFromYaw((-M_PI/2));
  FourPoints.push_back(p);

  for (int i=0; i<FourPoints.size(); i++)
  {
    setWaypoint(FourPoints[i]);
    std::cout <<FourPoints[i] << std::endl;
    moveVehicle(1);
    ros::Time lastTimeTurnTime=ros::Time::now();
    while(ros::Time::now() - lastTimeTurnTime < ros::Duration(0.5)) {ros::spinOnce(); ros::Rate(5).sleep();}
  }
}

void VehicleControlFloatingSensor::rotateOnTheSpot(){
  ROS_INFO("Perfroming -- rotation");
  geometry_msgs::PoseStamped rotatingPose;
  rotatingPose.pose.position.x = vehicle_current_pose_.position.x;
  rotatingPose.pose.position.y = vehicle_current_pose_.position.y;
  rotatingPose.pose.position.z = vehicle_current_pose_.position.z;
  int seqNum=0 ;
  double angleStep  = 0.05;  //before
  double angle = 0;
  bool done = false;
  double deltaTime = 1;
  ros::Time lastTimeTurnTime = ros::Time::now();
  ros::Rate rate(5);

  while(!done)
  {
    if(ros::Time::now() - lastTimeTurnTime > ros::Duration(deltaTime))
    {
      ROS_INFO_THROTTLE(0.2,"Turning around");
      tf::Quaternion quat = tf::Quaternion(tf::Vector3(0.0, 0.0, 1.0), angle);

      setpoint_.position.x    = rotatingPose.pose.position.x;
      setpoint_.position.y    = rotatingPose.pose.position.y;
      setpoint_.position.z    = rotatingPose.pose.position.z;
      setpoint_.orientation.x = quat[0] ;
      setpoint_.orientation.y = quat[1] ;
      setpoint_.orientation.z = quat[2] ;
      setpoint_.orientation.w = quat[3] ;
      //ROS_INFO("   - Trying to  rotate to: [%f %f %f %f]",setpoint_.pose.position.x,setpoint_.pose.position.y,setpoint_.pose.position.z,180*angle/M_PI);
      //ROS_INFO("   - Current Pose      is: [%f %f %f]",currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z);
      angle+=(angleStep*2.0*M_PI);
      if(angle>=2.0*M_PI)
        done = true;
      lastTimeTurnTime = ros::Time::now();
    }

    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.pose = setpoint_;

    pub_pose.publish(ps);
    ros::spinOnce();
    rate.sleep();
  }
}

void VehicleControlFloatingSensor::updateTwist()
{
  if (speed_ <= 0.0001)
  {
    printf("Stopping floating sensor vehicle\n");
    time_to_target_ = 0;
    twist_.linear.x = 0;
    twist_.linear.y = 0;
    twist_.linear.z = 0;
    return;
  }

  // Find direction vector (end point - start point)
  twist_.linear.x = setpoint_.position.x - vehicle_current_pose_.position.x;
  twist_.linear.y = setpoint_.position.y - vehicle_current_pose_.position.y;
  twist_.linear.z = setpoint_.position.z - vehicle_current_pose_.position.z;


  // Normalize and set speed accordingly
  double norm = sqrt(twist_.linear.x*twist_.linear.x + twist_.linear.y*twist_.linear.y + twist_.linear.z*twist_.linear.z);
  time_to_target_ = norm / speed_;

  twist_.linear.x = twist_.linear.x / time_to_target_;
  twist_.linear.y = twist_.linear.y / time_to_target_;
  twist_.linear.z = twist_.linear.z / time_to_target_;

  time_to_target_=0.1;
}

geometry_msgs::Pose VehicleControlFloatingSensor::getlastSP()
{
  return vehicle_current_pose_;
}

ros::Time VehicleControlFloatingSensor::getlastSPTime()
{
  return setpoint_last_received;
}
