#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "control/vehicle_control_iris.h"
#include "victim_localization/common.h"

VehicleControlIris::VehicleControlIris():
  VehicleControlBase() // Call super class constructor
{
  ros::NodeHandle ros_node;

  // Parameters
  std::string topic_Odometry;
  std::string topic_state;
  std::string topic_arming;
  std::string topic_set_mode;
  std::string topic_setpoint;
  ros::param::param("~topic_Odometry", topic_Odometry, std::string("iris/ground_truth/odometry"));
  //ros::param::param("~topic_Odometry", topic_Odometry, std::string("/iris/mavros/local_position/odom"));
  ros::param::param("~topic_setPose", topic_setpoint, std::string("localWOW"));
  ros::param::param("~topic_state", topic_state, std::string("iris/mavros/state"));
  ros::param::param("~topic_arming", topic_arming, std::string("iris/mavros/cmd/arming"));
  ros::param::param("~topic_set_mode", topic_set_mode, std::string("iris/mavros/set_mode"));
  ros::param::param("~nav_bounds_z_min", uav_height_min_, 0.5 );
  ros::param::param("~nav_bounds_z_max", uav_height_max_, 10.0);

  // Callbacks
  sub_odom = ros_node.subscribe(topic_Odometry, 1, &VehicleControlIris::callbackOdometry, this);
  sub_state = ros_node.subscribe(topic_state, 10, &VehicleControlIris::callbackState, this);
  sub_setpoint = ros_node.subscribe(topic_setpoint, 10,&VehicleControlIris::callbackSP, this);
  pub_setpoint = ros_node.advertise<geometry_msgs::PoseStamped>(topic_setpoint, 10);
  arming_client  = ros_node.serviceClient<mavros_msgs::CommandBool>(topic_arming);
  set_mode_client = ros_node.serviceClient<mavros_msgs::SetMode>(topic_set_mode);

  std::cout << cc.yellow << "Warning: 'uav_height_max_' monitoring not implimented in the VehicleControlIris class\n" << cc.reset;
}

// Update global position of UGV
void VehicleControlIris::callbackOdometry(const nav_msgs::Odometry& odom_msg)
{
  // Save pose and velocities
  vehicle_current_pose_ = odom_msg.pose.pose;
  vehicle_current_twist_= odom_msg.twist.twist;

  if (std::isnan(vehicle_current_pose_.position.x) || std::isnan(vehicle_current_pose_.orientation.x ))
  {
    std::cout << cc.red << "Current vehicle position not found..." << cc.reset;
    is_ready_ = false;
  }
  else
  {
    is_ready_ = true;
  }
}

void VehicleControlIris::callbackSP(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  setpoint_last_received = ros::Time::now();
  setpoint_last_pose.pose = msg->pose;
}


void VehicleControlIris::callbackState(const mavros_msgs::State& state_msg)
{
  // Save state
  vehicle_current_state_ = state_msg;
}


bool VehicleControlIris::isReady()
{
  return is_ready_;
}


bool VehicleControlIris::isStationary(double threshold_sensitivity)
{
  double max_speed = linear_speed_threshold_*threshold_sensitivity;
  double max_rate = angular_speed_threshold_*threshold_sensitivity;

  if (vehicle_current_twist_.linear.x < max_speed &&
      vehicle_current_twist_.linear.y < max_speed &&
      vehicle_current_twist_.linear.z < max_speed &&
      vehicle_current_twist_.angular.x < max_rate &&
      vehicle_current_twist_.angular.y < max_rate &&
      vehicle_current_twist_.angular.z < max_rate)
  {
    return true;
  }

  return false;
}


void VehicleControlIris::moveVehicle(double threshold_sensitivity)
{
  // Create stamped pose
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "";
  ps.header.stamp = ros::Time::now();
  ps.pose = setpoint_;

  // Convert setpoint to world frame
  geometry_msgs::Pose setpoint_world;
  setpoint_world = transformSetpoint2Global(setpoint_);
  std::cout << setpoint_world << std::endl;
  // Wait till we've reached the waypoint
  ros::Rate rate_(10);
  std::cout << "stuck...1\n";

  while(ros::ok() && (!isNear(setpoint_world, vehicle_current_pose_, threshold_sensitivity) || !isStationary(threshold_sensitivity) ) )
  {
    pub_setpoint.publish(ps);
    ros::spinOnce();
    rate_.sleep();
  }
  std::cout << "stuck...2\n";
}


void VehicleControlIris::setSpeed(double speed)
{
  std::cout << cc.yellow << "Warning: setSpeed(double) not implimented in the VehicleControlIris class\n" << cc.reset;

  if (speed < 0)
    return; //Ignore invalid speeds
}

void VehicleControlIris::setPath(nav_msgs::Path path)
{
  setpath_.clear();
  for (int i=0; i<path.poses.size(); i++)
  {
    setpath_.push_back(path.poses[i].pose);
  }
}

void VehicleControlIris::setPath(std::vector<geometry_msgs::Pose> path)
{
  setpath_ = path;
}

void VehicleControlIris::FollowPath(double threshold_sensitivity)
{
   for (int i=0;i<setpath_.size();i++)
   {
     setWaypoint(setpath_[i]);
     moveVehicle(threshold_sensitivity);
   }
}

void VehicleControlIris::setWaypoint(double x, double y, double z, double yaw)
{
  geometry_msgs::Pose setpoint_world;

  // Position
  setpoint_world.position.x = x;
  setpoint_world.position.y = y;
  setpoint_world.position.z = z;

  // Orientation
  setpoint_world.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  // Transform to setpoint frame
  setpoint_ = transformGlobal2Setpoint(setpoint_world);
}

void VehicleControlIris::setWaypoint(double x, double y, double z, geometry_msgs::Quaternion orientation_)
{
  geometry_msgs::Pose setpoint_world;

  // Position
  setpoint_world.position.x = x;
  setpoint_world.position.y = y;
  setpoint_world.position.z = z;

  // Orientation
  setpoint_world.orientation = orientation_;

  // Transform to setpoint frame
  setpoint_ = transformGlobal2Setpoint(setpoint_world);

}


void VehicleControlIris::setWaypoint(geometry_msgs::Pose p)
{
  // Transform to setpoint frame
  setpoint_ = transformGlobal2Setpoint(p);
}

void VehicleControlIris::rotateOnTheSpot(){
  ROS_INFO("Perfroming -- rotation");
  geometry_msgs::PoseStamped rotatingPose;
  rotatingPose.pose.position.x = vehicle_current_pose_.position.x;
  rotatingPose.pose.position.y = vehicle_current_pose_.position.y;
  rotatingPose.pose.position.z = vehicle_current_pose_.position.z;
  int seqNum=0 ;
  double angleStep  = 0.05;
  double angle = 3.14;
  bool done = false;
  double deltaTime = 1.0;
  ros::Time lastTimeTurnTime = ros::Time::now();
  ros::Rate rate(20);

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
    ps.header.frame_id = "base_link";
    ps.header.stamp = ros::Time::now();
    ps.pose = setpoint_;

    pub_setpoint.publish(ps);
    ros::spinOnce();
    rate.sleep();
  }
}

void VehicleControlIris::rotateOnTheSpot(double x, double y, double z){
  ROS_INFO("Perfroming -- rotation");
  geometry_msgs::PoseStamped rotatingPose;
  rotatingPose.pose.position.x = x;
  rotatingPose.pose.position.y = y;
  rotatingPose.pose.position.z = z;
  int seqNum=0 ;
  double yawStep  = 0.05;
  double yaw = 0;
  bool done = false;
  double deltaTime = 1.0;
  ros::Time lastTimeTurnTime = ros::Time::now();
  ros::Rate rate(20);

  while(ros::ok())
  {
    if(ros::Time::now() - lastTimeTurnTime > ros::Duration(deltaTime))
    {
      ROS_INFO_THROTTLE(0.2,"Turning around");
      geometry_msgs::Quaternion quat = pose_conversion::getQuaternionFromYaw(yaw);
      setpoint_.position.x    = x;
      setpoint_.position.y    = y;
      setpoint_.position.z    = z;

      setpoint_.orientation.x = quat.x ;
      setpoint_.orientation.y = quat.y ;
      setpoint_.orientation.z = quat.z ;
      setpoint_.orientation.w = quat.w ;
      //ROS_INFO("   - Trying to  rotate to: [%f %f %f %f]",setpoint_.pose.position.x,setpoint_.pose.position.y,setpoint_.pose.position.z,180*angle/M_PI);
      //ROS_INFO("   - Current Pose      is: [%f %f %f]",currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z);

      ROS_INFO("the values are x_pos %f, y_pos %f, yaw %f", setpoint_.position.x,setpoint_.position.y,yaw);

      yaw-=(yawStep*2.0*M_PI);
      if(yaw<=-2*M_PI) break;
      lastTimeTurnTime = ros::Time::now();
    }

     geometry_msgs::PoseStamped ps;
     ps.header.frame_id = "base_link";
     ps.header.stamp = ros::Time::now();
     ps.pose = setpoint_;

     pub_setpoint.publish(ps);
     ros::spinOnce();
     rate.sleep();
  }
}

void VehicleControlIris::Evaluate4Points(double x, double y, double z){

  ROS_INFO("Perfroming -- 4Points Evaluation");

  // generate four points
  std::vector <geometry_msgs::Pose> FourPoints;
  geometry_msgs::Pose p;
  p.position.x= x+0.5;
  p.position.y= y;
  p.position.z = z;
  p.orientation= pose_conversion::getQuaternionFromYaw(M_PI);
  FourPoints.push_back(p);
  p.position.x= x+0.5;
  p.position.y= y;
  p.position.z = z;
  p.orientation= pose_conversion::getQuaternionFromYaw(M_PI/2);
  FourPoints.push_back(p);
  p.position.x= x;
  p.position.y= y-0.5;
  p.orientation = pose_conversion::getQuaternionFromYaw(M_PI/2);
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
  p.orientation = pose_conversion::getQuaternionFromYaw(-M_PI/2);
  FourPoints.push_back(p);
  p.position.x= x;
  p.position.y= y+0.5;
  p.orientation = pose_conversion::getQuaternionFromYaw(-M_PI/2);
  FourPoints.push_back(p);

  for (int i=0; i<FourPoints.size(); i++)
  {
    setWaypoint(FourPoints[i]);
    moveVehicle(1.5);
  }
}
/*
 *   while(ros::ok())
  {
    if(ros::Time::now() - lastTimeTurnTime > ros::Duration(deltaTime))
    {
      ROS_INFO_THROTTLE(0.2,"Turning around");
      geometry_msgs::Quaternion quat = pose_conversion::getQuaternionFromYaw(yaw);
      if (x_inc_step<= 2*circleRadius)
      {
      setpoint_.position.x    = x - circleRadius + x_inc_step;
      setpoint_.position.y    =sqrt(pow(circleRadius,2) - pow(x - setpoint_.position.x,2))+y;
      setpoint_.position.z    = rotatingPose.pose.position.z;
      x_poses.push_back(setpoint_.position.x);
      y_poses.push_back(setpoint_.position.y);
      }
      else
      {
      x_poses.pop_back();
      y_poses.pop_back();
      setpoint_.position.x    = x_poses.back();
      setpoint_.position.y    = y -(y_poses.back()-y);
      setpoint_.position.z    = rotatingPose.pose.position.z;
      }
      setpoint_.orientation.x = quat.x ;
      setpoint_.orientation.y = quat.y ;
      setpoint_.orientation.z = quat.z ;
      setpoint_.orientation.w = quat.w ;
      //ROS_INFO("   - Trying to  rotate to: [%f %f %f %f]",setpoint_.pose.position.x,setpoint_.pose.position.y,setpoint_.pose.position.z,180*angle/M_PI);
      //ROS_INFO("   - Current Pose      is: [%f %f %f]",currentPose.pose.position.x,currentPose.pose.position.y,currentPose.pose.position.z);

      ROS_INFO("the values are x_pos %f, y_pos %f, yaw %f", setpoint_.position.x,setpoint_.position.y,yaw);

      yaw-=(yawStep*2.0*M_PI);
      x_inc_step+=(positionStep*2.0*circleRadius);
      if(yaw<=-2*M_PI) break;
      lastTimeTurnTime = ros::Time::now();
    }

     geometry_msgs::PoseStamped ps;
     ps.header.frame_id = "base_link";
     ps.header.stamp = ros::Time::now();
     ps.pose = setpoint_;

     pub_setpoint.publish(ps);
     ros::spinOnce();
     rate.sleep();
    //moveVehicle(0.8);
  }
}
*/
void VehicleControlIris::setWaypointIncrement(double x, double y, double z, double yaw)
{
  geometry_msgs::Pose setpoint_world;

  // Position
  setpoint_world.position.x = vehicle_current_pose_.position.x + x;
  setpoint_world.position.y = vehicle_current_pose_.position.y + y;
  setpoint_world.position.z = vehicle_current_pose_.position.z + z;

  // Orientation
  double yaw_current =  pose_conversion::getYawFromQuaternion(vehicle_current_pose_.orientation);
  setpoint_world.orientation =  pose_conversion::getQuaternionFromYaw(yaw_current + yaw);

  // Transform to setpoint frame
  setpoint_ = transformGlobal2Setpoint(setpoint_world);
}


void VehicleControlIris::start(double x, double y, double z, double yaw)
{
  printf("starting...\n");
  ros::Rate rate(10);
  while(ros::ok() && !is_ready_)
  {
    std::cout << cc.yellow << "Iris not ready!\n" << cc.reset;
    ros::spinOnce();
    rate.sleep();
  }

  // Take off
  std::cout << cc.green << "Taking off\n" << cc.reset;
  //setWaypoint(x,y,0.3,yaw);
  setOffboardState();
  //moveVehicle();
  setWaypoint(x,y,z,yaw);
  moveVehicle();
 // setWaypoint(x,y,z,yaw);


  std::cout << cc.green << "Done taking off\n" << cc.reset;

  is_ready_ = true;
}

void VehicleControlIris::start()
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

void VehicleControlIris::setOffboardState()
{
  ros::Rate rate_(20);
  mavros_msgs::SetMode offb_set_mode;
  offb_set_mode.request.custom_mode = "OFFBOARD";

  mavros_msgs::CommandBool arm_cmd;
  arm_cmd.request.value = true;

  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = "base_link";
  ps.header.stamp = ros::Time::now();
  ps.pose = setpoint_;

  std::cout << "Setting_OFFBOARD_MODE\n";

  while(ros::ok() && !vehicle_current_state_.armed){
    if(vehicle_current_state_.mode != "OFFBOARD") {
      //send a few setpoints before starting
      for(int i = 150; ros::ok() && i > 0; --i){
        pub_setpoint.publish(ps);
        ros::spinOnce();
        rate_.sleep();
      }

      set_mode_client.call(offb_set_mode);
    }

    else if (!vehicle_current_state_.armed) arming_client.call(arm_cmd);

    pub_setpoint.publish(ps);
    ros::spinOnce();
    rate_.sleep();
  }
  std::cout << cc.cyan <<"Request: Setting_OFFBOARD_MODE--> Request granted\n" << cc.reset;
}


geometry_msgs::Pose VehicleControlIris::transformSetpoint2Global (const geometry_msgs::Pose p_set)
{
  geometry_msgs::Pose p_global;

  // Apply a 90 degree clockwise rotation on the z-axis
  p_global.position.x = p_set.position.x;
  p_global.position.y = p_set.position.y;
  p_global.position.z = p_set.position.z;

  // Rotate orientation
  double yaw = pose_conversion::getYawFromQuaternion(p_set.orientation);
  //yaw -= M_PI_2;
  p_global.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  return p_global;
}


geometry_msgs::Pose VehicleControlIris::transformGlobal2Setpoint (const geometry_msgs::Pose p_global)
{
  geometry_msgs::Pose p_set;

  // Apply a 90 degree anti-clockwise rotation on the z-axis
  p_set.position.x = p_global.position.x;
  p_set.position.y = p_global.position.y;
  p_set.position.z = p_global.position.z;

  // Rotate orientation
  double yaw = pose_conversion::getYawFromQuaternion(p_global.orientation);
  //  yaw += M_PI_2;
  p_set.orientation = pose_conversion::getQuaternionFromYaw(yaw);

  return p_set;
}

geometry_msgs::Pose VehicleControlIris::getlastSP()
{
  return setpoint_last_pose.pose;
}

ros::Time VehicleControlIris::getlastSPTime()
{
  return setpoint_last_received;
}
