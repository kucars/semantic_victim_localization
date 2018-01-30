#include <ros/ros.h>
#include "control/vehicle_control_base.h"
//#include "victim_localization/common.h"

VehicleControlBase::VehicleControlBase():
  is_ready_(false)
{
  ros::param::param("~distance_threshold", distance_threshold_, 0.3);
  ros::param::param("~angular_threshold", angular_threshold_, DEG2RAD(10.0));
  ros::param::param("~linear_speed_threshold", linear_speed_threshold_, 0.1);   //before 0.05
  ros::param::param("~angular_speed_threshold", angular_speed_threshold_, 0.1);  //before 0.03

  std::cout << "calling the ROSPARAM" << std::endl;
  this->SetVehicleROSParams();
  std::cout << "done\n";
}


geometry_msgs::Pose VehicleControlBase::getPose()
{
  return vehicle_current_pose_;
}

geometry_msgs::Twist VehicleControlBase::getTwist()
{
  return vehicle_current_twist_;
}

geometry_msgs::Point VehicleControlBase::getPosition()
{
  return vehicle_current_pose_.position;
}

mavros_msgs::State VehicleControlBase::getState()
{
  return vehicle_current_state_;
}


geometry_msgs::Quaternion VehicleControlBase::getOrientation()
{
  vehicle_current_pose_.orientation;
}

double VehicleControlBase::getYaw()
{
  return pose_conversion::getYawFromQuaternion( getOrientation() );
}


double VehicleControlBase::getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return sqrt(
    (p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) +
    (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) +
    (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z) );
}

double VehicleControlBase::getAngularDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  double yaw1 = pose_conversion::getYawFromQuaternion(p1.orientation);
  double yaw2 = pose_conversion::getYawFromQuaternion(p2.orientation);

  // Set differnce from -pi to pi
  double yaw_diff = fmod(yaw1 - yaw2, 2*M_PI);

  if (yaw_diff > M_PI)
    yaw_diff = yaw_diff - 2*M_PI;

  else if (yaw_diff < -M_PI)
    yaw_diff = yaw_diff + 2*M_PI;

  //std::cout << yaw1 << " " << yaw2 <<std::endl;
  //std::cout << "yaw_diff: " << yaw_diff <<std::endl;
  return yaw_diff;
}

bool VehicleControlBase::isNear(double p1, double p2, double threshold_sensitivity )
{
  if (fabs(p1-p2)< distance_threshold_*threshold_sensitivity)
  {
    return true;
  }
  return false;
}

bool VehicleControlBase::isNear(const geometry_msgs::Pose p_target, const geometry_msgs::Pose p_current, double threshold_sensitivity){

  // std::cout << fabs(getAngularDistance(p_target, p_current)) << "< " <<  angular_threshold_*threshold_sensitivity << std::endl;
  // std::cout << getDistance(p_target, p_current) << "< " <<  distance_threshold_*threshold_sensitivity << std::endl;
  // std::cout << "theshold Dis: " << threshold_sensitivity << std::endl;

  if (
    getDistance(p_target, p_current) < distance_threshold_*threshold_sensitivity &&
    fabs(getAngularDistance(p_target, p_current)) < angular_threshold_*threshold_sensitivity )
  {
    return true;
  }

  return false;
}

void VehicleControlBase::SetVehicleROSParams(){
  std::string topic_setPose1;
  std::string topic_Odometry, topic_Pose,topic_setPose,topic_rgb_image,topic_depth_image,topic_pointcloud,topic_thermal_image;
  std::string camera_optical_frame,camera_thermal_frame,base_frame;
  XmlRpc::XmlRpcValue param;

  int vehicle_type;
  ros::param::param<int>("~vehicle_type", vehicle_type, 1);
  std::string vehicle_num=std::to_string(vehicle_type);

  ros::NodeHandle nh_private;
  // get Vehicle topics
    nh_private.getParam(ros::this_node::getName()+"/Topics", param);
    topic_Odometry=(std::string)param[vehicle_num]["topic_Odometry"];
    topic_Pose=(std::string)param[vehicle_num]["topic_Pose"];
    topic_setPose=(std::string)param[vehicle_num]["topic_setPose"];
    topic_rgb_image=(std::string)param[vehicle_num]["topic_rgb_image"];
    topic_depth_image=(std::string)param[vehicle_num]["topic_depth_image"];
    topic_depth_image=(std::string)param[vehicle_num]["topic_thermal_image"];
    topic_pointcloud=(std::string)param[vehicle_num]["topic_pointcloud"];

  // get Vehicle frames
    nh_private.getParam(ros::this_node::getName()+"/Frames", param);
    camera_optical_frame=(std::string)param[vehicle_num]["camera_optical_frame"];
    camera_optical_frame=(std::string)param[vehicle_num]["camera_thermal_frame"];
    base_frame=(std::string)param[vehicle_num]["base_frame"];

  // Update the topics & frames in ROS::PARAM
    ros::param::del(ros::this_node::getName()+"/Topics");
    ros::param::del(ros::this_node::getName()+"/Frames");

    ros::param::set(ros::this_node::getName()+"/topic_Odometry",topic_Odometry);
    ros::param::set(ros::this_node::getName()+"/topic_Pose",topic_Pose);
    ros::param::set(ros::this_node::getName()+"/topic_setPose",topic_setPose);
    ros::param::set(ros::this_node::getName()+"/topic_rgb_image",topic_rgb_image);
    ros::param::set(ros::this_node::getName()+"/topic_depth_image",topic_depth_image);
    ros::param::set(ros::this_node::getName()+"/topic_thermal_image",topic_thermal_image);
    ros::param::set(ros::this_node::getName()+"/topic_pointcloud",topic_pointcloud);
    ros::param::set(ros::this_node::getName()+"/camera_optical_frame",camera_optical_frame);
    ros::param::set(ros::this_node::getName()+"/camera_thermal_frame",camera_thermal_frame);
    ros::param::set(ros::this_node::getName()+"/base_frame",base_frame);

    // override other nodes params
    ros::param::set(ros::this_node::getName()+"/costmap/robot_base_frame",base_frame);
}
