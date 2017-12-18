#ifndef NAVIGATION_BASE_H
#define NAVIGATION_BASE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <victim_localization/volumetric_map_manager.h>
#include <octomap_world/octomap_manager.h>
#include <victim_localization/common.h>


class navigationBase
{
public:
  navigationBase();
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  volumetric_mapping::OctomapManager *manager_;
  geometry_msgs::Pose current_pose_;


  double nav_bounds_x_min_;
  double nav_bounds_y_min_;
  double nav_bounds_z_min_;
  double nav_bounds_x_max_;
  double nav_bounds_y_max_;
  double nav_bounds_z_max_;
  std::string methodName_;
  double d_close;
  double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);


  void setCurrentPose(geometry_msgs::Pose p);
  void setOctomapManager(volumetric_mapping::OctomapManager *manager);
  virtual std::string methodName(void);
  virtual bool GeneratePath(geometry_msgs::Pose end, std::vector<geometry_msgs::Pose> &Path);
  void setConfiguration(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                                         volumetric_mapping::OctomapManager *manager);

  virtual void start(){};



};

#endif // NAVIGATION_BASE_H
