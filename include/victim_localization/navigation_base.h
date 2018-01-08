#ifndef NAVIGATION_BASE_H
#define NAVIGATION_BASE_H

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <tf_conversions/tf_eigen.h>
#include <victim_localization/volumetric_map_manager.h>
#include <octomap_world/octomap_manager.h>
#include <victim_localization/common.h>


class navigationBase
{
public:
  navigationBase();
  geometry_msgs::Pose current_pose_;


  double nav_bounds_x_min_;
  double nav_bounds_y_min_;
  double nav_bounds_z_min_;
  double nav_bounds_x_max_;
  double nav_bounds_y_max_;
  double nav_bounds_z_max_;
  double uav_fixed_height;

  std::string methodName_;
  double d_close;
  double getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2);


  void setCurrentPose(geometry_msgs::Pose p);

  virtual std::string methodName(void);
  virtual bool GeneratePath(geometry_msgs::Pose end, nav_msgs::Path &Path);
  virtual bool GeneratePath(geometry_msgs::Pose end, std::vector<geometry_msgs::Pose> &Path);

  virtual void start(){};
  virtual void SetDynamicGridSize(double x, double y,double z){};
  virtual void SetOriginPose(double x, double y,double z){};

  std::vector<geometry_msgs::Pose> Path_discretization(geometry_msgs::Pose start, geometry_msgs::Pose end, double step_size);

};

#endif // NAVIGATION_BASE_H
