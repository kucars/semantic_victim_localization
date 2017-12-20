#ifndef REACTIVEPATHPLANNER_H
#define REACTIVEPATHPLANNER_H

#include <ros/ros.h>
#include "sspp/pathplanner.h"

#include <ros/package.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "sspp/distance_heuristic.h"
#include "rviz_visual_tools/rviz_visual_tools.h"
#include <octomap_world/octomap_manager.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sspp/sspp_srv.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include "victim_localization/reactive_planner_server.h"
#include "victim_localization/navigation_base.h"
#include "nav_msgs/Path.h"

class ReactivePathPlanner : public navigationBase
{
public:
  ReactivePathPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, volumetric_mapping::OctomapManager *manager);

  sspp::sspp_srv planningService;
  ReactivePlannerServer *reactivePlannerServer;

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  volumetric_mapping::OctomapManager *manager_;

 // ros::ServiceClient clientPath_;

  double uav_fixed_height;
  double extensionRange_;
  double boundingbox_x_;
  double boundingbox_y_;
  double boundingbox_z_;
  double dOvershoot_;

   std::string methodName(void);
   bool GeneratePath(geometry_msgs::Pose end, nav_msgs::Path &Path);
   void start();

};

#endif // REACTIVEPATHPLANNER_H
