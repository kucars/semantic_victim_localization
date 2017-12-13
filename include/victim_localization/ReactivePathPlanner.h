#ifndef REACTIVEPATHPLANNER_H
#define REACTIVEPATHPLANNER_H

#include "victim_localization/navigation_base.h"
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

class ReactivePathPlanner : public navigation_base
{
public:
  ReactivePathPlanner();
  sspp::sspp_srv planningService;

  double uav_fixed_height;
  double extensionRange_;
  double boundingbox_x_;
  double boundingbox_y_;
  double boundingbox_z_;
  double dOvershoot_;

   std::string methodName(void);
   bool GeneratePath(geometry_msgs::Pose end, std::vector<geometry_msgs::Pose> &Path);
};

#endif // REACTIVEPATHPLANNER_H
