#ifndef PLOTRESULTS_H
#define PLOTRESULTS_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Geometry>
#include <geometry_msgs/Pose.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "rviz_visual_tools/rviz_visual_tools.h"
#include <victim_localization/common.h>
#include <octomap_world/octomap_manager.h>
#include <victim_localization/volumetric_map_manager.h>
#include <string.h>

using namespace std;
class visualizeResults
{
public:
    visualizeResults(const ros::NodeHandle &nh_,
                     const ros::NodeHandle &nh_private_ );

    ros::NodeHandle nh;
    ros::NodeHandle nh_private;

    rviz_visual_tools::RvizVisualToolsPtr visualTools;
    volumetric_mapping::OctomapManager *manager_;

    std::string resultPath;
    std::vector<geometry_msgs::Pose> Path;
    void SetPathtoResults(std::string path);
    void trigger();


};

#endif // PLOTRESULTS_H
