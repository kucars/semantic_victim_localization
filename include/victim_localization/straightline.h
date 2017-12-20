#ifndef STRAIGHTLINE_H
#define STRAIGHTLINE_H

#include "victim_localization/navigation_base.h"


class straightLine : public navigationBase
{
public:
  straightLine(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private, volumetric_mapping::OctomapManager *manager);

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  volumetric_mapping::OctomapManager *manager_;
  double uav_fixed_height;
  double extensionRange_;
  double boundingbox_x_;
  double boundingbox_y_;
  double boundingbox_z_;
  double dOvershoot_;

  Eigen::Vector3d boundingbox_;

  std::string methodName(void);
  bool GeneratePath(geometry_msgs::Pose end, std::vector<geometry_msgs::Pose> &Path);
  void start(){};

};

#endif // STRAIGHTLINE_H
