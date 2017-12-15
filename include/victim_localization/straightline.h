#ifndef STRAIGHTLINE_H
#define STRAIGHTLINE_H

#include "victim_localization/navigation_base.h"


class straightLine : public navigationBase
{
public:
  straightLine();

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
