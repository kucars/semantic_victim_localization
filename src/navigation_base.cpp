#include "victim_localization/navigation_base.h"

navigationBase::navigationBase()
{
  ros::param::param("~close_distance", d_close, 0.1);
  ros::param::param("~nav_bounds_x_min", nav_bounds_x_min_,-10.0);
  ros::param::param("~nav_bounds_x_max", nav_bounds_x_max_, 10.0);
  ros::param::param("~nav_bounds_y_min", nav_bounds_y_min_,-10.0);
  ros::param::param("~nav_bounds_y_max", nav_bounds_y_max_, 10.0);
  ros::param::param("~nav_bounds_z_min", nav_bounds_z_min_, 0.5);
  ros::param::param("~nav_bounds_z_max", nav_bounds_z_max_, 5.0);
}

void navigationBase::setCurrentPose(geometry_msgs::Pose p)
{
  current_pose_ = p;
}

void navigationBase::setOctomapManager(volumetric_mapping::OctomapManager *manager)
{
  manager_ = manager;
}


void navigationBase::setConfiguration(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                                       volumetric_mapping::OctomapManager *manager)
{
  nh_=nh;
  nh_private_=nh_private;
  manager_ = manager;
}

double navigationBase::getDistance(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
  return sqrt(
    (p1.position.x-p2.position.x)*(p1.position.x-p2.position.x) +
    (p1.position.y-p2.position.y)*(p1.position.y-p2.position.y) +
    (p1.position.z-p2.position.z)*(p1.position.z-p2.position.z) );
}

bool navigationBase::GeneratePath(geometry_msgs::Pose end, std::vector<geometry_msgs::Pose> &Path)
{
  std::cout << "[Warning:] " << cc.red << " Navigation method is the Base\n" << cc.reset;
  return false;
}

std::string navigationBase::methodName(void)
{
  methodName_="Navigation_Base";
  return methodName_;
}


