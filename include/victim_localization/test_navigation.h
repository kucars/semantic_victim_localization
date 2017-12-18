#ifndef TEST_NAVIGATION_H
#define TEST_NAVIGATION_H

#include "victim_localization/ReactivePathPlanner.h"
#include "control/vehicle_control_iris.h"
#include "victim_localization/Volumetric_Map_Manager.h"
#include "mutex"
#include "thread"

class test_navigation
{
  tf::TransformListener tf_;
public:
  test_navigation(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private );
  ReactivePathPlanner *planner_;
  VehicleControlIris *vehicle_;
  volumetric_mapping::OctomapManager *manager_;
  Volumetric_Map *Volumetric_Map_;
  costmap_2d::Costmap2DROS *CostMapROS_;
  std::vector <geometry_msgs::Pose> Setpoints_;
  std::thread thread_1;
  std::thread thread_2;


  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Subscriber sub_sp;
  ros::Publisher pub_sp;
  bool start_PS;
  geometry_msgs::PoseStamped hover_pose;

  void Configuration();
  void Takeoff();
  void executeplanner();
  void setpoints();
  void PublishCurrentPose(geometry_msgs::Pose p);
  void plannerthread();
  void PublishSPCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);


};

#endif // TEST_NAVIGATION_H
