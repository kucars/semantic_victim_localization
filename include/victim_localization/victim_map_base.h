#ifndef VICTIM_MAP_H
#define VICTIM_MAP_H

#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <victim_localization/raytracing.h>
#include <control/drone_communicator.h>
#include <grid_map_msgs/GridMap.h>
#include <cmath>
#include "math.h"
#include <iostream>
#include <string>
#include <vector>
#include "geometry_msgs/PointStamped.h"
#include <boost/filesystem.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/array.hpp>
#include <sensor_msgs/PointCloud.h>
#include "victim_localization/common.h"
#include <octomap_world/octomap_manager.h>


typedef geometry_msgs::Point Point;
typedef geometry_msgs::PoseStamped PoseStamped;

using namespace grid_map;

struct Status {
  bool victim_found;
  Position victim_loc;
};


class Victim_Map_Base
{

protected://get it from config ..
  double Prob_D_H;  //P(D|H)
  double Prob_D_Hc;  //P(D|Hc)
  double Prob_Dc_H; //P(Dc|H)
  double Prob_Dc_Hc; //P(Dc|Hc)

public:
  Victim_Map_Base(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private);
  ~Victim_Map_Base();

  drone_communicator *drone_comm;

  std::string map_topic;
  std::string layer_name="general";//="victim";
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher pub_map;
  ros::Subscriber sub_loc;
  ros::Publisher pub_polygon;
  double map_resol;
  bool detection_enabled;

  float const_;
  geometry_msgs::Pose current_loc_;
  double current_yaw_;
  Status map_status;


  double HFOV_deg;
  double VFOV_deg;
  double max_depth_d;
  double min_depth_d;
  double x_arena_max;
  double y_arena_max;
  double octomap_resol;
  double victim_found_prob;


  grid_map::GridMap map;
  grid_map::GridMap projected_map;
  grid_map::Polygon polygon;
  volumetric_mapping::OctomapManager *manager_;
  Raytracing *raytracing_;
  std::string victimMapName;



  //Detection_info//
  Position detect_victim_loc_;
  bool is_detect_;
  Point p1; // rectangle corners for projected map update
  Point p2;
  Point p3;
  Point p4;

  //**************//

  virtual void Update(){};
  virtual void runDetector(){};
  virtual Status getMapResultStatus();

  std::string VictimMapType();

  Position approximate_detect(Position x);
  //bool valid(Position loc);
  void publish_Map();
  grid_map::Polygon draw_FOV();
  void callbackdrawFOV(const PoseStamped &ps_stamped);
  std::string getlayer_name();
  void setlayer_name(std::string layer_);
  void setCurrentPose(geometry_msgs::Pose ps);
  void setDetectionResult(Status detection_status);
  void setOctomapManager(volumetric_mapping::OctomapManager *manager);

  grid_map::GridMap Project_3d_rayes_to_2D_plane();
  void setCameraSettings(double fov_h, double fov_v, double r_max, double r_min);
  grid_map::Polygon Update_region(grid_map::GridMap Map, geometry_msgs::Pose corner_);
  virtual void setDroneCommunicator(drone_communicator *drone_comm_);

};




#endif // VICTIM_MAP_H
