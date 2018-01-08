#include "control/drone_communicator.h"

drone_communicator::drone_communicator(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private,
                                       volumetric_mapping::OctomapManager *mapManager):
  nh_(nh),
  nh_private_(nh_private),
  manager_(mapManager),
  current_drone_status(false),
  previous_time(ros::Time::now())
{
  // Paramteters
  std::string topic_pose;
  std::string topic_service_rotate;
  std::string topic_service_waypoint;
  std::string topic_service_path;
  std::string topic_service_path2;
  std::string topic_command_status;



  ros::param::param("~topic_pose", topic_pose, std::string("iris/mavros/local_position/pose"));

  // communication topic for service
  ros::param::param("~topic_service_rotate", topic_service_rotate, std::string("/iris_commander/iris/rotate"));
  ros::param::param("~topic_service_waypoints", topic_service_waypoint, std::string("/iris_commander/iris/waypoint"));
  ros::param::param("~topic_service_path", topic_service_path, std::string("/iris_commander/iris/path"));
  ros::param::param("~topic_service_path2", topic_service_path2, std::string("/iris_commander/iris/path2"));
  ros::param::param("~topic_service_command_status", topic_command_status, std::string("/iris_commander/iris/command_status"));

  sub_pose = nh_.subscribe(topic_pose, 10, &drone_communicator::callbackPose, this);

  clientExecuteRotation = nh_private_.serviceClient<victim_localization::rotate_action>(topic_service_rotate);
  clientExecuteWaypoint = nh_private_.serviceClient<victim_localization::waypoint_action>(topic_service_waypoint);
  clientExecutePath = nh_private_.serviceClient<victim_localization::path_action>(topic_service_path);
  clientExecutePath2 = nh_private_.serviceClient<victim_localization::path2_action>(topic_service_path2);
  ClientCheckStatus = nh_private_.serviceClient<victim_localization::status_action>(topic_command_status);
  Check_MapManager_and_Drone_Ready();

  visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/Path_points"));
  visualTools->loadMarkerPub();
  visualTools->deleteAllMarkers();
  visualTools->enableBatchPublishing();
}

void drone_communicator::callbackPose(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  current_pose = msg->pose;
}

void drone_communicator::callbackCommandStatus(const std_msgs::Bool::ConstPtr& msg)
{
  current_drone_status = msg->data;
}
void drone_communicator::Check_MapManager_and_Drone_Ready() {

  // if map manager is ready, peform rotation to capture enviroment pointdata and place it in the octomap,
  std::stringstream ss;
  ss << "Start Rotating";
  rotate_srv.request.req= ss.str();
  ros::Rate loop_rate(20);
  while (ros::ok()){
    if (manager_->getMapSize().norm() > 0.0)
    {
      if (clientExecuteRotation.call(rotate_srv))
      {
        break;
    }
  }
    ros::spinOnce();
    loop_rate.sleep();
  }
}


geometry_msgs::Pose drone_communicator::GetPose()
{
  return current_pose;
}

bool drone_communicator::Execute_waypoint(geometry_msgs::Pose p)
{
  current_drone_status=false;
  waypoint_srv.request.waypoint = p;
  if (clientExecuteWaypoint.call(waypoint_srv))
    return true;

  return false;
}

bool drone_communicator::Execute_path(nav_msgs::Path path)
{
  current_drone_status=false;
  path_srv.request.path = path;
  if (clientExecutePath.call(path_srv))
    return true;

  return false;
}

bool drone_communicator::Execute_path(std::vector<geometry_msgs::Pose> path)
{
  current_drone_status=false;
  path2_srv.request.path = path;
  if (clientExecutePath2.call(path2_srv))
    {
      // visualize the path...
      visualTools->publishPath(path,rviz_visual_tools::GREY,rviz_visual_tools::XLARGE,"PATH_");
      visualTools->trigger();
      return true;
    }
  return false;
}

bool drone_communicator::GetStatus()
{
  if(ros::Time::now() - previous_time < ros::Duration(1)) return false;
    previous_time= ros::Time::now();
  std::stringstream ss;
  ss << "Check Status";
  status_srv.request.req= ss.str();
  if (!ClientCheckStatus.call(status_srv)) std::cout<< "status can not be checked...\n";
  if (status_srv.response.resp == true) {
    visualTools->deleteAllMarkers();
    std::cout << "path visualization is resetted..." << std::endl;
    return true;
  }
  return false;
}

