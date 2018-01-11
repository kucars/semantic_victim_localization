#include <victim_localization/victim_map_base.h>

using namespace grid_map;

double range_max_=10;
Victim_Map_Base::Victim_Map_Base(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):
  nh_(nh),
  nh_private_(nh_private)
{
  //default values for configs
  ros::param::param<double>("~fov_horizontal", HFOV_deg , 58);
  ros::param::param<double>("~fov_vertical", VFOV_deg , 45);
  ros::param::param<double>("~depth_range_max", max_depth_d , 5);
  ros::param::param<double>("~depth_range_max", min_depth_d , 0.5);
  ros::param::param<double>("~maximum_arena_width", x_arena_max , 20);
  ros::param::param<double>("~maximum_arena_height", y_arena_max , 20);
  ros::param::param<double>("~octomap_resol", octomap_resol , 0.3);
  ros::param::param<double>("~victim_found_prob", victim_found_prob , 0.9);


  ros::param::param<bool>("~detection_enabled", detection_enabled , false); // added for debugging purpose

  const_=max_depth_d/cos(DEG2RAD(HFOV_deg/2));

  sub_loc = nh_.subscribe("/iris/mavros/local_position/pose", 1, &Victim_Map_Base::callbackdrawFOV, this);
  pub_polygon = nh_.advertise<geometry_msgs::PolygonStamped>("polygon", 1, true);

  //initialize_victim_to_false
  map_status.victim_loc=Position(std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::quiet_NaN());

  map_status.victim_found=false;
  curr_max_prob=0;

  victimMapName="victim Map base";
}

Victim_Map_Base::~Victim_Map_Base(){}


grid_map::Polygon Victim_Map_Base::draw_FOV(){

  grid_map::Polygon polygon_FOV;

  //determine the triangle corner of the FOV
  p1.x= current_loc_.position.x+ max_depth_d*cos(current_yaw_) + const_* cos(current_yaw_ + 4.712);
  p2.x= current_loc_.position.x + max_depth_d*cos(current_yaw_) + const_* cos(current_yaw_ + 1.57);
  p1.y= current_loc_.position.y+ max_depth_d*sin(current_yaw_) + const_* sin(current_yaw_ + 4.712);
  p2.y= current_loc_.position.y + max_depth_d*sin(current_yaw_) + const_* sin(current_yaw_ + 1.57);

  //iterate within the FOV
  polygon_FOV.setFrameId(map.getFrameId());
  polygon_FOV.addVertex(Position(current_loc_.position.x, current_loc_.position.y));
  polygon_FOV.addVertex(Position(p1.x, p1.y));
  polygon_FOV.addVertex(Position(p2.x, p2.y));
  polygon_FOV.addVertex(Position(current_loc_.position.x, current_loc_.position.y));

  return polygon_FOV;
  //geometry_msgs::PolygonStamped message;
  //grid_map::PolygonRosConverter::toMessage(polygon, message);
  //pub_polygon.publish(message);
}

grid_map::Polygon Victim_Map_Base::Update_region(grid_map::GridMap Map, geometry_msgs::Pose corner_){

  grid_map::Polygon rectange_update;

  //determine the rectangle corner of the update region
  Point P1,P2,P3,P4;
  P1.x= corner_.position.x-max_depth_d;   P1.y= corner_.position.y+max_depth_d;
  P2.x= corner_.position.x+max_depth_d;   P2.y= corner_.position.y+max_depth_d;
  P3.x= corner_.position.x+max_depth_d;   P3.y= corner_.position.y-max_depth_d;
  P4.x= corner_.position.x-max_depth_d;   P4.y= corner_.position.y-max_depth_d;

  //iterate within the FOV
  rectange_update.setFrameId(Map.getFrameId());
  rectange_update.addVertex(Position(P1.x, P1.y));
  rectange_update.addVertex(Position(P2.x, P2.y));
  rectange_update.addVertex(Position(P3.x, P3.y));
  rectange_update.addVertex(Position(P4.x, P4.y));
  rectange_update.addVertex(Position(P1.x, P1.y));

  return rectange_update;
  //geometry_msgs::PolygonStamped message;
  //grid_map::PolygonRosConverter::toMessage(polygon, message);
  //pub_polygon.publish(message);
}

void Victim_Map_Base::callbackdrawFOV(const PoseStamped &ps_stamped){
  geometry_msgs::Pose ps=ps_stamped.pose;
  double yaw_=pose_conversion::getYawFromQuaternion(ps.orientation);
  Point p1_corner; // first corner point for triangle
  Point p2_corner; // second corner point for triangle
  //determine the triangle corner of the FOV
  p1_corner.x= ps.position.x+ max_depth_d*cos(yaw_) + const_* cos(yaw_ + 4.712);
  p2_corner.x= ps.position.x + max_depth_d*cos(yaw_) + const_* cos(yaw_ + 1.57);
  p1_corner.y= ps.position.y+ max_depth_d*sin(yaw_) + const_* sin(yaw_ + 4.712);
  p2_corner.y= ps.position.y + max_depth_d*sin(yaw_) + const_* sin(yaw_ + 1.57);

  grid_map::Polygon polygon_FOV;

  //iterate within the FOV
  polygon_FOV.setFrameId(map.getFrameId());
  polygon_FOV.addVertex(Position(ps.position.x, ps.position.y));
  polygon_FOV.addVertex(Position(p1_corner.x, p1_corner.y));
  polygon_FOV.addVertex(Position(p2_corner.x, p2_corner.y));
  polygon_FOV.addVertex(Position(ps.position.x, ps.position.y));


  //publish Field of View
  geometry_msgs::PolygonStamped message;
 grid_map::PolygonRosConverter::toMessage(polygon_FOV, message);
 pub_polygon.publish(message);
}


void Victim_Map_Base::publish_Map(){
  ros::Time time = ros::Time::now();
  map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(map, message);
  pub_map.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.", message.info.header.stamp.toSec());
}


/*  bool valid (Position loc) {
  bool valid;
  Index index;
  map.getIndex(loc,index);
  float boundary_x=x_arena_max/2-0.1;
  float boundary_y=y_arena_max/2-0.1;

  if (((loc[0] >= -boundary_x) && (loc[0] <= boundary_x)) && ((loc[1] >= -boundary_y) && (loc[1]  <= boundary_y))) valid=true; // this is to check for NAN values
   else valid=false;
   return valid;
}
*/
Position Victim_Map_Base::approximate_detect(Position res){
  Position approx_;
  if (int(res[0])==0) if (res[0]>0) approx_[0]=0.5; else approx_[0]=-0.5;
  if (int(res[0])>0) approx_[0]= int(res[0]) + 0.5;
  if (int(res[0])<0) approx_[0]= int(res[0]) - 0.5;

  if (approx_[0] > 10) approx_[0]=9.9;  if (approx_[0] < -10) approx_[0]=-9.9;

  if (int(res[1])==0) if (res[1]>0) approx_[1]=0.5; else approx_[1]=-0.5;
  if (int(res[1])>0) approx_[1]= int(res[1]) + 0.5;
  if (int(res[1])<0) approx_[1]= int(res[1]) - 0.5;

  if (approx_[1] > 10) approx_[1]=9.9;  if (approx_[1] < -10) approx_[1]=-9.9;

  return approx_;
}

std::string Victim_Map_Base::getlayer_name() {
  return layer_name;
}
void Victim_Map_Base::setlayer_name(std::string layer_) {
  layer_name=layer_;
}

void Victim_Map_Base::setCurrentPose(geometry_msgs::Pose ps) {
  current_loc_=ps;
  current_yaw_=pose_conversion::getYawFromQuaternion(current_loc_.orientation);
}

void Victim_Map_Base::setDetectionResult(Status detection_status) {
  detect_victim_loc_=detection_status.victim_loc;
  map.getIndex(detect_victim_loc_,detect_victim_index);
  is_detect_=detection_status.victim_found;
}


Status Victim_Map_Base::getMapResultStatus(){
 return map_status;
}


std::string Victim_Map_Base::VictimMapType(){
  return victimMapName;
}

void Victim_Map_Base::setDroneCommunicator(drone_communicator *drone_comm_){
  drone_comm = drone_comm_;
  raytracing_->setDroneCommunicator(drone_comm);
}
