#include "victim_localization/victim_map_wireless.h"

Victim_Map_Wireless::Victim_Map_Wireless(const ros::NodeHandle &nh,const ros::NodeHandle &nh_private):
  Victim_Map_Base(nh,nh_private)
{
  setlayer_name(wireless_layer_name);

  ros::param::param<std::string>("~map_topic_wireless", map_topic , "victim_map/grid_map_wireless");
  ros::param::param<double>("~map_resol_wireless", map_resol , 1.0);
  ros::param::param<double>("~victim_location_x", victim_loc[0] , -4.331);
  ros::param::param<double>("~victim_location_y", victim_loc[1] , -1.427);
  ros::param::param<double>("~offline_map_resol_wireless", offline_map_resol , 1.0);
  ros::param::param<double>("~detection_threshold", detection_threshold , 0.7); //above what value the wireless sensor is assume to detect.
  ros::param::param<double>("~fov_horizontal_wireless", HFOV_deg , 40.0);
  ros::param::param<double>("~wireless_range_max", max_depth_d , 6.0);
  ros::param::param<double>("~wireless_range_min", min_depth_d , 0.3);
  ros::param::param<double>("~wireless_detection_error", wireless_error , 0.5);

  Generate_offline_wireless_map(wireless_error);

  std::cout <<"done with offlinemap\n";

  // Create grid map
  map.setFrameId("map");
  map.setGeometry(Length(x_arena_max,y_arena_max), map_resol); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           map.getLength().x(), map.getLength().y(),
           map.getSize()(0), map.getSize()(1));

  map.add(layer_name,0.5); // initialize probability in the map to 0.5
  const_=max_depth_d/cos(DEG2RAD(HFOV_deg/2));

  pub_map=nh_.advertise<grid_map_msgs::GridMap>(map_topic, 1, true);
  pub_map_offline=nh_.advertise<grid_map_msgs::GridMap>("offline_", 1, true);
  pub_polygon=nh_.advertise<geometry_msgs::PolygonStamped>(wireless_polygon_topic, 1, true);


  //Values for probability
  ros::param::param<double>("~Prob_D_H_for_wireless", Prob_D_H , 0.8);
  ros::param::param<double>("~Prob_D_Hc_for_wireless", Prob_D_Hc , 0.15);
  ros::param::param<double>("~Prob_Dc_H_for_wireless", Prob_Dc_H , 0.2);
  ros::param::param<double>("~Prob_Dc_Hc_for_wireless", Prob_Dc_Hc , 0.85);

  switch(raytracing_type) {
  case 0:
    if (map_resol> octomap_resol){
      raytracing_ = new Raytracing(map_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    else
    {
      raytracing_ = new Raytracing(octomap_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    break;
  case 1:
    if (map_resol> octomap_resol){
      raytracing_ = new Raytracing2D(map_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    else
    {
      raytracing_ = new Raytracing2D(octomap_resol,HFOV_deg,VFOV_deg,max_depth_d,min_depth_d);
    }
    break;
  }

  victimMapName="victim map Wireless";
  Publish_Offline_Map();
}

void Victim_Map_Wireless::Generate_offline_wireless_map(double error)
{
  // Create grid map
  offline_map.setFrameId("map");
  offline_map.setGeometry(Length(x_arena_max,y_arena_max), offline_map_resol); //(Map is 20 by 20 meter with a resolution of 1m).
  ROS_INFO("Created Map with size %f x %f m (%i x %i cells).",
           offline_map.getLength().x(), offline_map.getLength().y(),
           offline_map.getSize()(0), offline_map.getSize()(1));

  offline_map.add("offline",0.0); // initialize map probability to 0.5

  float sigma_x= error;
  float sigma_y= error;
  float mean_x=victim_loc[0];
  float mean_y=victim_loc[1];

  for (grid_map::GridMapIterator it(offline_map);
       !it.isPastEnd(); ++it) {
    Position position;
    offline_map.getPosition(*it, position);
    offline_map.at("offline", *it) = offline_map.at("offline", *it) + (1/(2*M_PI*sigma_x*sigma_y))*exp(-((pow(position.x()-mean_x,2)/(2*pow(sigma_x,2)))+(pow(position.y()-mean_y,2)/(2*pow(sigma_y,2)))));
  }
}

void Victim_Map_Wireless::Publish_Offline_Map()
{
  double curr_max_prob_=0;
  grid_map::Matrix& data = offline_map["offline"];
  for (GridMapIterator iterator(offline_map); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    if(data(index(0), index(1))> curr_max_prob_)
      curr_max_prob_=data(index(0), index(1));
  }

  offline_map["offline"]=offline_map["offline"]/curr_max_prob_; // normalized over the maximum value

  ros::Time time = ros::Time::now();
  offline_map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(offline_map, message);
  pub_map_offline.publish(message);
  ROS_INFO_THROTTLE(1.0, "Offline map (timestamp %f) published.", message.info.header.stamp.toSec());
  //ros::Rate(0.00001).sleep();
}

void Victim_Map_Wireless::Update()
{
  map_status.victim_found=false; //initialize detection to false
  curr_max_prob=0; //initiate current max probabitity to zero

  wireless_polygon=draw_FOV();

  for (grid_map::PolygonIterator iterator(map, wireless_polygon);
       !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    map.getPosition(index, position);

    // Update victim location based on the offline_map
    float P_prior=map.atPosition(layer_name, position);

    if (offline_map.atPosition("offline",position)> 0.7) // update map with believe that victim is detected
      if (P_prior>0.01)
      {
        double Detec_prob=(Prob_D_H* P_prior)/((Prob_D_H* P_prior)+(Prob_D_Hc* (1-P_prior)));
        map.atPosition(layer_name, position)= Detec_prob;

        //check if the victim is found
        if (Detec_prob>victim_found_prob) {
          map_status.victim_found=true;
          map_status.victim_loc=position;
        }
      }
      else map.atPosition(layer_name, position)=(Prob_D_H* 0.5)/((Prob_D_H* 0.5)+(Prob_D_Hc*0.5));

    if (offline_map.atPosition("offline",position)<= 0.7)  // update map with believe that victim is not detected
      map.atPosition(layer_name, position)=(Prob_Dc_H* P_prior)/((Prob_Dc_H* P_prior)+(Prob_Dc_Hc* (1-P_prior)));
  }


  //fastest way to acces grid map
  grid_map::Matrix& data = map[layer_name];
  for (GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
    const Index index(*iterator);
    if(data(index(0), index(1))> curr_max_prob)
      curr_max_prob=data(index(0), index(1));
  }
  if (pub_map.getNumSubscribers()>0)
    publish_Map();
}







