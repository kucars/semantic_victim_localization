#include "victim_localization/raytracing_2d.h"
#include <stdio.h>

Raytracing2D::Raytracing2D(double map_res_):
  rebuild_octomap_once(false),
  octomap_height(1.0),
  Raytracing(map_res_)
{
  map_.add("occlusion");

  occupied_nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "octomap_occupied_", 1, true);
  free_nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "octomap_free_", 1, true);
  pub_temp_map1=nh_.advertise<grid_map_msgs::GridMap>("temp_map1", 1, true);

}

Raytracing2D::Raytracing2D(double map_res_, double HFOV_deg, double VFOV_deg,
                           double max_d, double min_d):
  rebuild_octomap_once(false),
  octomap_height(1.0),
  Raytracing(map_res_,HFOV_deg,VFOV_deg,max_d,min_d)
{
  map_.add("occlusion");
  occupied_nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "octomap_occupied_", 1, true);
  free_nodes_pub_ = nh_.advertise<visualization_msgs::MarkerArray>(
        "octomap_free_", 1, true);
  pub_temp_map1=nh_.advertise<grid_map_msgs::GridMap>("temp_map1", 1, true);
}

Raytracing2D::~Raytracing2D(){}

void Raytracing2D::GenerateOctomap(bool rebuild_once, bool publish){
  if (rebuild_octomap_once){
    if(publish){
      if (occupied_nodes_pub_.getNumSubscribers()>0)
        Publish2DOctomap();
    }
    return;
  }
  // get parameters for the 2D octomap

  ros::param::param("~sensor_max_range", sensor_max_range,5.0);
  ros::param::param("~visualize_max_z", visualize_max_z,std::numeric_limits<double>::max());
  ros::param::param("~visualize_min_z", visualize_min_z,-std::numeric_limits<double>::max());
  ros::param::param("~treat_unknown_as_occupied", treat_unknown_as_occupied,true);
  ros::param::param("~map_publish_frequency", map_publish_frequency,1.0);



  octree_2d.reset(new octomap::OcTree(grid_->info.resolution));


  if (octree_2d->getResolution() != drone_comm->manager_->getResolution())
    std::cout << "octree_2d has different resolution: " << octree_2d->getResolution()
             << " ," << drone_comm->manager_->getResolution() << std::endl;

  // convert occupancy grid to Grid Map type for easy data maniputation
  if(!GridMapRosConverter::fromOccupancyGrid(*grid_,"occlusion",map_))
    ROS_INFO("Unable to convert occupancy grid to grid Map");

  // fill octomap map with the grid map data
  Position pos_;
  Index index;
  octomap::KeySet free_cells, occupied_cells;

  for (grid_map::GridMapIterator iterator(map_);
       !iterator.isPastEnd(); ++iterator) {
    index=*iterator;
    map_.getPosition(index, pos_);

    const octomap::point3d p_point(pos_[0], pos_[1], octomap_height);
    octomap::OcTreeKey key = octree_2d->coordToKey(p_point);


    // Check if this is within the allowed sensor range.
    if (!(occupied_cells.find(key) == occupied_cells.end())) continue;

    if (map_.at("occlusion",index)==0){
      free_cells.insert(key);
    }

    if (map_.at("occlusion",index)==100){
      occupied_cells.insert(key);
    }
  }
  updateOctomapOccupancy(&free_cells, &occupied_cells);

  if(publish)
    if (occupied_nodes_pub_.getNumSubscribers()>0)
      Publish2DOctomap();

  if (rebuild_once)
    rebuild_octomap_once=true;
}

void Raytracing2D::updateOctomapOccupancy(octomap::KeySet* free_cells,
                                          octomap::KeySet* occupied_cells) {
  CHECK_NOTNULL(free_cells);
  CHECK_NOTNULL(occupied_cells);

  bool lazy_eval=true;
  double log_odds_value_min= octree_2d->getClampingThresMinLog();
  double log_odds_value_max= octree_2d->getClampingThresMaxLog();


  // Mark occupied cells.
  for (octomap::KeySet::iterator it = occupied_cells->begin(),
       end = occupied_cells->end();
       it != end; it++) {
    octree_2d->setNodeValue(*it, log_odds_value_max, lazy_eval);

    //octomap::OcTreeNode* node = octree_2d->search(*it) ;
    //std::cout << "the occupied nodes has loc: " << octree_2d->keyToCoord(*it)  << " Occup: " << node->getOccupancy() << " Value: " << node->getValue();

    // Remove any occupied cells from free cells
    if (free_cells->find(*it) != free_cells->end()) {
      free_cells->erase(*it);
    }
  }

  // Mark free cells.
  for (octomap::KeySet::iterator it = free_cells->begin(),
       end = free_cells->end();
       it != end; ++it) {
    octree_2d->setNodeValue(*it, log_odds_value_min, lazy_eval);

    //octomap::OcTreeNode* node = octree_2d->search(*it) ;
    //std::cout << "the free nodes has loc: " << octree_2d->keyToCoord(*it)  << " Occup: " << node->getOccupancy() << " Value: " << node->getValue();
  }
  octree_2d->updateInnerOccupancy();
}


void Raytracing2D::update()
{
  current_pose_= drone_comm->GetPose();
  tree_=octree_2d;
  compute2DRelativeRays();
}

double Raytracing2D::compute2DRelativeRays()
{
  rays_far_plane_.clear();
  double deg2rad = M_PI/180;
  double min_x = -range_max_ * tan(HFOV_deg/2 * deg2rad);
  double max_x =  range_max_ * tan(HFOV_deg/2 * deg2rad);

  double x_step = drone_comm->manager_->getResolution();

  for( double x = min_x; x<=max_x; x+=x_step )
  {
    Eigen::Vector3d p_far(range_max_, x, 0);
    rays_far_plane_.push_back(p_far);
  }
}

void Raytracing2D::compute2DRaysAtPose(geometry_msgs::Pose p)
{
  rays_far_plane_at_pose_.clear();

  Eigen::Matrix3d r_pose, rotation_mtx_;

  // in 2D Raytracing, both the roll and pitch are set to 0, only yaw is considered.

  r_pose = pose_conversion::getRotationMatrixfromYaw(p);

  rotation_mtx_ = r_pose;

  for (int r=0; r<rays_far_plane_.size(); r++)
  {
    // Rotate ray to its final position
    Eigen::Vector3d temp = rotation_mtx_*rays_far_plane_[r];

    // Create an octomap point to later cast a ray
    octomap::point3d p_ (temp[0], temp[1], temp[2]);

    //std::cout << 'roll, pitch and yaw are...' << p_.roll()*(180/M_PI) << " " << p_.pitch()*(180/M_PI) << " " << p_.yaw()*(180/M_PI) << "\n";

    //std::cout << "for pose: "<< p << " finalpoints: "<< p_ <<std::endl;
    //if (temp[2]>(current_pose_.position.z -0.5) && temp[2]<(current_pose_.position.z +0.5))
     rays_far_plane_at_pose_.push_back(p_);
 }
 }

grid_map::GridMap Raytracing2D::Generate_2D_Safe_Plane(geometry_msgs::Pose p_, bool publish_)
{
  std::cout << " this is 2d ray tracing\n";

  if (publish_ray_) visualTools->deleteAllMarkers(); // clear the previous rays

  update();
  compute2DRaysAtPose(p_);
  grid_map::GridMap Pose_map ;
  Pose_map.setFrameId("map");
  Pose_map.setGeometry(Length(range_max_*2.5,range_max_*2.5),map_res,Position (p_.position.x,p_.position.y));
  Pose_map.add({"temp"},0.5);  //0 for free, 0.5 for unexplored,

  // set the height to the same height of the 2d octomap
  octomap::point3d origin (p_.position.x, p_.position.y, octomap_height);


  std::set<octomap::OcTreeKey, OctomapKeyCompare> nodes; //all nodes in a set are UNIQUE

  for (int i=0; i<rays_far_plane_at_pose_.size(); i++)
  {
    octomap::point3d endpoint;

    // Get length of beam to the far plane of sensor
    double range = rays_far_plane_at_pose_[i].norm();
    // Get the direction of the ray
    octomap::point3d dir = rays_far_plane_at_pose_[i].normalize();

    // Cast through unknown cells as well as free cells
    bool found_endpoint = tree_->castRay(origin, dir, endpoint, true, range);
    if (!found_endpoint)
    {
      endpoint = origin + dir * range;
    }

    if( found_endpoint ) {
      /* Check ray
   *
   * We first draw a ray from the UAV to the endpoint
   *
   * For generality, we assume the UAV is outside the object bounds. We only consider nodes
   * 	that are inside the object bounds, and past the near-plane of the camera.
   *
   * The ray continues until the end point.
   * If the ray exits the bounds, we stop adding nodes to IG and discard the endpoint.
   *
   */

      // std::cout << "NEWRAY" << origin << std::endl;

      octomap::KeyRay ray;
      tree_->computeRayKeys( origin, endpoint, ray );

      for( octomap::KeyRay::iterator it = ray.begin() ; it!=ray.end(); ++it )
      {
        octomap::point3d p = tree_->keyToCoord(*it);

        //std::cout << "this is the point from the ray: " << p << std::endl;

        // project the point to the Map
        octomap::OcTreeNode* node = tree_->search(*it);
        Position position(p.x(),p.y());

     //   if (!isInsideRegionofInterest(p.z())) continue;

        if (!isInsideBounds(position)) continue;

        double prob = getNodeOccupancy(node);
        if (prob > 0.8) Pose_map.atPosition("temp",position)=1;

        if (prob < 0.2)
          if (Pose_map.atPosition("temp",position)!=1) Pose_map.atPosition("temp",position)=0;
      }
    }

    octomap::OcTreeKey end_key;
    if (tree_->coordToKeyChecked(endpoint,end_key))
    {
      octomap::OcTreeNode* node = tree_->search(end_key);
      octomap::point3d p = tree_->keyToCoord(end_key);
      Position position(p.x(),p.y());

     // if (!isInsideRegionofInterest(p.z())) continue;

      if (!isInsideBounds(position)) continue;

      double prob = getNodeOccupancy(node);
      if (prob > 0.8) Pose_map.atPosition("temp",position)=1;

      if (prob < 0.2)
        if (Pose_map.atPosition("temp",position)!=1) Pose_map.atPosition("temp",position)=0;
    }

    // visualize the ray
    if (publish_ray_){
    geometry_msgs::Pose  end_;
    end_.position.x=endpoint.x();
    end_.position.y=endpoint.y();
    end_.position.z=endpoint.z();
    visualTools->publishLine(p_.position,end_.position,rviz_visual_tools::PINK,rviz_visual_tools::LARGE);
    }
  }

if (publish_ray_) visualTools->trigger();


  if (publish_)
    publish_Map(Pose_map);
  publish_Map2(map_);

  return Pose_map;
}


void Raytracing2D::publish_Map2(GridMap Map)
{
  ros::Time time = ros::Time::now();
  Map.setTimestamp(time.toNSec());
  grid_map_msgs::GridMap message;
  GridMapRosConverter::toMessage(Map, message);
  pub_temp_map1.publish(message);
  ROS_INFO_THROTTLE(1.0, "Grid map test (timestamp %f) published.", message.info.header.stamp.toSec());

}


void Raytracing2D::Publish2DOctomap()
{
  visualization_msgs::MarkerArray occupied_nodes, free_nodes;
  generateMarkerArray("map", &occupied_nodes, &free_nodes);
  occupied_nodes_pub_.publish(occupied_nodes);
  free_nodes_pub_.publish(free_nodes);
}

void Raytracing2D::ResetOctomapRebuild()
{
  rebuild_octomap_once=false;
}

void Raytracing2D::Initiate(bool rebuild_once, bool publish)
{
   // rebuild_once is by default set to false...

  // if rebuild_once is explicitly set to true then reset 2d_octomap
  // if rebuild_once is false, check it is value from config file.

  if (rebuild_once)
    GenerateOctomap(rebuild_once,publish);

  else{
    ros::param::param<bool>("~rebuild_octomap_for_2D_Raytracing", rebuild_octomap_, false);
    GenerateOctomap(rebuild_octomap_,publish);
  }

}

void Raytracing2D::Done()
{
  ResetOctomapRebuild();
}


//*********
// The following portion is borrowed from octomap_world for visualizing the octomap
//*********

void Raytracing2D::generateMarkerArray(
    const std::string& tf_frame,
    visualization_msgs::MarkerArray* occupied_nodes,
    visualization_msgs::MarkerArray* free_nodes) {
  CHECK_NOTNULL(occupied_nodes);
  CHECK_NOTNULL(free_nodes);

  // Prune the octree first.
  octree_2d->prune();
  int tree_depth = octree_2d->getTreeDepth() + 1;

  // In the marker array, assign each node to its respective depth level, since
  // all markers in a CUBE_LIST must have the same scale.
  occupied_nodes->markers.resize(tree_depth);
  free_nodes->markers.resize(tree_depth);

  // Metric min and max z of the map:
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree_2d->getMetricMin(min_x, min_y, min_z);
  octree_2d->getMetricMax(max_x, max_y, max_z);

  // Update values from params if necessary.
  if (visualize_min_z > min_z) {
    min_z = visualize_min_z;
  }
  if (visualize_max_z < max_z) {
    max_z = visualize_max_z;
  }

  for (int i = 0; i < tree_depth; ++i) {
    double size = octree_2d->getNodeSize(i);

    occupied_nodes->markers[i].header.frame_id = tf_frame;
    occupied_nodes->markers[i].ns = "map";
    occupied_nodes->markers[i].id = i;
    occupied_nodes->markers[i].type = visualization_msgs::Marker::CUBE_LIST;
    occupied_nodes->markers[i].scale.x = size;
    occupied_nodes->markers[i].scale.y = size;
    occupied_nodes->markers[i].scale.z = size;

    free_nodes->markers[i] = occupied_nodes->markers[i];
  }

  for (octomap::OcTree::leaf_iterator it = octree_2d->begin_leafs(),
       end = octree_2d->end_leafs();
       it != end; ++it) {
    geometry_msgs::Point cube_center;
    cube_center.x = it.getX();
    cube_center.y = it.getY();
    cube_center.z = it.getZ();

    if (cube_center.z > max_z || cube_center.z < min_z) {
      continue;
    }

    int depth_level = it.getDepth();

    if (octree_2d->isNodeOccupied(*it)) {
      occupied_nodes->markers[depth_level].points.push_back(cube_center);
      occupied_nodes->markers[depth_level].colors.push_back(
            percentToColor(colorizeMapByHeight(it.getZ(), min_z, max_z)));
    } else {
      free_nodes->markers[depth_level].points.push_back(cube_center);
      free_nodes->markers[depth_level].colors.push_back(
            percentToColor(colorizeMapByHeight(it.getZ(), min_z, max_z)));
    }
  }

  for (int i = 0; i < tree_depth; ++i) {
    if (occupied_nodes->markers[i].points.size() > 0) {
      occupied_nodes->markers[i].action = visualization_msgs::Marker::ADD;
    } else {
      occupied_nodes->markers[i].action = visualization_msgs::Marker::DELETE;
    }

    if (free_nodes->markers[i].points.size() > 0) {
      free_nodes->markers[i].action = visualization_msgs::Marker::ADD;
    } else {
      free_nodes->markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
}

std_msgs::ColorRGBA Raytracing2D::percentToColor(double h) const {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
  // blend over HSV-values (more colors)

  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1)) f = 1 - f;  // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i) {
  case 6:
  case 0:
    color.r = v;
    color.g = n;
    color.b = m;
    break;
  case 1:
    color.r = n;
    color.g = v;
    color.b = m;
    break;
  case 2:
    color.r = m;
    color.g = v;
    color.b = n;
    break;
  case 3:
    color.r = m;
    color.g = n;
    color.b = v;
    break;
  case 4:
    color.r = n;
    color.g = m;
    color.b = v;
    break;
  case 5:
    color.r = v;
    color.g = m;
    color.b = n;
    break;
  default:
    color.r = 1;
    color.g = 0.5;
    color.b = 0.5;
    break;
  }

  return color;
}
