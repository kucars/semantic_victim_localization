#ifndef VIEW_GENERATOR_IG_FRONTIER_H
#define VIEW_GENERATOR_IG_FRONTIER_H

#include <victim_localization/view_generator_ig.h>
#include <victim_localization/victim_map_base.h>
#include <costmap_2d/costmap_2d.h>



class view_generator_ig_frontier: public view_generator_IG
{
public:
  costmap_2d::Costmap2D *costmap_;
  grid_map::GridMap *victim_map;
  std::string map_layer;
  double frontier_yaw_res_;
  void setvictimmap(grid_map::GridMap *map,std::string layer_name);
  std::vector<geometry_msgs::Pose> setYawtoViewpoint(geometry_msgs::Pose vp);



  view_generator_ig_frontier();
  bool FindFrontiers(std::vector<geometry_msgs::Pose> &frontiers, std::vector<geometry_msgs::Pose> &noFrontiers);

  bool isFrontier(Index point);
  bool isFreeFrontiers(Index point);
  bool isFrontierReached(Index point);
  double getYawToUnknown(Index point);
  inline void getAdjacentPoints(Index point, Index points[]);
  inline int downleft(Index point);
  inline int downright(Index point);
  inline int down(Index point);
  inline int up(Index point);
  inline int upleft(Index point);
  inline int upright(Index point);
  inline int left(Index point);
  inline int right(Index point);
  inline bool IsValid(int point){
    return (point>=0);
  }

  bool isValid(Index point);
  void clearFrontiers();
  void resetMaps();
  void setupMapData();
  void setCostMapROS(costmap_2d::Costmap2DROS *CostMapROS_);
  std::__cxx11::string getMethodName();

  costmap_2d::Costmap2DROS* costmap_ros_;

  const unsigned char* occupancy_grid_array_;
  boost::shared_array<unsigned int> exploration_trans_array_;
  boost::shared_array<unsigned int> obstacle_trans_array_;
  boost::shared_array<int> frontier_map_array_;
  boost::shared_array<bool> is_goal_array_;

  bool initialized_;
  int previous_goal_;

  std::string name;
  unsigned int map_width_;
  unsigned int map_height_;
  unsigned int num_map_cells_;

  double dist_for_frontier_reached;
  bool use_inflated_obs_;

};

#endif // VIEW_GENERATOR_IG_FRONTIER_H
