#ifndef VIEW_GENERATOR_IG_FRONTIER_H
#define VIEW_GENERATOR_IG_FRONTIER_H

#include <victim_localization/view_generator_ig.h>
#include <victim_localization/victim_map_base.h>
#include <costmap_2d/costmap_2d.h>
#include <math.h>


class view_generator_ig_frontier: public view_generator_IG
{
public:
  double frontier_yaw_res_;
  void setvictimmap(grid_map::GridMap *map,std::string layer_name);
  bool setYawtoViewpoint(geometry_msgs::Pose Frontier, Index index_, std::vector<geometry_msgs::Pose> &Frontier_with_yaws);
  bool IsPointingtoUnkown(double yaw, Index index_);

  view_generator_ig_frontier();
  std::vector<geometry_msgs::Pose> FindFrontiers();

  bool isFrontier(Index point);
  bool isFreeFrontiers(Index point);
  bool isFrontierReached(Index point);
  inline void getAdjacentPoints(Index point, Index points[]);
  inline Index downleft(Index point);
  inline Index downright(Index point);
  inline Index down(Index point);
  inline Index up(Index point);
  inline Index upleft(Index point);
  inline Index upright(Index point);
  inline Index left(Index point);
  inline Index right(Index point);
  inline bool IsValid(int point){
    return (point>=0);
  }

  bool isValid(Index point);
  void clearFrontiers();
  void resetMaps();
  void setupMapData();
  void setCostMapROS(costmap_2d::Costmap2DROS *CostMapROS_);
  std::string getMethodName();

  const unsigned char* occupancy_grid_array_;
  boost::shared_array<unsigned int> exploration_trans_array_;
  boost::shared_array<unsigned int> obstacle_trans_array_;
  boost::shared_array<int> frontier_map_array_;
  boost::shared_array<bool> is_goal_array_;

  bool initialized_;
  int previous_goal_;
  std::vector<geometry_msgs::Pose> Frontier_yaw_rejected_poses;

  std::string name;
  unsigned int map_width_;
  unsigned int map_height_;
  unsigned int num_map_cells_;

  double dist_for_frontier_reached;
  bool use_inflated_obs_;


  virtual void generateViews();

  void Visualize();
  bool checkValidity(int costmap_index);

};

#endif // VIEW_GENERATOR_IG_FRONTIER_H
