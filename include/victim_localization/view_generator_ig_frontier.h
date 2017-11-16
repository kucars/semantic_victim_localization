#ifndef VIEW_GENERATOR_IG_FRONTIER_H
#define VIEW_GENERATOR_IG_FRONTIER_H

#include <victim_localization/view_generator_ig.h>
#include <costmap_2d/costmap_2d.h>



class view_generator_ig_frontier
{
public:
  costmap_2d::Costmap2D costmap_;
  view_generator_ig_frontier();
  bool FindFrontiers(std::vector<geometry_msgs::PoseStamped>
                    &frontiers, std::vector<geometry_msgs::PoseStamped>
                    &noFrontiers);

  bool isFrontier(int point);
  bool isFrontierReached(int point);
  double getYawToUnknown(int point);
  inline bool isValid(int point) ;
  inline void getAdjacentPoints(int point, int points[]);
  inline int downleft(int point);
  inline int downright(int point);
  inline int down(int point);
  inline int up(int point);
  inline int upleft(int point);
  inline int upright(int point);
  inline int left(int point);
  inline int right(int point);
  inline bool isValid(int point){
    return (point>=0);
  }

  void clearFrontiers();
  void resetMaps();
  void setCostMapROS(costmap_2d::Costmap2DROS *CostMapROS_);



  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;

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
