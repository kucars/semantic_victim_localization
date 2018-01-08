#ifndef VIEW_GENERATOR_IG_ADAPTIVE_FRONTIER_H
#define VIEW_GENERATOR_IG_ADAPTIVE_FRONTIER_H

#include <victim_localization/view_generator_ig_nn_adaptive.h>
#include <victim_localization/view_generator_ig_frontier.h>

class view_generator_ig_adaptive_frontier : public view_generator_IG
{
public:
  view_generator_ig_adaptive_frontier();

  view_generator_ig_frontier *frontier_generator;
  view_generator_ig_nn_adaptive *nn_adaptive_generator;

  void generateViews(); //viewpoints is  generated at current pose
  std::string getMethodName();

 //visualize
  void visualize(std::vector<geometry_msgs::Pose> valid_poses, std::vector<geometry_msgs::Pose> invalid_poses, geometry_msgs::Pose selected_pose);


};

#endif // VIEW_GENERATOR_IG_ADAPTIVE_FRONTIER_H
