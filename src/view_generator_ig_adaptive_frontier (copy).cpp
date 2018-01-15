#include "victim_localization/view_generator_ig_adaptive_frontier.h"

view_generator_ig_adaptive_frontier::view_generator_ig_adaptive_frontier():
view_generator_IG()
{
  frontier_generator    =  new view_generator_ig_frontier;
  nn_adaptive_generator =  new view_generator_ig_nn_adaptive;
}

void view_generator_ig_adaptive_frontier::generateViews()
{
  if (nn_adaptive_generator->scale_factor_<7.5)
  {
    nn_adaptive_generator->generateViews();
    nav_type= nn_adaptive_generator->nav_type;
  }
  else
  {
    frontier_generator->generateViews();
    nav_type= frontier_generator->nav_type;
  }
}

void view_generator_ig_adaptive_frontier::visualize(std::vector<geometry_msgs::Pose> valid_poses, std::vector<geometry_msgs::Pose> invalid_poses, geometry_msgs::Pose selected_pose)


{
  if (nn_adaptive_generator->scale_factor_<7.5)
    nn_adaptive_generator->visualize(valid_poses,invalid_poses,selected_pose);

  else
    frontier_generator->visualize(valid_poses,invalid_poses,selected_pose);
}

std::string view_generator_ig_adaptive_frontier::getMethodName()
{
  return "NN Adaptive Frontier";
}
