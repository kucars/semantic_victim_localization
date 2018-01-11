#include "victim_localization/view_generator_ig_adaptive_frontier.h"

view_generator_ig_adaptive_frontier::view_generator_ig_adaptive_frontier():
view_generator_ig_frontier(),
scale_factor_(1)
{
ros::param::param<int>("~view_generator_nn_adaptive_local_minima_iterations", minima_iterations_, 5);
ros::param::param<double>("~view_generator_nn_adaptive_utility_threshold", minima_threshold_, 5.0);
}

void view_generator_ig_adaptive_frontier::generateViews()
{
  if (scale_factor_<7.5)   // Use the adaptive NN
  {
  if (isStuckInLocalMinima())
  {
    scale_factor_*= 1.4;
    std::cout << "[ViewGeneratorNNAdaptive]: " << cc.yellow << "Local minima detected. Increasing scale factor to " << scale_factor_ << "\n" << cc.reset;

    if (scale_factor_ >= 7.5)
    {
      std::cout << "[ViewGeneratorNNAdaptive]: " << cc.red << "Warning: Scale factor very large: " << scale_factor_ << "\n" << cc.reset;
      scale_factor_ = 7.5;
    }
  }
  else
  {
    scale_factor_ = 1;
  }

  // Scale up sampling resolution
  double backup_res_x_ = res_x_;
  double backup_res_y_ = res_y_;
  double backup_res_z_ = res_z_;

  res_x_ *= scale_factor_;
  res_y_ *= scale_factor_;
  res_z_ *= scale_factor_;

  // Call base function
  if (scale_factor_ > 1){
    // Do not generate any viewpoints in current location to escape local minima
    view_generator_IG::generateViews(false);
    nav_type = 1; // set navigation type as reactive planner for adaptive_nn_view_generator
  }
  else
  {
    view_generator_IG::generateViews(true);
    nav_type = 0; // set navigation type as straight line for adaptive_nn_view_generator
  }

  // Return scale to normal
  res_x_ = backup_res_x_;
  res_y_ = backup_res_y_;
  res_z_ = backup_res_z_;
  }

  else {   // Use the Frontier Generator
    view_generator_ig_frontier::FindFrontiers();
  }
}

bool view_generator_ig_adaptive_frontier::isStuckInLocalMinima()
{
  if (minima_iterations_ > nbv_history_->iteration)
    return false;

  // See if the last few moves are repeating
  if (nbv_history_->isRepeatingMotions(4))
    return true;

  // Find max entropy change in the past few iterations
  float utility = nbv_history_->getMaxUtility(minima_iterations_);
   std::cout << "maximum_entropy_change_is: " << utility << std::endl;
  if (utility < minima_threshold_)
    return true;

  return false;
}

std::string view_generator_ig_adaptive_frontier::getMethodName()
{
  return "NN Adaptive Frontier";
}
