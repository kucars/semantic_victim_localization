#include "victim_localization/view_generator_ig_adaptive_frontier.h"

view_generator_ig_adaptive_frontier::view_generator_ig_adaptive_frontier():
view_generator_ig_frontier(),
scale_factor_(1),
adaptive_iteration_(0)
{
generator_type=Generator::NN_Generator;

ros::param::param<int>("~view_generator_nn_adaptive_local_minima_iterations", minima_iterations_, 3);
ros::param::param<double>("~view_generator_nn_adaptive_utility_threshold", minima_threshold_, 5.0);
ros::param::param<int>("~adaptive_ig_max_iteration", adaptive_ig_max_iteration_, 3);
}

void view_generator_ig_adaptive_frontier::generateViews()
{
  if ((scale_factor_<7.5) && (adaptive_iteration_<adaptive_ig_max_iteration_))   // Use the adaptive NN
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
    generator_type=Generator::NN_Adaptive_Generator;
    nav_type = 1; // set navigation type as reactive planner for adaptive_nn_view_generator
    adaptive_iteration_=+1;
    std::cout << "[ViewGenerator]: " << cc.blue << "Perform NN Adaptive Generator\n" << cc.reset;
  }
  else
  {
    view_generator_IG::generateViews(true);
    generator_type=Generator::NN_Generator;
    nav_type = 0; // set navigation type as straight line for adaptive_nn_view_generator
    adaptive_iteration_=0; //reset adaptive iteration
    std::cout << "[ViewGenerator]: " << cc.red << "Perform NN Adaptive Generator\n" << cc.reset;
  }

  // Return scale to normal
  res_x_ = backup_res_x_;
  res_y_ = backup_res_y_;
  res_z_ = backup_res_z_;
  }

  else {   // Use the Frontier Generator
    std::cout << "[ViewGenerator]: " << cc.green << "Perform Frontier Generator\n" << cc.reset;
    view_generator_ig_frontier::generateViews();
    nav_type = 1; // set navigation type as reactive planner for adaptive_nn_view_generator
    generator_type=Generator::Frontier_Generator;
  }

  float utility = nbv_history_->getMaxUtility(minima_iterations_);
   std::cout << "maximum_entropy_change_is: " << utility << std::endl;
   std::cout << "minimum thershold is : " << minima_threshold_ << std::endl;
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
   std::cout << "maximum_entropy_change_is After minimum Iter: " << utility << std::endl;
  if (utility < minima_threshold_)
    return true;

  return false;
}

std::string view_generator_ig_adaptive_frontier::getMethodName()
{
  return "NN Adaptive Frontier";
}
