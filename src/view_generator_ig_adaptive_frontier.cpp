#include "victim_localization/view_generator_ig_adaptive_frontier.h"

view_generator_ig_adaptive_frontier::view_generator_ig_adaptive_frontier():
view_generator_ig_frontier(),
scale_factor_(1),
adaptive_iteration_(0)
{
generator_type=Generator::NN_Generator;

ros::param::param<int>("~view_generator_nn_adaptive_local_minima_iterations", minima_iterations_, 3);
ros::param::param<double>("~view_generator_nn_adaptive_utility_threshold", minima_threshold_, 5.0);
ros::param::param<double>("~view_generator_nn_adaptive_scale_multiplier", scale_multiplier_, 2.0);
}


void view_generator_ig_adaptive_frontier::generateViews()
{
  if ((scale_factor_< 8.0))   // Use the adaptive NN
  {
    if (isStuckInLocalMinima())
    {
      scale_factor_*= scale_multiplier_;
      std::cout << "[ViewGeneratorNNAdaptive]: " << cc.yellow << "Local minima detected. Increasing scale factor to " << scale_factor_ << "\n" << cc.reset;
    }
    else
    {
      scale_factor_ = 1;
    }

    // Scale up sampling resolution
    double backup_start_x_ = start_x_;
    double backup_start_y_ = start_y_;
    double backup_end_x_ = end_x_;
    double backup_end_y_ = end_y_;

    start_x_ *= scale_factor_;
    start_y_ *= scale_factor_;
    end_x_ *= scale_factor_;
    end_y_ *= scale_factor_;

    // for scale_factor=1, generate viewpoint using NN generator
    if (scale_factor_==1)
    {
      view_generator_IG::generateViews(true);
      nav_type = 0; // set navigation type as straight line for adaptive_nn_view_generator
      generator_type=Generator::NN_Generator;
    adaptive_iteration_=0;
    std::cout << "[ViewGenerator]:" << cc.blue << "Perform NN Generator\n" << cc.reset;
    }

    // for scale_factor>1, generate viewpoint using adaptive grid
    else
    {
      view_generator_IG::generateViews(false); // do not sample in the current pose to escape local minimum
      nav_type = 1; // set navigation type as reactive planner for adaptive_nn_view_generator
      generator_type=Generator::NN_Adaptive_Generator;
      adaptive_iteration_=+1;
      std::cout << "[ViewGenerator]: " << cc.blue << "Perform NN Adaptive Generator\n" << cc.reset;
    }

    // Return start and end  to normal
      start_x_= backup_start_x_;
      start_y_= backup_start_y_;
      end_x_ = backup_end_x_;
      end_y_=  backup_end_y_;
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
