#include "victim_localization/view_generator_ig_nn_adaptive.h"

view_generator_ig_nn_adaptive::view_generator_ig_nn_adaptive():
view_generator_IG(),
scale_factor_(1),
do_adaptive_generation(true)
{
  ros::param::param<int>("~view_generator_nn_adaptive_local_minima_iterations", minima_iterations_, 5);
  ros::param::param<double>("~view_generator_nn_adaptive_utility_threshold", minima_threshold_, 3.0);
  ros::param::param<double>("~view_generator_nn_adaptive_scale_multiplier", scale_multiplier_, 2.0);
  generator_type=Generator::NN_Generator;
}

bool view_generator_ig_nn_adaptive::isStuckInLocalMinima()
{
  if (minima_iterations_ > nbv_history_->iteration)
    return false;

  // See if the last few moves are repeating
  if (nbv_history_->isRepeatingMotions(4))
    return true;

  // Find max entropy change in the past few iterations
  float utility = nbv_history_->getMaxUtility(minima_iterations_);
  // std::cout << "maximum_entropy_change_is: " << utility << std::endl;
  if (utility < minima_threshold_)
    return true;

  return false;
}

void view_generator_ig_nn_adaptive::generateViews()
{
  if (isStuckInLocalMinima())
  {
    scale_factor_*= scale_multiplier_;
    std::cout << "[ViewGeneratorNNAdaptive]: " << cc.yellow << "Local minima detected. Increasing scale factor to " << scale_factor_ << "\n" << cc.reset;

    if (scale_factor_ > 8)
    {
      std::cout << "[ViewGeneratorNNAdaptive]: " << cc.red << "Warning: Scale factor very large: Resetting" << scale_factor_ << "\n" << cc.reset;
      scale_factor_ = 1;
    }
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
  }

  // for scale_factor>1, generate viewpoint using adaptive grid
  else
  {
      generator_type=Generator::NN_Adaptive_Generator;
      nav_type = 1; // set navigation type as reactive planner for adaptive_nn_view_generator
      view_generator_IG::generateViews(false); // do not sample in the current pose to escape local minimum
  }

  // Return start and end  to normal
    start_x_= backup_start_x_;
    start_y_= backup_start_y_;
    end_x_ = backup_end_x_;
    end_y_=  backup_end_y_;
}

bool view_generator_ig_nn_adaptive::IsPreviouslyCheckedSamples(double i_x,double i_y)
{
  if ((i_x>=(-res_x_/scale_multiplier_)) && (i_x<=(res_x_/scale_multiplier_)))
    if ((i_y>=(-res_y_/scale_multiplier_)) && (i_y<=(res_y_/scale_multiplier_)))
       return true;

  return false;
}

std::string view_generator_ig_nn_adaptive::getMethodName()
{
  return "NN Adaptive";
}
