#include "victim_localization/termination_check_max_iterations.h"

TerminationCheckMaxIterations::TerminationCheckMaxIterations()
{
  ros::param::param("~termination_iterations_max", max_iterations_, 100);
  ros::param::param("~termination_victim_max_probability", max_probablity, 0.9);
}

bool TerminationCheckMaxIterations::isTerminated()
{
  if (nbv_history_->iteration >= max_iterations_)
    return true;

  if (nbv_history_->max_prob >= max_probablity)
  {
    ROS_INFO("victim found at location %f, %f",
             (view_evaluator->mapping_module_->getMapResultStatus().victim_loc)[0]
        ,(view_evaluator->mapping_module_->getMapResultStatus().victim_loc)[1]);
    return true;
  }

  return false;
}
