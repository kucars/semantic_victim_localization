#include "victim_localization/termination_check_max_probability.h"

TerminationCheckMaxProbability::TerminationCheckMaxProbability()
{
  ros::param::param("~termination_victim_max_probability", max_probablity, 0.9);
}

bool TerminationCheckMaxProbability::isTerminated()
{
  if (nbv_history_->max_prob >= max_probablity)
  {
    ROS_INFO("victim found at location %f, %f",
             (view_evaluator->mapping_module_->getMapResultStatus().victim_loc)[0]
        ,(view_evaluator->mapping_module_->getMapResultStatus().victim_loc)[1]);
    return true;
  }

  return false;
}
