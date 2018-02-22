#ifndef TERMINATION_CHECK_MAX_PROBABILITY_H
#define TERMINATION_CHECK_MAX_PROBABILITY_H

#include <ros/ros.h>
#include "victim_localization/termination_check_base.h"

class TerminationCheckMaxProbability: public TerminationCheckBase
{
public:
  TerminationCheckMaxProbability();
  bool isTerminated();

protected:
  double max_probablity;
};

#endif // TERMINATION_CHECK_MAX_PROBABILITY_H
