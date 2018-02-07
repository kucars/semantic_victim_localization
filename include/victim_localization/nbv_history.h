#ifndef NBV_HISTORY_H
#define NBV_HISTORY_H

#include <victim_localization/common.h>

class nbv_history
{
public:
  nbv_history();
  void computeEntropyDiff();
  double getMaxUtility (int N_iterations);
  bool isRepeatingMotions(int window_size);
  void update();

  int iteration;
   std::vector<float> entropy_diff;
   std::vector<geometry_msgs::Pose> selected_poses;
  std::vector<float> selected_utility;
   std::vector<float> total_entropy;
    std::vector<float> time_per_iteration;

};

#endif // NBV_HISTORY_H
