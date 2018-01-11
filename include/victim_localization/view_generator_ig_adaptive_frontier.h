#ifndef VIEW_GENERATOR_IG_ADAPTIVE_FRONTIER_H
#define VIEW_GENERATOR_IG_ADAPTIVE_FRONTIER_H

#include <victim_localization/view_generator_ig_frontier.h>

class view_generator_ig_adaptive_frontier : public view_generator_ig_frontier
{
public:
  view_generator_ig_adaptive_frontier();

  void generateViews(); //viewpoints is  generated at current pose
  std::string getMethodName();

 //visualize
  double scale_factor_;

private:
  int minima_iterations_;
  double minima_threshold_;
  bool isStuckInLocalMinima();

};

#endif // VIEW_GENERATOR_IG_ADAPTIVE_FRONTIER_H
