#include "victim_localization/view_evaluator_ig_exp_max.h"

view_evaluator_ig_exp_max::view_evaluator_ig_exp_max():
    view_evaluator_base() //Call base class constructor
{
    ros::param::param<double>("~view_evaluator_weight_distance", w_dist_, 1.0);
}

double view_evaluator_ig_exp_max::calculateUtiltiy(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
    double IG_MAX = calculateIGMax(p,mapping_module);
    double dist = calculateDistance(p);

    return IG_MAX*exp(-dist*w_dist_);
}

double view_evaluator_ig_exp_max::calculateWirelessUtility(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
    double IG_MAX = calculateWirelessIGMAX(p,mapping_module);
    double dist = calculateDistance(p);

    return IG_MAX*exp(-dist*w_dist_);
}

std::string view_evaluator_ig_exp_max::getMethodName()
{
    return "IG_exp_max";
}

double view_evaluator_ig_exp_max::calculateIGMax(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
    grid_map::GridMap temp_Map;
    double max=0;
    double current_prob=0;
    double IG_view=0;

    mapping_module->raytracing_->Initiate(false);

    temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);

    for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
        Position position;
        Index index=*iterator;
        mapping_module->map.getPosition(index, position);
        if(!temp_Map.isInside(position)) continue;

        if(temp_Map.atPosition("temp", position)==0.5){
            continue;
        }

        if(temp_Map.atPosition("temp", position)==0){
            IG_view+=getCellEntropy(position,mapping_module);
            current_prob=mapping_module->map.at(mapping_module->getlayer_name(),index);

            if (current_prob>max){
                max=current_prob;
            }
        }

        if(temp_Map.atPosition("temp", position)==1){
        current_prob=mapping_module->map.at(mapping_module->getlayer_name(),index);
        if (current_prob!=0.5)
            if (current_prob>max){
                max=current_prob;
            }
        }
    }
    return max*IG_view;
}

double view_evaluator_ig_exp_max::calculateWirelessIGMAX(geometry_msgs::Pose p, Victim_Map_Base *mapping_module)
{
    double IG_view=0;
    double max=0;
    double current_prob;

    Position center(p.position.x,p.position.y);
    double radius = wireless_max_range;

    for (grid_map::CircleIterator iterator(mapping_module->map, center, radius);
         !iterator.isPastEnd(); ++iterator) {
        Position position;
        Index index=*iterator;
        mapping_module->map.getPosition(index, position);
        IG_view+=getCellEntropy(position,mapping_module);

        current_prob=mapping_module->map.at(mapping_module->getlayer_name(),index);
        if (current_prob>max){
            max=current_prob;
        }

    }
    return max*IG_view;
}

