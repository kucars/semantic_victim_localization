#include "victim_localization/view_evaluator_fieldofview.h"

view_evaluator_FOV::view_evaluator_FOV():
    view_evaluator_base() //Call base class constructor
{
    ros::param::param<double>("~view_evaluator_weight_distance", w_dist_, 1.0);
}

view_evaluator_FOV::~view_evaluator_FOV(){}

double view_evaluator_FOV::FOVSize(geometry_msgs::Pose p, Victim_Map_Base *mapping_module){

  grid_map::GridMap temp_Map;

  mapping_module->raytracing_->Initiate(false);

  temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);
  double IG_view=0;
  double IG_view_count=0;

  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;

    if(temp_Map.atPosition("temp", position)==0){
     // IG_view+=mapping_module->map.atposition(mapping_module->getlayer_name(),position);
      IG_view_count+=1;
    }
  }

  //double dist = calculateDistance(p);
  //return IG_view*exp(-dist*w_dist_);
  return IG_view_count;
}



void view_evaluator_FOV::evaluate()
{

// clear all vectors
Info_View_counts_dl.clear();
Info_poses.clear();

if (mapping_module_->Maptype==MAP::WIRELESS)   // if the map is Wireless then use wireless evaluator
{
    std::cout << "Warning Wireless evaluation not implemented" << std::endl;
    return;
}

if (mapping_module_->Maptype==MAP::COMBINED)   // if the map is combined then use combined evaluator
{
    evaluateCombined();
    return;
}

view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf



selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

double FOVcount=0;
double FOVcountMax=0;

for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
{
    geometry_msgs::Pose p = view_gen_->generated_poses[i];
    FOVcount=0;

    FOVcount=0;
    FOVcount=FOVSize(p,mapping_module_);

    if (FOVcount!=0){
        Info_View_counts.push_back(FOVcount);
        Info_poses.push_back(p);
    }

    if (FOVcountMax<FOVcount)
        FOVcount=FOVcountMax;
}


double utility;
double Info_View_utilities_mehtod=0;

for (int i=0; i<Info_View_counts.size(); i++) {
    utility=Info_View_counts[i]/FOVcountMax;

    // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
    if (utility >= Info_View_utilities_mehtod)
    {
        Info_View_utilities_mehtod = utility;
        info_selected_utility_ = getCellEntropy(pose_conversion::convertToGridMapPosition(Info_poses[i]),mapping_module_);
        selected_pose_ = Info_poses[i];
    }
}

// No valid poses found, end
if ( std::isnan(selected_pose_.position.x) )
{
    return;
}

view_gen_->visualizeSelectedpose(selected_pose_);

info_distance_total_ += calculateDistance(selected_pose_);
mapping_module_->raytracing_->Done();
}

void view_evaluator_FOV::evaluateCombined()
{
// clear all vectors
Info_View_counts_dl.clear();
Info_View_counts_thermal.clear();

Info_poses.clear();

view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf


selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

double FOVcount_dl=0;
double FOVcount_thermal=0;

double FOVcountMax_dl=0;
double FOVcountMax_thermal=0;



for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
{
    geometry_msgs::Pose p = view_gen_->generated_poses[i];
    FOVcount_dl=0;
    FOVcount_thermal=0;


    FOVcount_dl=FOVSize(p,mapping_module_->getMapLayer(MAP::DL));
    FOVcount_thermal=FOVSize(p,mapping_module_->getMapLayer(MAP::THERMAL));

    if ((FOVcount_dl!=0) && (FOVcount_thermal!=0) ){
        Info_View_counts_dl.push_back(FOVcountMax_dl);
        Info_View_counts_thermal.push_back(FOVcountMax_thermal);
        Info_poses.push_back(p);
    }

    if (FOVcountMax_dl<FOVcountMax_dl)
        FOVcountMax_dl=FOVcountMax_dl;

    if (FOVcountMax_thermal<FOVcountMax_thermal)
        FOVcountMax_thermal=FOVcountMax_thermal;
}

double utility;
double Info_View_utilities_mehtod=0;

for (int i=0; i<Info_View_counts_dl.size(); i++) {
    utility= (alpha*(Info_View_counts_dl[i]/FOVcountMax_dl)+(beta*(Info_View_counts_thermal[i]/FOVcountMax_thermal)))/(alpha+beta);

    // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
    if (utility > Info_View_utilities_mehtod)
    {
        Info_View_utilities_mehtod = utility;
        info_selected_utility_ = getCellEntropy(pose_conversion::convertToGridMapPosition(Info_poses[i]),mapping_module_);
        selected_pose_ = Info_poses[i];
    }
}

// No valid poses found, end
if ( std::isnan(selected_pose_.position.x) )
{
    return;
}

view_gen_->visualizeSelectedpose(selected_pose_);

info_distance_total_ += calculateDistance(selected_pose_);
mapping_module_->raytracing_->Done();

mapping_module_->getMapLayer(MAP::DL)->raytracing_->Done();
mapping_module_->getMapLayer(MAP::THERMAL)->raytracing_->Done();
mapping_module_->getMapLayer(MAP::WIRELESS)->raytracing_->Done();
}

std::string view_evaluator_FOV::getMethodName()
{
    return "Evaluator_FOV";
}
