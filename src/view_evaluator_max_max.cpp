#include <victim_localization/view_evaluator_max_max.h>

view_evaluator_MaxMax::view_evaluator_MaxMax():
  view_evaluator_base()
{
  max_belief=0;
  ros::param::param<double>("~distance_threshold", wireless_max_range , 7.0);
}


double view_evaluator_MaxMax::GetMAXANDCOUNT(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &max, double &max_count)
{
  grid_map::GridMap temp_Map;

  mapping_module->raytracing_->Initiate(false);

  temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);
  double view_max=0;
  double view_max_absolute=0;

  double view_max_count=0;

  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);
    if(!temp_Map.isInside(position)) continue;


    if(temp_Map.atPosition("temp", position)==0){
      view_max=mapping_module->map.at(mapping_module->getlayer_name(),index);
    }

    if (view_max_absolute<view_max)
    {
       view_max_count=0;
       view_max_absolute=view_max;
    }

    if (view_max_absolute==view_max)
        view_max_count=view_max_count+1;
  }

   max=view_max_absolute;
   max_count=view_max_count;
}


double view_evaluator_MaxMax::GetMAXANDCOUNTWIRELESS(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &max, double &max_count)
{
    double view_max=0;
    double view_max_absolute=0;

    double view_max_count=0;

    Position center(p.position.x,p.position.y);
    double radius = wireless_max_range;

    for (grid_map::CircleIterator iterator(mapping_module->map, center, radius);
         !iterator.isPastEnd(); ++iterator) {

        Position position;
        Index index=*iterator;
        mapping_module->map.getPosition(index, position);

        view_max=mapping_module->map.at(mapping_module->getlayer_name(),index);

        if (view_max_absolute<view_max)
        {
           view_max_count=0;
           view_max_absolute=view_max;
        }

        if (view_max_absolute==view_max)
            view_max_count++;

}
       max=view_max_absolute;
       max_count=view_max_count;
}



double view_evaluator_MaxMax::GetMAXANDCOUNCombined(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &max, double &max_count)
{
  grid_map::GridMap temp_Map_dl;
  grid_map::GridMap temp_Map_thermal;


  mapping_module->getMapLayer(MAP::DL)->raytracing_->Initiate(false);
  mapping_module->getMapLayer(MAP::THERMAL)->raytracing_->Initiate(false);

  temp_Map_dl=mapping_module->getMapLayer(MAP::DL)->raytracing_->Generate_2D_Safe_Plane(p,true,true);
  temp_Map_thermal=mapping_module->getMapLayer(MAP::THERMAL)->raytracing_->Generate_2D_Safe_Plane(p,true,true);

  double view_max_dl=0;
  double view_max_absolute_dl=0;
  double view_max_count_dl=0;
  double view_max_thermal=0;
  double view_max_absolute_thermal=0;
  double view_max_count_thermal=0;
  double view_max_wireless=0;
  double view_max_absolute_wireless=0;
  double view_max_count_wireless=0;



  // for deep learning
  for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);

    if(!temp_Map_dl.isInside(position)) continue;

    if(temp_Map_dl.atPosition("temp", position)==0){
     view_max_dl=mapping_module->map.at(mapping_module->getlayer_name(),index);
    }

    if (view_max_absolute_dl<view_max_dl)
    {
       view_max_count_dl=0;
       view_max_absolute_dl=view_max_dl;
    }

    if (view_max_absolute_dl==view_max_dl)
        view_max_count_dl++;
  }


// for thermal
for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
  Position position;
  Index index=*iterator;
  mapping_module->map.getPosition(index, position);

  if(!temp_Map_thermal.isInside(position)) continue;

  if(temp_Map_thermal.atPosition("temp", position)==0){
   view_max_thermal=mapping_module->map.at(mapping_module->getlayer_name(),index);
  }

  if (view_max_absolute_thermal<view_max_thermal)
  {
     view_max_count_thermal=0;
     view_max_absolute_thermal=view_max_thermal;
  }

  if (view_max_absolute_thermal==view_max_thermal)
      view_max_count_thermal++;
}


//wireless
Position center(p.position.x,p.position.y);
double radius = wireless_max_range;

for (grid_map::CircleIterator iterator(mapping_module->map, center, radius);
     !iterator.isPastEnd(); ++iterator) {

    Position position;
    Index index=*iterator;
    mapping_module->map.getPosition(index, position);

    view_max_wireless=mapping_module->map.at(mapping_module->getlayer_name(),index);

    if (view_max_absolute_wireless<view_max_wireless)
    {
       view_max_count_wireless=0;
       view_max_absolute_wireless=view_max_wireless;
    }

    if (view_max_absolute_wireless==view_max_wireless)
        view_max_count_wireless++;

}




   if (view_max_absolute_dl>=view_max_absolute_thermal)
       if (view_max_absolute_dl>=view_max_absolute_wireless){
           max=view_max_absolute_dl;}
        else {
           max=view_max_absolute_wireless;}
   else
       if (view_max_absolute_thermal>=view_max_absolute_wireless){
                  max=view_max_absolute_thermal;}
               else {
                  max=view_max_absolute_wireless;}


   if (view_max_count_dl>=view_max_count_thermal)
       if (view_max_count_dl>=view_max_count_wireless){
           max_count=view_max_count_dl;}
        else {
           max_count=view_max_count_wireless;}
   else
       if (view_max_count_thermal>=view_max_count_wireless){
                  max_count=view_max_count_thermal;}
               else {
                  max_count=view_max_count_wireless;}


}

void view_evaluator_MaxMax::evaluate()
{

// clear all vectors
Info_View_max.clear();
Info_View_max_count.clear();
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

double MAX_=0;
double absolute_max=0;
double MAX_COUNT=0;

for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
{
    MAX_=0;
    MAX_COUNT=0;
    geometry_msgs::Pose p=view_gen_->generated_poses[i];

    GetMAXANDCOUNT(p,mapping_module_,MAX_,MAX_COUNT);

    if (MAX_!=0)
    {
        Info_View_max.push_back(MAX_);
        Info_View_max_count.push_back(MAX_COUNT);
        Info_poses.push_back(p);
    }

    if (absolute_max<MAX_)
        absolute_max=MAX_;
}

double absolute_max_count=0;
for (int i=0; i<Info_View_max.size() && ros::ok(); i++)
{
    if (Info_View_max[i]==absolute_max)
        if (absolute_max_count<=Info_View_max_count[i])
        {
            selected_pose_=Info_poses[i];
            info_selected_utility_ = getCellEntropy(pose_conversion::convertToGridMapPosition(Info_poses[i]),mapping_module_);
             absolute_max_count = Info_View_max_count[i];
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



void view_evaluator_MaxMax::evaluateWireless()
{
  view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

  // clear all vectors
  Info_View_max.clear();
  Info_View_max_count.clear();

  Info_poses.clear();
  info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
  info_selected_direction_=0;
  info_utilities_.clear();
  Info_WirelessDiection.clear();


  selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

  double MAX_=0;
  double absolute_max=0;
  double MAX_COUNT=0;


   for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
      geometry_msgs::Pose p = view_gen_->generated_poses[i];

      MAX_=0;
      MAX_COUNT=0;

      GetMAXANDCOUNT(p,mapping_module_,MAX_,MAX_COUNT);

      double utility_direction = calculateIG(p,mapping_module_);
      double utility;
      if (i==0){
          GetMAXANDCOUNT(p,mapping_module_,MAX_,MAX_COUNT);
      }
      if (i!=0) {
        if (!IsSamePosition(view_gen_->generated_poses[i],view_gen_->generated_poses[i-1]))
        {
         GetMAXANDCOUNT(p,mapping_module_,MAX_,MAX_COUNT);
        }
        else
            MAX_=0;
            MAX_COUNT=0;
      }

      if (MAX_!=0){
          Info_View_max.push_back(MAX_);
          Info_View_max_count.push_back(MAX_COUNT);
          Info_WirelessDiection.push_back(utility_direction);
          Info_poses.push_back(p);
      }

      if (absolute_max<MAX_)
          absolute_max=MAX_;

   }

   double info_selected_method=0;
   double absolute_max_count=0;

   for (int i=0; i< Info_View_max.size(); i++)
   {
      if (Info_View_max[i]==absolute_max)
           if (absolute_max_count<=Info_View_max_count[i])
           {
               info_selected_method=absolute_max;
               info_selected_utility_ = getCellEntropy(pose_conversion::convertToGridMapPosition(Info_poses[i]),mapping_module_);
               absolute_max_count = Info_View_max_count[i];

               if (Info_WirelessDiection[i]>info_selected_direction_)
               {
                   info_selected_direction_ = Info_WirelessDiection[i];
                   selected_pose_ = Info_poses[i];
               }
           }

       }

       // No valid poses found, end
       if ( std::isnan(selected_pose_.position.x) )
       {
         return;
       }

       view_gen_->visualizeSelectedpose(selected_pose_);
      //view_gen_->visualize(view_gen_->generated_poses, view_gen_->rejected_poses,selected_pose_);

 info_distance_total_ += calculateDistance(selected_pose_);
 mapping_module_->raytracing_->Done();
  }

void view_evaluator_MaxMax::evaluateCombined()
{

// clear all vectors
Info_View_max.clear();
Info_View_max_count.clear();
Info_poses.clear();

view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf

selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

double MAX_=0;
double absolute_max=0;
double MAX_COUNT=0;

for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
{
    MAX_=0;
    MAX_COUNT=0;
    geometry_msgs::Pose p=view_gen_->generated_poses[i];

    GetMAXANDCOUNCombined(p,mapping_module_,MAX_,MAX_COUNT);

    if (MAX_!=0){
        Info_View_max.push_back(MAX_);
        Info_View_max_count.push_back(MAX_COUNT);
        Info_poses.push_back(p);
    }

    if (absolute_max<MAX_)
        absolute_max=MAX_;
}

 double absolute_max_count=0;
for (int i=0; i<Info_View_max.size() && ros::ok(); i++)
{
    if (Info_View_max[i]==absolute_max)
        if (absolute_max_count<=Info_View_max_count[i])
        {
            selected_pose_=Info_poses[i];
            info_selected_utility_ = getCellEntropy(pose_conversion::convertToGridMapPosition(Info_poses[i]),mapping_module_);
            absolute_max_count = Info_View_max_count[i];
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


std::string view_evaluator_MaxMax::getMethodName()
{
  return "MAXMAX";
}







