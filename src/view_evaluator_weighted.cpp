#include <victim_localization/view_evaluator_weighted.h>

view_evaluator_weighted::view_evaluator_weighted():
    view_evaluator_base()
{
    min_belief=std::numeric_limits<float>::infinity();
    max_belief=-std::numeric_limits<float>::infinity();

    ros::param::param<double>("~exploration_weight",exploration_weight,0.5);
    ros::param::param<double>("~victim_finding_weight",victim_finding_weight,0.5);
    ros::param::param<double>("~distance_weight",dist_weight,0.2);
    ros::param::param<double>("~penality_factor",f_,2);
}

void view_evaluator_weighted::calculateIGwithMax(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &IG, double &Max)
{
    grid_map::GridMap temp_Map;
    double max_view=0;
    double current_prob=0;
    double IG_view=0;

    mapping_module->raytracing_->Initiate(false);

    temp_Map=mapping_module->raytracing_->Generate_2D_Safe_Plane(p,true,true);

    for (grid_map::GridMapIterator iterator(mapping_module->map); !iterator.isPastEnd(); ++iterator) {
        Position position;
        Index index=*iterator;
        current_prob=0;
        mapping_module->map.getPosition(index, position);
        if(!temp_Map.isInside(position)) continue;
         if(temp_Map.atPosition("temp", position)==0.5) continue;

        if(temp_Map.atPosition("temp", position)==0){
            IG_view+=getCellEntropy(position,mapping_module);
            current_prob=mapping_module->map.at(mapping_module->getlayer_name(),index);
            if (current_prob>max_view){
                max_view=current_prob;
            }
        }
        if(temp_Map.atPosition("temp", position)==1){
        current_prob=mapping_module->map.at(mapping_module->getlayer_name(),index);
        if (current_prob!=0.5)
            if (current_prob>max_view){
                max_view=current_prob;
            }
        }
    }
    IG= IG_view;
    Max=max_view;
}

void view_evaluator_weighted::calculateWirelessIGwithMax(geometry_msgs::Pose p, Victim_Map_Base *mapping_module, double &IG, double &Max)
{
    double IG_view=0;
    double max_view=0;
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
        if (current_prob>max_view){
            max_view=current_prob;
        }
    }
    IG= IG_view;
    Max=max_view;
}

void view_evaluator_weighted::evaluate()
{
    // clear all vectors
    Info_View_utilities.clear();
    Info_View_Max.clear();
    Info_poses.clear();
    Info_View_utilities_DL.clear();
    Info_View_utilities_Thermal.clear();
    Info_View_utilities_Wireless.clear();
    Info_View_Max_DL.clear();
    Info_View_Max_Thermal.clear();
    Info_View_Max_Wireless.clear();

    if (mapping_module_->Maptype==MAP::WIRELESS)   // if the map is Wireless then use wireless evaluator
    {
        evaluateWireless();
        return;
    }

    if (mapping_module_->Maptype==MAP::COMBINED)   // if the map is combined then use combined evaluator
    {
        evaluateCombined();
        return;
    }

    view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

    info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
    info_utilities_.clear();



    selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

    double MaxIGinAllView=0;
    double MaxProbinAllView=0;

     double MaxIGindex, MaxProbIndex;

    for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
        geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double ViewIG=0;
        double ViewMax=-std::numeric_limits<float>::infinity();

        calculateIGwithMax(p,mapping_module_,ViewIG,ViewMax);

            Info_View_utilities.push_back(ViewIG);
            Info_View_Max.push_back(ViewMax);
            Info_poses.push_back(p);

            // keep track of max information IG across all views
            if(MaxIGinAllView<ViewIG){
                MaxIGinAllView=ViewIG;
                MaxIGindex=i;
            }
            // keep track of max Prob in All views
            if(MaxProbinAllView<ViewMax){
                MaxProbinAllView=ViewMax;
                 MaxProbIndex=i;
            }


    }

    // Normalized the IG view to 1
    for (int i=0; i<Info_View_utilities.size(); i++)
        Info_View_utilities[i]=Info_View_utilities[i]/MaxIGinAllView;


    // Utiltiy ( A Weighted Summation is taken from Entropy (Exploration Objective) &
    // Exponential prob ratio function (Victim Finding Objective)
    double utility;
    for (int i=0; i<Info_View_utilities.size(); i++)
    {
        utility= (exploration_weight*Info_View_utilities[i])+
                (victim_finding_weight*exp((Info_View_Max[i]-MaxProbinAllView)/(Info_View_Max[i])));

        // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
        if (utility > info_selected_utility_)
        {
            info_selected_utility_ = utility;
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

    std::cout << "Map of resoltuion " << mapping_module_->map.getResolution() << std::endl;
    std::cout << "exploration weight " << exploration_weight << std::endl;
    std::cout << "victim_finding_weight " << victim_finding_weight << std::endl;
    int i=MaxIGindex;
    std::cout << "utility correspond to the exploration is " << (exploration_weight*Info_View_utilities[i])+
                 (victim_finding_weight*exp((Info_View_Max[i]-MaxProbinAllView)/(Info_View_Max[i]))) << std::endl;

    std::cout << "corresponing IG and MAXXXXXX:" << Info_View_utilities[i] << " " << Info_View_Max[i] << std::endl;

     i=MaxProbIndex;

    std::cout << "utility correspond to the victim is " << (exploration_weight*Info_View_utilities[i])+
                 (victim_finding_weight*exp((Info_View_Max[i]-MaxProbinAllView)/(Info_View_Max[i]))) << std::endl;

    std::cout << "corresponing IG and MAXXXXXX:" << Info_View_utilities[i] << " " << Info_View_Max[i] << std::endl;

    std::cout << "utility correspond to the selected utility is " << info_selected_utility_ << std::endl;

    std::cout << "reality...... max IG is" << MaxIGinAllView << "while max prob is " << MaxProbinAllView << std::endl;

    std::cout << "Well well well , HFOV is " << mapping_module_->raytracing_->HFOV_deg <<std::endl;
     ros::Rate(0.1).sleep();
}


void view_evaluator_weighted::evaluateWireless()
{
    view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

    info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
    info_selected_direction_=0;
    info_utilities_.clear();

    selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

    double MaxIGinAllView=0;
    double MaxProbinAllView=0;

    for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
        geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double ViewIG=0;
        double ViewMax=-std::numeric_limits<float>::infinity();
        double utility_direction = calculateIG(p,mapping_module_);

        if (i==0)
        {
            calculateWirelessIGwithMax(p,mapping_module_,ViewIG,ViewMax);
        }

        if (i!=0)
        {
            if (!IsSamePosition(view_gen_->generated_poses[i],view_gen_->generated_poses[i-1]))
            {
                calculateWirelessIGwithMax(p,mapping_module_,ViewIG,ViewMax);
            }
            else
            ViewIG = Info_View_utilities.back();
            ViewMax = Info_View_Max.back();
        }

        if ((ViewIG>=0)||(ViewMax>=0))
        {
            Info_View_utilities.push_back(ViewIG);
            Info_View_Max.push_back(ViewMax);
            Info_WirelessDiection.push_back(utility_direction);
            Info_poses.push_back(p);

            // keep track of max information IG across all views
            if(MaxIGinAllView<ViewIG)
                MaxIGinAllView=ViewIG;

            // keep track of max Prob in All views
            if(MaxProbinAllView<ViewMax)
                MaxProbinAllView=ViewMax;
        }
    }

    // Normalized the IG view to 1
    for (int i=0; i<Info_View_utilities.size(); i++)
        Info_View_utilities[i]=Info_View_utilities[i]/MaxIGinAllView;



    // Utiltiy ( A Weighted Summation is taken from Entropy (Exploration Objective) &
    // Exponential prob ratio function (Victim Finding Objective)
    double utility;
    for (int i=0; i<Info_View_utilities.size(); i++)
    {
        utility= (exploration_weight*Info_View_utilities[i])+
                (victim_finding_weight*exp((Info_View_Max[i]-MaxProbinAllView)/Info_View_Max[i]));

        // First Select Pose that maximize the Utility
        if (utility >= info_selected_utility_)
        {
            info_selected_utility_ = utility;
            selected_pose_ = Info_poses[i];
        }

        // Second select the best exploration direction for the wireless best selected pose
        if (!(utility-info_selected_utility_))
            if (Info_WirelessDiection[i]>info_selected_direction_)
            {
                info_selected_direction_ = Info_WirelessDiection[i];
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

    std::cout << "Map of resoltuion " << mapping_module_->map.getResolution() << std::endl;
}


void view_evaluator_weighted::evaluateCombined()
{
    view_gen_->visualizeAllpose(view_gen_->generated_poses, view_gen_->rejected_poses);

    info_selected_utility_ = 0; //- std::numeric_limits<float>::infinity(); //-inf
    info_utilities_.clear();


    selected_pose_.position.x = std::numeric_limits<double>::quiet_NaN();

    double MaxIGinAllDLView=0;
    double MaxIGinAllThermalView=0;
    double MaxIGinAllWirelessView=0;

    double MaxProbinAllDLView=0;
    double MaxProbinAllThermalView=0;
    double MaxProbinAllWirelessView=0;



    for (int i=0; i<view_gen_->generated_poses.size() && ros::ok(); i++)
    {
        geometry_msgs::Pose p = view_gen_->generated_poses[i];
        double ViewIGDL=0;
        double ViewIGThermal=0;
        double ViewIGWireless=0;

        double ViewMaxDL=-std::numeric_limits<float>::infinity();
        double ViewMaxThermal=-std::numeric_limits<float>::infinity();
        double ViewMaxWireless=-std::numeric_limits<float>::infinity();


        calculateIGwithMax(p,mapping_module_->getMapLayer(MAP::DL),ViewIGDL,ViewMaxDL);
        calculateIGwithMax(p,mapping_module_->getMapLayer(MAP::THERMAL),ViewIGThermal,ViewMaxThermal);
        calculateIGwithMax(p,mapping_module_->getMapLayer(MAP::WIRELESS),ViewIGWireless,ViewMaxWireless);



        Info_View_utilities_DL.push_back(ViewIGDL);
        Info_View_utilities_Thermal.push_back(ViewIGThermal);
        Info_View_utilities_Wireless.push_back(ViewIGWireless);

        Info_View_Max_DL.push_back(ViewMaxDL);
        Info_View_Max_Thermal.push_back(ViewMaxThermal);
        Info_View_Max_Wireless.push_back(ViewMaxWireless);

        Info_poses.push_back(p);

        // keep track of max information IG across all views
        if(MaxIGinAllDLView<ViewIGDL)
            MaxIGinAllDLView=ViewIGDL;
        if(MaxIGinAllThermalView<ViewIGThermal)
            MaxIGinAllThermalView=ViewIGThermal;
        if(MaxIGinAllWirelessView<ViewIGWireless)
            MaxIGinAllWirelessView=ViewIGWireless;

        // keep track of max Prob in All views
        if(MaxProbinAllDLView<ViewMaxDL)
            MaxProbinAllDLView=ViewMaxDL;
        if(MaxProbinAllThermalView<ViewMaxThermal)
            MaxProbinAllThermalView=ViewMaxThermal;
        if(MaxProbinAllWirelessView<ViewMaxWireless)
            MaxProbinAllWirelessView=ViewMaxWireless;
    }

    // Normalized the IG view to 1
    for (int i=0; i<Info_View_utilities_DL.size(); i++)
    {
        Info_View_utilities_DL[i]=Info_View_utilities_DL[i]/MaxIGinAllDLView;
        Info_View_utilities_Thermal[i]=Info_View_utilities_Thermal[i]/MaxIGinAllThermalView;
        Info_View_utilities_Wireless[i]=Info_View_utilities_Wireless[i]/MaxIGinAllWirelessView;
    }

    // Utiltiy ( A Weighted Summation is taken from Entropy (Exploration Objective) &
    // Exponential prob ratio function (Victim Finding Objective)
    double utility_DL;
    double utility_Thermal;
    double utility_Wireless;
    double utility;


    for (int i=0; i<Info_View_utilities_DL.size(); i++)
    {
        // utility_DL
        utility_DL= (exploration_weight*Info_View_utilities_DL[i])+
                (victim_finding_weight*exp((Info_View_Max_DL[i]-MaxProbinAllDLView)/Info_View_Max_DL[i]));

        // utility_Thermal
        utility_Thermal= (exploration_weight*Info_View_utilities_Thermal[i])+
                (victim_finding_weight*exp((Info_View_Max_Thermal[i]-MaxProbinAllThermalView)/Info_View_Max_Thermal[i]));

        // utility_Wirelss
        utility_Wireless= (exploration_weight*Info_View_utilities_Wireless[i])+
                (victim_finding_weight*exp((Info_View_Max_Wireless[i]-MaxProbinAllWirelessView)/Info_View_Max_Wireless[i]));

        utility=alpha*utility_DL+beta*utility_Thermal+gama*utility_Wireless;

        // Ignore invalid utility values (may arise if we rejected pose based on IG requirements)
        if (utility > info_selected_utility_)
        {
            info_selected_utility_ = utility;
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

std::string view_evaluator_weighted::getMethodName()
{
    return "Weighted_Multiobjective_Utility";
}







