#include <victim_localization/test_NBV.h>
#include <iostream>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <Eigen/Geometry>
#include <eigen_conversions/eigen_msg.h>

TestNBV::TestNBV(const ros::NodeHandle &nh_,const ros::NodeHandle &nh_private_ ):
  nh(nh_),
  nh_private(nh_private_),
  NBV_loop_rate(20)
{
  pub_iteration_info = nh.advertise<victim_localization::IterationInfo>("victim_localization/iteration_info", 10);
  // >>>>>>>>>>>>>>>>>
  // Initialization
  // >>>>>>>>>>>>>>>>>
  state = NBVState::INITIALIZING;

  visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/Final_path"));
  visualTools->loadMarkerPub();

  visualTools->deleteAllMarkers();
  visualTools->enableBatchPublishing();

  ROS_INFO("test_NBV: Begin!");
}

bool TestNBV::CheckGazeboIsWorking()
{
  // wait until gazebo is ready...
  std_srvs::Empty srv;
  bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
  unsigned int i = 0;
  while (i <= 20 && !unpaused) {
    ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    unpaused = ros::service::call("/gazebo/unpause_physics", srv);
    ++i;
  }
  if (!unpaused) {
    ROS_FATAL("Could not wake up Gazebo.");
    return 0;
  } else {
    ROS_INFO("Unpaused the Gazebo simulation.");
    return 1;
  }
}

void TestNBV::initVehicle(){

  int vehicle_type;
  ros::param::param<int>("~vehicle_type",vehicle_type,0);
  switch(vehicle_type){
  default:
  case 0:
    vehicle_ = new VehicleControlIris();
    break;
  case 1:
    vehicle_ = new VehicleControlIrisOrigin();
    break;
  case 2:
    vehicle_ = new VehicleControlFloatingSensor();
    break;
  }
}

void TestNBV::initMap(){
  ros::param::param("~map_type", map_type, 0);
  switch(map_type)
  {
  default:
  case 0:
    Map_ = new victim_map_DL(nh,nh_private);
    break;
  case 1:
    Map_ = new victim_map_Thermal(nh,nh_private);
    break;
  case 2:
    Map_ = new Victim_Map_Wireless(nh,nh_private);
    break;
  case 3:
    Map_ = new victim_map_combined(nh,nh_private);
    break;
  }
}

void TestNBV::initNavigation(){

  ros::param::param("~nav_type", nav_type, 1);
  switch(nav_type)
  {
  default:
  case 0:
    // navigation_ = new straightLine(nh,nh_private,manager_);
    // break;
  case 1:
    navigation_ = new ReactivePathPlanner(nh,nh_private,manager_);
    break;
  }
  navigation_->start();
}

void TestNBV::initViewGenerator(){

  ros::param::param("~view_generator_type", view_generator_type, 0);

  switch(view_generator_type)
  {
  default:
  case 0:
    view_generate_ = new view_generator_IG();
    break;
  case 1:
    view_generate_ = new view_generator_ig_nn_adaptive();
    break;
  case 2:
    view_generate_ = new view_generator_ig_frontier();
    break;
  case 3:
    view_generate_ = new view_generator_ig_adaptive_frontier();
    break;
  }
}

void TestNBV::initViewEvaluator(){

  ros::param::param("~view_evaluator_type", view_evaluator_type, 5);

  switch(view_evaluator_type)
  {
  default:
  case 0:
    View_evaluate_ = new view_evaluator_ig();
    break;
  case 1:
    View_evaluate_ = new  view_evaluator_MaxSUM();
    break;
  case 2:
    View_evaluate_ = new view_evaluator_MaxMax();
    break;
  case 3:
    View_evaluate_ = new view_evaluator_MaxMIN();
  case 4:
    View_evaluate_ = new view_evaluator_MinNEIGH();
    break;
  case 5:
    View_evaluate_ = new view_evaluator_log_reward();
    break;
  case 6:
    View_evaluate_ = new view_evaluator_ig_exp();
    break;
  case 7:
    View_evaluate_ = new view_evaluator_ig_exp_max();
    break;
  }
}

void TestNBV::initTerminationCondition(){

  ros::param::param("~termination_type", termination_type, 1);

  switch(termination_type)
  {
  default:
  case 0:
    termination_check_module_ = new TerminationCheckMaxProbability();
    break;
  case 1:
    termination_check_module_ = new TerminationCheckMaxIterations();
    break;
  }
}


void TestNBV::initOctomap(){

  manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);

  Occlusion_Map_ = new Volumetric_Map(manager_);

  Occlusion_Map_->CheckTFConnection(); // TF tend to fails at start up, check until success

  CostMapROS_ = new costmap_2d::Costmap2DROS ("costmap",tf_);

  CostMapROS_->start();
  Occlusion_Map_->SetCostMapRos(CostMapROS_);
}


void TestNBV::initParameters(){

  ros::param::param("~detection_enabled", detection_enabled, false);//for debugging

  history_=  new nbv_history();
  view_generate_->setHistory(history_);
  view_generate_->setOcclusionMap(Occlusion_Map_);
  view_generate_->setCostMapROS(CostMapROS_);

  //Passing the Mapping and view_generator module to the view_evaluator
  view_generate_->setOctomapManager(manager_);
  View_evaluate_->setMappingModule(Map_);
  View_evaluate_->setViewGenerator(view_generate_);

  // initialize vehicle communicator
  drone_communicator_ = new vehicle_communicator(nh,nh_private,manager_);

  // pass vehicle communicator to Map Module
  Map_->setDroneCommunicator(drone_communicator_);
  Map_->SetNavMap(Occlusion_Map_->m_gridmap);
  Map_->setOctomapManager(manager_);

  // pass params to termination_condition
  termination_check_module_->setHistory(history_);
  termination_check_module_->setViewEvaluator(View_evaluate_);
}

void TestNBV::initAllModules()
{
  initVehicle();
  initMap();
  initOctomap();
  initNavigation();
  initViewGenerator();
  initViewEvaluator();
  initTerminationCondition();
  initParameters();
}

void TestNBV::updateHistory()
{
  history_->selected_poses.push_back(View_evaluate_->getTargetPose());
  history_->selected_utility.push_back(View_evaluate_->info_selected_utility_);
  history_->total_entropy.push_back(View_evaluate_->info_entropy_total_);

  history_->max_prob=Map_->curr_max_prob;

  history_->update();

  // Publish information about this iteration
  if (pub_iteration_info.getNumSubscribers() > 0)
  {
    victim_localization::IterationInfo iteration_msg;
    iteration_msg.iteration        = history_->iteration;
    iteration_msg.distance_total   = View_evaluate_->info_distance_total_;
    iteration_msg.entropy_total    = View_evaluate_->info_entropy_total_;

    iteration_msg.method_generation= view_generate_->getMethodName();
    iteration_msg.method_selection = View_evaluate_->getMethodName();
    iteration_msg.selected_pose    = View_evaluate_->getTargetPose();
    iteration_msg.selected_utility  =View_evaluate_->info_selected_utility_;
    iteration_msg.curr_max_prob  = Map_->curr_max_prob;
    iteration_msg.curr_max_loc_x  =Map_->curr_max_loc[0];
    iteration_msg.curr_max_loc_y  =Map_->curr_max_loc[1];

    iteration_msg.generator_type =view_generate_->generator_type;

    if (InitializedService)
    {
    iteration_msg.time_iteration   = (timer.getLatestTime("[NBVLoop]Iteration")/1000)-Map_->getServiceConnectionTimeout().toSec(); // time in sec
     InitializedService=false;
    }
    else
    iteration_msg.time_iteration   = (timer.getLatestTime("[NBVLoop]Iteration")/1000); // time in sec

    if (Map_->Maptype==MAP::COMBINED){
      iteration_msg.entropy_total_dl= View_evaluate_->info_dl_entropy_total_;
      iteration_msg.entropy_total_thermal = View_evaluate_->info_thermal_entropy_total_;
      iteration_msg.entropy_total_wireless = View_evaluate_->info_wireless_entropy_total_;

      iteration_msg.selected_utility_dl= View_evaluate_->info_dl_selected_utility_;
      iteration_msg.selected_utility_thermal = View_evaluate_->info_thermal_selected_utility_;
      iteration_msg.selected_utility_wireless = View_evaluate_->info_wireless_selected_utility_;

    }
    else  // if combined map is not used then assign the three map related data to zero
    {
      iteration_msg.entropy_total_dl=0;
      iteration_msg.entropy_total_thermal=0;
      iteration_msg.entropy_total_wireless=0;
      iteration_msg.selected_utility_dl=0;
      iteration_msg.selected_utility_thermal=0;
      iteration_msg.selected_utility_wireless=0;
    }
    pub_iteration_info.publish(iteration_msg);
  }
}

void TestNBV::evaluateViewpoints()
{
  if (state != NBVState::VIEWPOINT_EVALUATION)
  {
    std::cout << "[test_NBV] " << cc.red << "ERROR: Attempt to evaluate viewpoints out of order\n" << cc.reset;
    return;
  }

  std::cout << "[test_NBV] " << cc.green << "Evaluating viewpoints\n" << cc.reset;

  // Evaluate viewpoints
  View_evaluate_->update_parameters();
  View_evaluate_->evaluate();


  // Set the pose of the next best view
  p_ = View_evaluate_->getTargetPose();
  if ( std::isnan(p_.position.x) )
  {
    std::cout << "[test_NBV] " << cc.red << "View evaluator determined all poses are invalid. Terminating.\n" << cc.reset;
    state = NBVState::TERMINATION_MET;
    return;
  }


  std::cout << "[test_NBV] " << cc.green << "Done evaluating viewpoints\n" << cc.reset;
  state = NBVState::VIEWPOINT_EVALUATION_COMPLETE;
}

void TestNBV::navigate()
{
  std::string navigation_method;
  if (view_generate_->nav_type==0)
    navigation_method="Straight Line";

  else
    navigation_method="Reactive Planner";


  if (state != NBVState::NAVIGATION)
  {
    std::cout << "[test_NBV] " << cc.red << "ERROR: Attempt to navigate to viewpoint out of order\n" << cc.reset;
    return;
  }

  std::cout << "[test_NBV] " << cc.green << "Navigating to viewpoint using" << "[" << navigation_method << "]" << cc.reset;

  path_to_waypoint.clear();
  navigation_->setCurrentPose(drone_communicator_->GetPose());

  switch(view_generate_->nav_type)
  {
  default:
  case 0:  // move through fixed setpoint

    if (!drone_communicator_->Execute_waypoint(View_evaluate_->getTargetPose()))
    {
      ROS_WARN("Drone Communicator is unable to set waypoint, terminating....");
    }

    break;


  case 1:  // move through path if reactive planner is used

    // initially check for a straight line.

    if (!view_generate_->isCollide(View_evaluate_->getTargetPose()))
    {
      if (!drone_communicator_->Execute_waypoint(View_evaluate_->getTargetPose()))
      {
        ROS_WARN("Drone Communicator is unable to set waypoint, terminating....");
      }
      history_->black_listed_poses.clear();
      break;
    }

    // if straight line did not work then try the reactive planner

    Occlusion_Map_->GetActiveOctomapSize(grid_size_x,grid_size_y);
    navigation_->SetDynamicGridSize(grid_size_x,grid_size_y,0);

    Occlusion_Map_->GetActiveOrigin(grid_origin_x,grid_origin_y);
    navigation_->SetOriginPose(grid_origin_x,grid_origin_y,round(drone_communicator_->GetPose().position.z));

    if (navigation_->GeneratePath(View_evaluate_->getTargetPose(),path_to_waypoint)) {
      printf("path Found...\n");
      drone_communicator_->Execute_path(path_to_waypoint);
      history_->black_listed_poses.clear();
    }

    else {
      ROS_WARN("Unable to generate path, terminating....");
      if(view_generate_->generator_type==Generator::Frontier_Generator)
        history_->black_listed_poses.push_back(View_evaluate_->getTargetPose());
    }

    view_generate_->visualTools->deleteAllMarkers();
    view_generate_->visualTools->trigger();
    break;


  case 2: // move to waypoint in a discretized Path
    double step_size=0.02;
    path_to_waypoint = navigation_->Path_discretizationtoPath(drone_communicator_->GetPose(),View_evaluate_->getTargetPose(),
                                                              step_size);

    for (int i=0; i<path_to_waypoint.size() ; i++)
      std::cout << path_to_waypoint[i]  << std::endl;

    if (!drone_communicator_->Execute_path(path_to_waypoint))
    {
      ROS_WARN("Drone Communicator is unable to set path to waypoint, terminating....");
    }
    break;
  }
  std::cout << "[test_NBV] " << cc.green << "Waiting for Vehicle to reach Viewpoint\n" << cc.reset;

  // wait for the drone commander node until it moves the drone to the viewpoint
  while ((ros::ok() && !drone_communicator_->GetStatus()))
  {
    ros::spinOnce();
    ros::Rate(5).sleep();
  }
  std::cout << "[test_NBV] " << cc.green << "Done Navigating to viewpoint\n" << cc.reset;
  state = NBVState::NAVIGATION_COMPLETE;
}

void TestNBV::generateViewpoints()
{
  if (state != NBVState::VIEWPOINT_GENERATION)
  {
    std::cout << "[test_NBV] " << cc.red << "ERROR: Attempt to generate viewpoints out of order\n" << cc.reset;
    return;
  }
  std::cout << "[test_NBV] " << cc.green << "Generatring viewpoints\n" << cc.reset;

  //Delete the Reactive Planner Visualziation Markers
  navigation_->reactivePlannerServer->visualTools->deleteAllMarkers();
  navigation_->reactivePlannerServer->visualTools->trigger();

  view_generate_->setCurrentPose(drone_communicator_->GetPose());
  view_generate_->setvictimmap(Map_->map,Map_->getlayer_name());

  // std::cout << "curretn pose: " << drone_communicator_->GetPose() <<" "<< std::endl;
  view_generate_->generateViews();

  if (view_generate_->generated_poses.size() == 0)
  {
    std::cout << "[test_NBV] " << cc.red << "View generator created no poses. Terminating.\n" << cc.reset;
    state = NBVState::UPDATE_MAP_COMPLETE;
  }
  else
  {
    state = NBVState::VIEWPOINT_GENERATION_COMPLETE;
  }
}


void TestNBV::UpdateMap()
{
  if (state != NBVState::START_MAP_UPDATE)
  {
    std::cout << "[test_NBV] " << cc.red << "ERROR: Attempt to update map out of order\n" << cc.reset;
    return;
  }

  std::cout << "[test_NBV] " << cc.green << "Updateing [" << Map_->getlayer_name() << "]" << cc.reset;

  Map_->setCurrentPose(drone_communicator_->GetPose());

  Map_->Update();

  if (Map_->getMapResultStatus().victim_found)
  {
    std::cout << cc.magenta << "Victim Found at Location: " << "(x,y)=" << "(" <<
                 (Map_->getMapResultStatus().victim_loc)[0] << "," <<
                 (Map_->getMapResultStatus().victim_loc)[1] << ") terminating...\n" << cc.reset;

    return;
  }
  state = NBVState::UPDATE_MAP_COMPLETE;
}

void TestNBV::terminationCheck()
{
    std::cout << "[NBVLoop] " << cc.green << "Checking termination condition for iteration " << history_->iteration << "\n" << cc.reset;

  if (termination_check_module_->isTerminated())
    state = NBVState::TERMINATION_MET;
  else
    state = NBVState::START_MAP_UPDATE;
}

void TestNBV::runStateMachine()
{
  ROS_INFO("test_NBV: Starting victim_localization node waiting for the drone to take off and rotate.");
  state = NBVState::STARTING_ROBOT;

  timer.start("NBV: Total Time");
  StartTimeForNBV = ros::Time::now();
  while (ros::ok())
  {
    switch(state)
    {
    case NBVState::STARTING_ROBOT:
      if (!drone_communicator_->GetStatus()){
        break; // check if the drone is ready from drone commander node
      }
      Occlusion_Map_->Stop();
      std::cout << "drone is ready\n";
      state = NBVState::START_MAP_UPDATE;
      break;

    case NBVState::START_MAP_UPDATE:
      timer.start("[NBVLoop]Iteration");  // detect and update map
      //ros::Rate(1).sleep();
      Occlusion_Map_->GetPointCloud();
      while (!Occlusion_Map_->GetPointCloudDone()) {ros::spinOnce() ; NBV_loop_rate.sleep();}
      UpdateMap();
      //if (state!=NBVState::TERMINATION_MET) state = NBVState::UPDATE_MAP_COMPLETE;
      break;

    case NBVState::UPDATE_MAP_COMPLETE:
      state = NBVState::VIEWPOINT_GENERATION ;
      generateViewpoints();
      break;

    case NBVState::VIEWPOINT_GENERATION_COMPLETE:
      state = NBVState::VIEWPOINT_EVALUATION;
      evaluateViewpoints();
      break;

    case NBVState::VIEWPOINT_EVALUATION_COMPLETE:
      state = NBVState::NAVIGATION;
      navigate();
      break;

    case NBVState::NAVIGATION_COMPLETE:
      timer.stop("[NBVLoop]Iteration");
      updateHistory();
      terminationCheck();
      break;

    case NBVState::TERMINATION_MET:

      for(int i =0; i< (history_->selected_poses.size()- 1) ;i+=1)
      {
        visualTools->publishLine(history_->selected_poses[i].position,history_->selected_poses[i+1].position, rviz_visual_tools::BLUE,rviz_visual_tools::XLARGE);
      }
      for(int i =0; i< (history_->selected_poses.size()- 1) ;i+=1)
      {
        visualTools->publishArrow(history_->selected_poses[i],rviz_visual_tools::GREEN,rviz_visual_tools::XXLARGE,0.5);
      }
      visualTools->trigger();
      Map_->publish_Map();
      timer.stop("NBV: Total Time");

      //std::cout << "time Taken: Method1" <<   (timer.getLatestTime("NBV: Total Time")/1000.0) << std::endl;
      //std::cout << "time Taken: Method2" <<   (ros::Time::now()-StartTimeForNBV).toSec() << std::endl;
      ros::Duration TotalNBVTime;
      TotalNBVTime.fromNSec(timer.getLatestTime("NBV: Total Time"));
      ros::Duration Service_Startup_time = Map_->getServiceConnectionTimeout(); // this is added to account for the time due possible connection drop for the first time when connecting  NBV node and SSD service
      termination_check_module_->setNBVTimeTaken(TotalNBVTime-Service_Startup_time);
      termination_check_module_->SaveResults();

      timer.dump();

      // Clean up
      std::cout << "[NBV_test] " << cc.yellow << "Shutting down\n" << cc.reset;
      ros::shutdown();

      //ros::spinOnce();
      //ros::Rate(5).sleep();
      break;
    }
    ros::spinOnce();
    NBV_loop_rate.sleep();
  }

  // Dump time data
  timer.dump();


  // Clean up
  std::cout << "[NBV_test] " << cc.yellow << "Shutting down\n" << cc.reset;
  ros::shutdown();
}


void sigIntHandler(int sig)
{
  std::cout << cc.yellow << "Handling SIGINT exception\n" << cc.reset;
  // Forces ros::Rate::sleep() to end if it's stuck (for example, Gazebo isn't running)
  ros::shutdown();
}

int main(int argc, char **argv)
{  
  ros::init(argc, argv, "test_NBV", ros::init_options::NoSigintHandler);
  signal(SIGINT, sigIntHandler);

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  //Create an object of class test_NBV

  TestNBV *test_;
  test_ = new TestNBV(nh,nh_private);

  if(!test_)
  {
    ROS_ERROR("Something is wrong");
    return 0;
  }

  if(!test_->CheckGazeboIsWorking()) return 0;

  test_->initAllModules();
  test_->runStateMachine();
  return 0;
}
