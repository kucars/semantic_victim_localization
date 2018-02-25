#include "victim_localization/termination_check_base.h"

TerminationCheckBase::TerminationCheckBase()
{
  ros::param::param<int>("~termination_repeat_window_size", repeat_window_size_, 8);
  ros::param::param<std::string>("~SaveDataFolder",saveFolder, std::string("Data"));
  result_status="Success";
}

bool TerminationCheckBase::isTerminated()
{
  std::cout << "[TerminationCheckBase]: " << cc.yellow << "update() not defined. Used derived class with proper implimentation.\n" << cc.reset;
  return false;
}

void TerminationCheckBase::setHistory(nbv_history* h)
{
  nbv_history_ = h;
}

void TerminationCheckBase::setViewEvaluator(view_evaluator_base* v)
{
  view_evaluator = v;
}

void TerminationCheckBase::setNBVTimeTaken(ros::Duration t)
{
  NBVDurationTaken=t;
}

void TerminationCheckBase::SaveResults()
{
  std::string path = ros::package::getPath("victim_localization");
  std::cout << "path to package is" << path <<std::endl;

  // ---- Save octomap ---
  std::string file_path;
  file_path=path + "/" + saveFolder+ "/";
  view_evaluator->view_gen_->manager_->
      octree_->writeBinary(file_path+"environment");

  // --- Save selected Poses ---
  ofstream myfile (file_path+"POSES.txt");
  if (myfile.is_open())
  {
    for(int i =0; i< (nbv_history_->selected_poses.size()) ;i+=1)
    {
      myfile << nbv_history_->selected_poses[i].position.x << " "
             << nbv_history_->selected_poses[i].position.y<< " "
             << nbv_history_->selected_poses[i].position.z<< " "
             << nbv_history_->selected_poses[i].orientation.x<< " "
             << nbv_history_->selected_poses[i].orientation.y<< " "
             << nbv_history_->selected_poses[i].orientation.z<< " "
             << nbv_history_->selected_poses[i].orientation.w<< "\n";
    }
    myfile.close();
  }
  else
    cout << "Unable to open PATH.txt file";


  // --- Save Path Generated ---
  ofstream myfile2 (file_path+"PATH.txt");
  if (myfile2.is_open())
  {
    for(int i =0; i< (nbv_history_->selected_poses_along_path.size()) ;i+=1)
    {
      myfile2 << nbv_history_->selected_poses_along_path[i].position.x << " "
             << nbv_history_->selected_poses_along_path[i].position.y<< " "
             << nbv_history_->selected_poses_along_path[i].position.z<< " "
             << nbv_history_->selected_poses_along_path[i].orientation.x<< " "
             << nbv_history_->selected_poses_along_path[i].orientation.y<< " "
             << nbv_history_->selected_poses_along_path[i].orientation.z<< " "
             << nbv_history_->selected_poses_along_path[i].orientation.w<< "\n";
    }
    myfile2.close();
  }
  else
    cout << "Unable to open PATH.txt file";

  //----- Save Occupancy Map ----
  // The occupancy map is initially upscaled to improve the image resolution
  cv::Mat occupancyImage;
  grid_map::GridMap gridMap;
  Victim_Map_Base *Map_=view_evaluator->mapping_module_;
  double upscale_factor=32;
  double upscale_resolution= Map_->map_resol/upscale_factor;
  std::string mapName= "SaveOccupancy";
  gridMap.setGeometry(Map_->map.getLength(),upscale_resolution);
  gridMap.add(mapName,0);

  occupancyImage= upscaleOccupancyImage(Map_->map,Map_->layer_name,gridMap,mapName);
  cv::imwrite(file_path+Map_->getlayer_name()+"Occupancy.jpeg",occupancyImage);

  // In case the Fused_Map is used , then save its individual maps
  if (Map_->Maptype==MAP::COMBINED)
  {
    // Save DL MAP
    gridMap[mapName].setZero();  // reset gridMap;
    occupancyImage= upscaleOccupancyImage(Map_->getMapLayer(MAP::DL)->map,
                                 Map_->getMapLayer(MAP::DL)->getlayer_name(),
                                 gridMap,mapName);
    cv::imwrite(file_path+Map_->getMapLayer(MAP::DL)->getlayer_name()+"Occupancy.jpeg",occupancyImage);

    // Save THERMAL MAP
    gridMap[mapName].setZero();  // reset gridMap;
    occupancyImage= upscaleOccupancyImage(Map_->getMapLayer(MAP::THERMAL)->map,
                                 Map_->getMapLayer(MAP::THERMAL)->getlayer_name(),
                                 gridMap,mapName);
    cv::imwrite(file_path+Map_->getMapLayer(MAP::THERMAL)->getlayer_name()+"Occupancy.jpeg",occupancyImage);

    // Save WIRELESS MAP
    gridMap[mapName].setZero();  // reset gridMap;
    occupancyImage= upscaleOccupancyImage(Map_->getMapLayer(MAP::WIRELESS)->map,
                                 Map_->getMapLayer(MAP::WIRELESS)->getlayer_name(),
                                 gridMap,mapName);
    cv::imwrite(file_path+Map_->getMapLayer(MAP::WIRELESS)->getlayer_name()+"Occupancy.jpeg",occupancyImage);
  }

  //---- Store Test Results ---
  ofstream Results (file_path+Map_->layer_name+"Result.txt");
  if (Results.is_open())
  {
    Results << "time taken: " << NBVDurationTaken.toSec() << "\n"
            << "Total entropy: " << view_evaluator->info_entropy_total_ << "\n"
            << "distance: " <<  view_evaluator->info_distance_total_ << "\n"
            << "iteration: " << nbv_history_->iteration << "\n"
            << "Result Status: " << result_status << std::endl << "\n";
    Results.close();
  }
  else cout << "Unable to find Results file";
}

cv::Mat TerminationCheckBase::upscaleOccupancyImage(grid_map::GridMap inputMap , std::string Input, grid_map::GridMap upscaledMap, std::string Output)
{
  cv::Mat image;
  const float minValue = 0.0;
  const float maxValue = 1.0;
  Index index;
  Position position;
  for (grid_map::GridMapIterator iterator(upscaledMap);
       !iterator.isPastEnd(); ++iterator)
  {
    index=*iterator;
    upscaledMap.getPosition(index,position);
    upscaledMap.atPosition(Output,position)=1-inputMap.atPosition(Input,position);
  }

  GridMapCvConverter::toImage<unsigned char, 4>(upscaledMap, Output, CV_8UC4, minValue, maxValue, image);

  return image;
}

