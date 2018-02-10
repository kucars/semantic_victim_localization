#include "victim_localization/visualize_results.h"

visualizeResults::visualizeResults(const ros::NodeHandle &nh_,const ros::NodeHandle &nh_private_ ):
    nh(nh_),
    nh_private(nh_private_)
{
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/generatedPath"));
    visualTools->loadMarkerPub();
    visualTools->deleteAllMarkers();
    visualTools->enableBatchPublishing();
}

void visualizeResults::SetPathtoResults(std::string path)
{
 resultPath = path;
}

void visualizeResults::trigger()
{

   std::string pathFile = resultPath+"/PATH.txt";
  //------Read Path poses-----
     const char * filename1 = pathFile.c_str();

     assert(filename1 != NULL);
     filename1 = strdup(filename1);
     FILE *file1 = fopen(filename1, "r");
     if (!file1)
     {
         std::cout<<"\nCan not open File";
         fclose(file1);
     }

     double locationx,locationy,locationz,qx, qy, qz , qw;
     geometry_msgs::Pose pose;

     while (!feof(file1))
     {
         fscanf(file1,"%lf %lf %lf %lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qx,&qy,&qz,&qw);
         pose.position.x = locationx;
         pose.position.y = locationy;
         pose.position.z = locationz;
         pose.orientation.x = qx;
         pose.orientation.y = qy;
         pose.orientation.z = qz;
         pose.orientation.w = qw;
         Path.push_back(pose);
    }

     //------Read octree -----
     manager_ = new volumetric_mapping::OctomapManager(nh, nh_private);
     manager_->octree_->readBinary(resultPath+"/environment");

     //------Visualize Results Continuously -----
     while (ros::ok())
     {
     for(int i =0; i< (Path.size()- 1) ;i++)
     {
       visualTools->publishLine(Path[i].position,Path[i+1].position, rviz_visual_tools::BLUE,rviz_visual_tools::XLARGE);
     }
     for(int i =0; i< Path.size() ;i++)
     {
       visualTools->publishArrow(Path[i],rviz_visual_tools::GREEN,rviz_visual_tools::XXLARGE,0.5);
     }
     visualTools->trigger();

     manager_->publishAll();
     ros::spinOnce();
     ros::Rate(0.5).sleep();
     }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "results");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  visualizeResults *results;
  results= new visualizeResults(nh,nh_private);

  std::cout << cc.cyan << "[Start Visualization]\n" << cc.reset;

  std::string result_path;
  ros::param::param<std::string>("~resultFolder",result_path,ros::package::getPath("victim_localization")+"/Data");

  results->SetPathtoResults(result_path);
  results->trigger();

  ros::spin();
  return 0;
}
