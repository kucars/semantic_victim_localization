#include "victim_localization/visualize_results1.h"

visualizeResults1::visualizeResults1(const ros::NodeHandle &nh_,const ros::NodeHandle &nh_private_ ):
    nh(nh_),
    nh_private(nh_private_)
{
    visualTools.reset(new rviz_visual_tools::RvizVisualTools("world", "/generatedPath"));
    visualTools->loadMarkerPub();
    visualTools->deleteAllMarkers();
    visualTools->enableBatchPublishing();
}

void visualizeResults1::SetPathtoResults(std::string path)
{
 resultPath = path;
}

void visualizeResults1::trigger()
{
   std::string File_poses = resultPath+"/POSES.txt";

  //------Read Path poses-----
     const char * filename1 = File_poses.c_str();

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
         Poses.push_back(pose);
    }

     //------Read Generated Path-----
     std::string File_path = resultPath+"/PATH.txt";

        const char * filename2 = File_path.c_str();

        assert(filename2 != NULL);
        filename2 = strdup(filename2);
        FILE *file2 = fopen(filename2, "r");
        if (!file2)
        {
            std::cout<<"\nCan not open File";
            fclose(file2);
        }

        geometry_msgs::Pose pathPose;

        while (!feof(file2))
        {
            fscanf(file2,"%lf %lf %lf %lf %lf %lf %lf\n",&locationx,&locationy,&locationz,&qx,&qy,&qz,&qw);
            pathPose.position.x = locationx;
            pathPose.position.y = locationy;
            pathPose.position.z = locationz;
            pathPose.orientation.x = qx;
            pathPose.orientation.y = qy;
            pathPose.orientation.z = qz;
            pathPose.orientation.w = qw;
            Path.push_back(pathPose);
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
     for(int i =0; i< Poses.size()  ;i++)
     {
       visualTools->publishArrow(Poses[i],rviz_visual_tools::GREEN,rviz_visual_tools::XXLARGE,0.5);
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

  visualizeResults1 *results;
  results= new visualizeResults1(nh,nh_private);

  std::cout << cc.cyan << "[Start Visualization]\n" << cc.reset;

  std::string result_path;
  ros::param::param<std::string>("~resultFolder",result_path,ros::package::getPath("victim_localization")+"/Data");

  results->SetPathtoResults(result_path);
  results->trigger();

  ros::spin();
  return 0;
}
