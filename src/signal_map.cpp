#include "ros/ros.h"
#include "std_msgs/String.h"
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <cmath>
#include <iostream>
#include <iostream>
#include <sstream>
#include <string>
#include <octomap_msgs/conversions.h>
using namespace octomap;

double calcRecievedPowProb(point3d pt)
{
  float  xr = 50, yr=50, zr=0;
  double Pt = 0.5;
  double Gr = 6.309573445;
  double Gt = 10;
  float  pi = 3.14159265359;
  double f = 5000000000;
  double c = 300000000;
//  float  Pmax = Pt*Gt*Gr*c*c/(4*pi*1*f*4*pi*1*f);
//  std::cout << "power max = "<<Pmax<< std::endl;
  float  Pmin = Pt*Gt*Gr*c*c/(4*pi*100*f*4*pi*100*f);
  std::cout << "power min = "<<Pmin<< std::endl;
  float  d = sqrt((xr-pt.x())*(xr-pt.x())+(yr-pt.y())*(yr-pt.y())+(zr-pt.z())*(zr-pt.z()));
  //std::cout << "distance = "<<d<< std::endl;
  float  power = Pt*Gt*Gr*c*c/(4*pi*d*f*4*pi*d*f);
//  float  prob =  (28-(d*0.5))/28;
//  float  prob =  power/Pmax;
//  float prob = abs(power-Pmax)/Pmax;
//   float prob = (100-d)/100;
   float prob = abs(Pmin-power)/Pmin;
   std::cout << "prob = "<<prob<< std::endl;
  return prob;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "signal_map");
  ros::NodeHandle n;
  ros::Publisher octomapPub = n.advertise<octomap_msgs::Octomap>("octomap", 1);
  float max = 0 ; 
   OcTree tree(1);
   tree.setProbHit(0.99999);
   tree.setProbMiss(0);
   tree.setClampingThresMax(0.99999);
   tree.setClampingThresMin(0);
    for (int x=-10; x<=10; x++) {
	for (int y=-10; y<=10; y++) {
	  for (int z=1; z<=5; z++) {
	    point3d end((float)x,(float)y,(float)z);
	    
	    double prob =  calcRecievedPowProb(end);
	    if (prob > max)
	      max = prob ; 
	    
	    double lg= logodds(prob);
	   // std::cout << "lg = "<<lg<< std::endl;

	    tree.setNodeValue(end, lg);
	    
	  }
	}
    }
      std::cout << "max = "<<max<< std::endl;

  
  
  
  
  ros::Rate loop_rate(10);
  octomap_msgs::Octomap octomap ;
      octomap.binary = 1 ;
      octomap.id = 1 ;
      octomap.resolution =0.1;
      octomap.header.frame_id = "/map";

  while (ros::ok())
  {

      octomap.header.stamp = ros::Time::now();
      bool res = octomap_msgs::fullMapToMsg(tree, octomap);
      if(res)
      {
          octomapPub.publish(octomap);
      }
      else
      {
          ROS_WARN("OCT Map serialization failed!");
      }
      ros::spinOnce();
      loop_rate.sleep();
  }
  return 0;
}
