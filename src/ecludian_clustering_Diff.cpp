#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>

#include <Eigen/Geometry>
#include <sensor_msgs/PointCloud2.h>

#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/objdetect/objdetect.hpp"
#include <geometry_msgs/PoseStamped.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/centroid.h>


//tf
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_listener.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>


//sync_time
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>


#include "utilities/pose_conversion.h"

#include <kuri_usar_teleoperation/box.h>
#include <kuri_usar_teleoperation/boxes.h>
#include <detect_msgs/position.h>
#include <detect_msgs/positions.h>
#include <detect_msgs/Dec_vic.h>


class Clustering
{
tf::TransformListener *tf_listener;

public:
Clustering()
{
tf_listener = new tf::TransformListener();

// == Consts
std::string topic_depth_in   = "/front_cam/depth/image_raw";
std::string topic_depth_out = "/larger_cluster";
std::string topic_box_in = "/ssd_detction/box";
std::string location = "/mavros/local_position/pose";


depth_in_  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, topic_depth_in.c_str(), 1);
box_ = new message_filters::Subscriber<kuri_usar_teleoperation::boxes>(nh_, topic_box_in.c_str(), 1);
loc_sub_  = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, location.c_str(), 1);

sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(4), *depth_in_, *box_ ,*loc_sub_);
sync->registerCallback(boost::bind(&Clustering::CallBack, this, _1, _2,_3));

pub_depth = nh_.advertise<sensor_msgs::PointCloud2>(topic_depth_out, 4);
VicDec_pub = nh_.advertise<detect_msgs::Dec_vic>("victim_Dec", 100);

}


void CallBack(const sensor_msgs::ImageConstPtr& input_depth, const kuri_usar_teleoperation::boxes::ConstPtr& boxes_, const geometry_msgs::PoseStamped::ConstPtr& loc)
{


    // == Transform
    tf::StampedTransform transform;


    try{
        // Listen for transform
        tf_listener->lookupTransform("/world", "/front_cam_depth_optical_frame", ros::Time(0), transform);
    }
    catch (tf::TransformException ex){
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    Eigen::Matrix4d tf_eigen = pose_conversion::convertStampedTransform2Matrix4d(transform);
    //Eigen::Affine3f tf_eigen_aff;
    //tf_eigen_aff.matrix()=tf_eigen;


    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(input_depth, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr depth_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);


    //define detection msgs
    detect_msgs::Dec_vic vicDec_;
    vicDec_.curr_loc.x=loc->pose.position.x;
    vicDec_.curr_loc.y=loc->pose.position.y;
    vicDec_.z=loc->pose.position.z;
    vicDec_.yaw = pose_conversion::getYawFromQuaternion(loc->pose.orientation);


    if (boxes_->boxes[0].Class != "person"
            || (boxes_->boxes[0].ymax==0 && boxes_->boxes[0].ymin==0 )) {
    vicDec_.victim=false;
    vicDec_.victim_loc.x=0;
    vicDec_.victim_loc.y=0;
     }

    else {
        std::cout << vicDec_.victim << std::endl;
        int y_max=boxes_->boxes[0].ymax;
        int y_min=boxes_->boxes[0].ymin;
        int x_max=boxes_->boxes[0].xmax;
        int x_min=boxes_->boxes[0].xmin;

      // Fill in the cloud data
        depth_cloud->width    = (y_max-y_min) * (x_max-x_min);
        depth_cloud->height   = 1;
        depth_cloud->is_dense = false;
        depth_cloud->points.resize (depth_cloud->width * depth_cloud->height);


      int cnt=0;
      int cnt_non=0;
      for (int i=x_min; i<x_max; i++){
          for (int j=y_min; j<y_max; j++){
              int index=(j-y_min)+cnt*(y_max-y_min);
              float depth_=cv_ptr->image.at<float>(j,i);
              if (std::isnan(depth_)) cnt_non++;
              float FL_SS=524.2422531097977;

              depth_cloud->points[index].x=(i- 320.5)*((depth_)/(FL_SS));
              depth_cloud->points[index].y=(j- 240.5)*((depth_)/(FL_SS));
              depth_cloud->points[index].z=depth_;

             // std::cout <<  "Index: " << "(" << i << "," << j << ")" << depth_cloud->points[index].z << std::endl;

          }
          cnt++;
      }


       //std::cout << "size_of_cloud" << depth_cloud->size() << std::endl;
       //std::cout <<  "theoritical size: " << depth_cloud->width << std::endl;
       //std::cout <<  "non values: " << cnt_non << std::endl;
       //std::cout <<  x_max << " " << y_max << " " << x_min << " " << y_min << std::endl;

        //remove non values from the point cloud
           std::vector<int> indices;
           pcl::removeNaNFromPointCloud(*depth_cloud, *depth_cloud, indices);
        //
  /*
           pcl::transformPointCloud(*depth_cloud, *transformed_cloud, tf_eigen);

       pub_depth.publish(output_msg);

  */

      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>),
      filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>),filtered_cloud_1 (new pcl::PointCloud<pcl::PointXYZ>),
      cloud_f(new pcl::PointCloud<pcl::PointXYZ>);

     //  Create the filtering object: downsample the dataset using a leaf size of 1cm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud (depth_cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*filtered_cloud);





      // Create the filtering object: downsample the dataset using a leaf size of 1cm



      //  std::cout << "PointCloud after filtering z has: " << filtered_cloud->points.size ()  << " data points." << std::endl; //*

        // Create the segmentation object for the planar model and set all the parameters
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
        seg.setOptimizeCoefficients (true);
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        //seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.01);



        //std::cout << "cloud before process" << filtered_cloud->size() ;

        int i=0, nr_points = (int) filtered_cloud->points.size ();
        while (filtered_cloud->points.size () > 0.3 * nr_points)
        {
          // Segment the largest planar component from the remaining cloud
          seg.setInputCloud (filtered_cloud);
          seg.segment (*inliers, *coefficients);
          if (inliers->indices.size () == 0)
          {
            //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
          }



          // Extract the planar inliers from the input cloud
          pcl::ExtractIndices<pcl::PointXYZ> extract;
          extract.setInputCloud (filtered_cloud);
          extract.setIndices (inliers);
          extract.setNegative (false);

          // Get the points associated with the planar surface
          extract.filter (*cloud_plane);
         // std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

          // Remove the planar inliers, extract the rest
          extract.setNegative (true);
          extract.filter (*cloud_f);
          *filtered_cloud = *cloud_f;
        }


        if (!filtered_cloud->points.size()) {  // this is to skip the empty cloud error of the Kdtree
            vicDec_.victim=false;
            vicDec_.victim_loc.x=0;
            vicDec_.victim_loc.y=0;
            goto End_;
            }



          //std::cout << "cloud after process" << filtered_cloud->size() ;
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (filtered_cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.1); // 10cm
        ec.setMinClusterSize (100);
        ec.setMaxClusterSize (25000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (filtered_cloud);
        ec.extract (cluster_indices);



      std::vector <pcl::PointCloud<pcl::PointXYZ>::Ptr, Eigen::aligned_allocator <pcl::PointCloud <pcl::PointXYZ>::Ptr > > cloud_clusters;
        int j = 0;
        for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
          cloud_cluster->points.push_back((filtered_cloud->points[*pit])); //*
          cloud_cluster->width = cloud_cluster->points.size ();
          cloud_cluster->height = 1;
          cloud_cluster->is_dense = true;

          cloud_clusters.push_back(cloud_cluster);


          //std::cout << "PointCloud representing the Cluster: " << cloud_clusters[j]->size () << " data points." << std::endl;

          j++;
        }





        // == convert max pcl cloud back to pointcloud2 and publish it
         sensor_msgs::PointCloud2 output_msg;
         pcl::toROSMsg(*cloud_clusters[0], output_msg); 	//cloud of original (white) using original cloud
          output_msg.header.frame_id = "front_cam_depth_optical_frame";
          output_msg.header.stamp = ros::Time::now();
          pub_depth.publish(output_msg);

       // getting the centroid of the cluster in world frame
          pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
          temp_cloud=cloud_clusters[0];
          pcl::CentroidPoint<pcl::PointXYZ> centroid;

          while (temp_cloud->points.size()) {
              centroid.add(temp_cloud->points.back());
              temp_cloud->points.pop_back();
          }



          pcl::transformPointCloud(*depth_cloud, *transformed_cloud, tf_eigen);
          pcl::PointXYZ point_centroid;
          geometry_msgs::PointStamped point_centroid_gm;
          geometry_msgs::PointStamped point_out;

          centroid.get(point_centroid);
          point_centroid_gm.point=pose_conversion::convertToGeometryMsgPoint(point_centroid);
          point_centroid_gm.header.frame_id="front_cam_depth_optical_frame";

          try
          {
            tf_listener->transformPoint("/world", point_centroid_gm, point_out);
          }
          catch (tf::TransformException &ex)
          {
            printf ("Failure %s\n", ex.what()); //Print exception which was caught
      return ;
          }
          std::cout << "centroid point in world frame:" << point_out << std::endl;
          vicDec_.victim=true;
          vicDec_.victim_loc.x=point_out.point.x;
          vicDec_.victim_loc.y=point_out.point.y;

    }
End_:
    VicDec_pub.publish(vicDec_);


}




private:
ros::NodeHandle nh_;
typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, kuri_usar_teleoperation::boxes,geometry_msgs::PoseStamped> MySyncPolicy;
message_filters::Subscriber<sensor_msgs::Image> *depth_in_;
message_filters::Subscriber<kuri_usar_teleoperation::boxes> *box_;
message_filters::Subscriber<geometry_msgs::PoseStamped> *loc_sub_; //added

message_filters::Synchronizer<MySyncPolicy> *sync;
ros::Publisher pub_depth;
ros::Publisher VicDec_pub;



};

int main(int argc, char **argv)
{
// >>>>>>>>>>>>>>>>>
// Initialize ROS
// >>>>>>>>>>>>>>>>>
ros::init(argc, argv, "clustering");

Clustering cluster;

ros::spin();

return 0;

}

