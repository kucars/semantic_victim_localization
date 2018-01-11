#include "victim_localization/victim_thermal_detector.h"


victim_thermal_detector::victim_thermal_detector():
victim_detector_base()
{
  ros::param::param<double>("~min_Dist_Between_Blobs", minDistBetweenBlobs_ , 40.0);
  ros::param::param<double>("~thermal_min_Area_Victim", minAreaVictim_ , 40.0);

  ros::param::param("~thermal_Image_topic", thermal_image_topic, std::string("/thermal_cam/image_raw"));
  ros::param::param("~topic_Odometry", topic_Odometry, std::string("iris/mavros/local_position/pose"));

  image_transport::ImageTransport it(nh_);

  //message_filters configurations
  thermal_image_sub  = new message_filters::Subscriber<sensor_msgs::Image>(nh_, thermal_image_topic, 1);
  loc_sub_  = new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, topic_Odometry, 1);
  sync = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(4), *thermal_image_sub ,*loc_sub_);

  // Callbacks
  sync->registerCallback(boost::bind(&victim_thermal_detector::imageCallback, this, _1, _2));


 // sub_image = it.subscribe("/thermal_cam/image_raw", 1, &victim_thermal_detector::imageCallback,this);
  pub_detection_ = it.advertise("image_detection1", 10);
}


victim_thermal_detector::~victim_thermal_detector(){}

void victim_thermal_detector::imageCallback(const sensor_msgs::ImageConstPtr& img,
                                            const geometry_msgs::PoseStamped::ConstPtr& loc)
{
  input_image=img;
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvShare(img, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  img_proc=cv_ptr->image;

  current_ps= *loc;
  }


  void victim_thermal_detector::BlobDetection(void){
  victim_found=false;  //initially set victim_found to false
  std::cout << "blob detecotr is runing...." << std::endl;
  //Perform blob detection

  cv::Mat threshold_image;
  cv::threshold(img_proc,threshold_image,250,255,cv::THRESH_BINARY);
  //Perform blob detection


  cv::SimpleBlobDetector::Params params;
  params.filterByColor = true;
  params.blobColor = 255;
  params.minDistBetweenBlobs = minDistBetweenBlobs_;
  params.filterByArea = true;
  params.minArea = minAreaVictim_;
  params.maxArea = threshold_image.rows * threshold_image.cols;
  params.filterByCircularity = false;
  params.filterByColor = false;
  params.filterByConvexity = false;
  params.filterByInertia = false;

  capture_ps=current_ps;

  std::cout << "this is the expected yaw for the drone pointOfInterest..."  << pose_conversion::getYawFromQuaternion(capture_ps.pose.orientation) << std::endl;
  std::cout << "this is the expected time..." << capture_ps.header.stamp << std::endl;

  cv::Ptr<cv::SimpleBlobDetector> blob_detector = cv::SimpleBlobDetector::create(params);
  std::vector<cv::KeyPoint> keypoints;
  blob_detector->detect(threshold_image,keypoints);

  for(unsigned int i=0; i<keypoints.size();i++)
  {
      plot_=true; // draw box on the image as long as the detected object is within region of interest

      cv::KeyPoint k = keypoints.at(i);
      std::cout << "blob found at (x,y)= " << k.pt.x << "," << k.pt.y << std::endl;
      ROS_DEBUG("Heat blob found at image coord: (%f, %f)", k.pt.x , k.pt.y);
     // ROS_DEBUG("Heat blob numer: %f", keypoints.size());
      cv::Rect rect_image(0, 0, threshold_image.cols, threshold_image.rows);
      cv::Rect rect_roi(k.pt.x - (k.size - 1)/2, k.pt.y - (k.size - 1)/2, k.size - 1, k.size - 1);

      //See http://stackoverflow.com/questions/29120231/how-to-verify-if-rect-is-inside-cvmat-in-opencv
      bool is_inside = (rect_roi & rect_image) == rect_roi;

      if (!is_inside){
        ROS_ERROR("ROI image would be partly outside image border, aborting plotting!");
        plot_=false;
       // continue;
      }

   victim_found=true;
   victim_loc[0]=k.pt.x;
   victim_loc[1]=k.pt.y;
  }

  if (keypoints.size() == 0) {
      std::cout << "No blob detected" << std::endl;
  }

  if((pub_detection_.getNumSubscribers() > 0) && victim_found ){

  //Create image with detection frames
      int width = 3;
      int height = 3;

      IplImage ipl_img = img_proc;

     //Display Keypoints
      for(unsigned int i = 0; i < keypoints.size(); i++){
          if (keypoints.at(i).size > 1){

            cv::Point point_;
            point_.x = keypoints.at(i).pt.x;
            point_.y = keypoints.at(i).pt.y;
            cv::Scalar black( 0, 0, 0 );
            cv::circle(img_proc,point_,2,black);
            std::cout << "Image center  is ..." << keypoints.at(i).pt << std::endl;

            if (!plot_) continue;


              //Write rectangle into image
              width = (int)(keypoints.at(i).size );
              height = (int)(keypoints.at(i).size );
              for(int j = -width; j <= width;j++){
                   if ((keypoints.at(i).pt.x + j) >= 0  &&  (keypoints.at(i).pt.x + j) < ipl_img.width){
                      //Draw upper line
                      if ((keypoints.at(i).pt.y - height) >= 0){
                           cvSet2D(&ipl_img,(int)(keypoints.at(i).pt.y - height), (int)(keypoints.at(i).pt.x + j),cv::Scalar(0));
                      }
                      //Draw lower line
                      if ((keypoints.at(i).pt.y + height) < ipl_img.height){
                           cvSet2D(&ipl_img,(int)(keypoints.at(i).pt.y + height), (int)(keypoints.at(i).pt.x + j),cv::Scalar(0));
                      }
                   }
              }

              for(int k = -height; k <= height;k++){
                  if ((keypoints.at(i).pt.y + k) >= 0  &&  (keypoints.at(i).pt.y + k) < ipl_img.height){
                      //Draw left line
                      if ((keypoints.at(i).pt.x - width) >= 0){
                           cvSet2D(&ipl_img,(int)(keypoints.at(i).pt.y +k), (int)(keypoints.at(i).pt.x - width),cv::Scalar(0));
                      }
                       //Draw right line
                      if ((keypoints.at(i).pt.x + width) < ipl_img.width){
                           cvSet2D(&ipl_img,(int)(keypoints.at(i).pt.y +k), (int)(keypoints.at(i).pt.x + width),cv::Scalar(0));
                      }
                  }
              }

          }
      }



      cv_bridge::CvImage cvImg;
      cvImg.image = img_proc;



      cvImg.header = input_image->header;
      cvImg.encoding = sensor_msgs::image_encodings::MONO8;
      pub_detection_.publish(cvImg.toImageMsg());
   }
}

  void victim_thermal_detector::performDetection(){
    BlobDetection();
  }


Status victim_thermal_detector::getDetectorStatus(){
  Status status;
  status.victim_found = victim_found;
  status.victim_loc=victim_loc;
  return status;
}

ros::Time victim_thermal_detector::getCaptureTime()
{
  return capture_ps.header.stamp;
}
