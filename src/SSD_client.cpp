#include <victim_localization/ssd_client.h>

SSD_client::SSD_client(){
  ros::NodeHandle n;
  loop_rate(10);
  client = n.serviceClient<victim_localization::DL_box>("SSD_Detection");
}

void SSD_client::Get_SSD_Detection(){
  Detection_success=false;
  while (ros::ok() && !Detection_success)
	{
	  std::stringstream ss;
    ss << "Start Detecting";
    srv.request.req = ss.str();

    if (client.call(srv)) {
      Detection_success=true;
    }

  std::cout << "detected class" << srv.response.Class.c_str() << std::endl;

	ros::spinOnce();
	//Setting the loop rate
	loop_rate.sleep();

	}
}

