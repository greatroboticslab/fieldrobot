#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sstream>
#include <boost/thread/thread.hpp>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <fstream>
#include <cassert>
#include <iostream>
#include <stdlib.h> // This is required for the system() call.
#include <jaguar4x4_2014/GPSInfo.h> // This is required to process GPS data.
#include <jaguar4x4_2014/IMUData.h> // This is required to process IMU data.
#include <sys/stat.h> // This is required to verify the existence of certain files during processing.
#include <chrono> // This is required for the timestamp on each command/image.


class cmdImgSync{

public:
	cmdImgSync(){
		motor_cmd_sub_ = nh.subscribe<std_msgs::String>("drrobot_motor_cmd", 1, boost::bind(&cmdImgSync::jagCmnd_cb, this, _1));
		
		gpsData = nh.subscribe<jaguar4x4_2014::GPSInfo>("drrobot_gps", 1, boost::bind(&cmdImgSync::gps_cb, this, _1));
		
		imuData = nh.subscribe<jaguar4x4_2014::IMUData>("drrobot_imu", 1, boost::bind(&cmdImgSync::imu_cb, this, _1));
	}
	
	~cmdImgSync(){}
	
	void jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd);
	void gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo);
	void imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& gpsInfo);

	std::string timeConverter(std::chrono::milliseconds timeSinceEpoch)
	bool pubReady();
	
	
private:
	ros::NodeHandle nh;
	ros::Subscriber motor_cmd_sub_; // Same name as in drrobot_player.cpp for clarity
	ros::Subscriber gpsData;
	ros::Subscriber imuData;
	
	double latitude;
	double longitude;
	string curCmnd;
	string curTime;
	
	jaguar4x4_2014::GPSInfo gpsBuffer;
	jaguar4x4_2014::IMUData imuBuffer;
	
	std::chrono::milliseconds tse;
	int msEpoch;
	int secEpoch;
	int minEpoch;
	int hrEpoch;
};

// ------------------------------------------------------------

void jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd){
	tse = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
	curTime = this.timeConverter(tse);
	curCmnd = cmnd->data;
	
	std::system("source /opt/ros/noetic/setup.bash && source ~/.bashrc && cd ~/catkin_rbt_ws/ && source devel/setup.bash && rosservice call /image_saver/save");
}

bool cmdImgSync::pubReady(){
	if (motor_cmd_sub_.getNumPublishers() < 1){
		return false;
	} else { return true; }
	
	if (gpsData.getNumPublishers() < 1){
		return false;
	} else { return true; }
	
	if (imuData.getNumPublishers() < 1){
		return false;
	} else { return true; }
}

std::string cmdImgSync::timeConverter(std::chrono::milliseconds timeSinceEpoch){	
    	int msEpoch = timeSinceEpoch.count() % 1000;
    	int secEpoch = (timeSinceEpoch.count() / 1000) % 60;
    	int minEpoch = (timeSinceEpoch.count() / 60000 ) % 60;
    	int hrEpoch = (timeSinceEpoch.count() / 86400000) % 24;
    	
    	std::string timestamp = std::to_string(hrEpoch) + ":" + std::to_string(minEpoch) + ":" + std::to_string(secEpoch) + ":" + std::to_string(msEpoch);
    	
	return timestamp;
}

// ------------------------------------------------------------

int main(){
	// Initializing the node.
	ros::init(argc,argv,"cmdImg_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	
	cmdImgSync syncer;
	
	std::system("source /opt/ros/noetic/setup.bash && source ~/.bashrc && cd ~/catkin_rbt_ws/ && source devel/setup.bash && roslaunch axis_camera axis.launch hostname:=192.168.0.65:8081 username:=root password:=password encrypted:=true");
	std::system("source /opt/ros/noetic/setup.bash && source ~/.bashrc && cd ~/catkin_rbt_ws/ && source devel/setup.bash && rosrun image_view image_saver image:=/axis/image_raw _image_transport:=compressed _filename_format:=fubar%04i.jpg _save_all_image:=false __name:=image_saver");
	
	// Loop until the jaguar4x4_2014_node node responds
	while(!syncer.pubReady());
	
	ros::Rate loopRate(50);
	
	while(syncer.pubReady()){
		// Spinning the node once
		ros::spinOnce();
		
		loopRate.sleep();
	}
	
	return(0); // Confirming successful program execution. 
}
