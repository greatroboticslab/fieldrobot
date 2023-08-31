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
		motor_cmd_sub_ = nh.subscribe<std_msgs::String>("drrobot_motor_cmd", 1, boost::bind(&cmdImgSync::jagCmnd_cb this, _1));
		
		gpsData = nh.subscribe<jaguar4x4_2014::GPSInfo>("drrobot_gps", 1, boost::bind(&cmdImgSync::gps_cb, this, _1));
		
		imuData = nh.subscribe<jaguar4x4_2014::IMUData>("drrobot_imu", 1, boost::bind(&cmdImgSync::imu_cb, this, _1));
	}
	
	~cmdImgSync(){}
	
	void jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd);
	void gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo);
	void imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& gpsInfo);


private:
	ros::NodeHandle nh;
	ros::Subscriber motor_cmd_sub_; // Same name as in drrobot_player.cpp for clarity
	ros::Subscriber gpsData;
	ros::Subscriber imuData;
	
	double latitude;
	double longitude;
	
	jaguar4x4_2014::GPSInfo gpsBuffer;
	jaguar4x4_2014::IMUData imuBuffer;
};
