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
#include <sys/stat.h> // This is required to verify the existence of certain files during processing.

class cmdImgSync{
public:
	cmdImgSync(){
		motor_cmd_sub_ = node_.subscribe<std_msgs::String>("drrobot_motor_cmd", 1, boost::bind(&cmdImgSync::jagCmnd_cb this, _1));
		
	gpsData = node_.subscribe<std_msgs::String>("drrobot_gps", 1, boost::bind(&cmdImgSync::gps_cb, this, _1));
	}
	
	void jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd);
	void gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo);

private:
	ros::NodeHandle nh;
	ros::Subscriber motor_cmd_sub_; // Same name as in drrobot_player.cpp for clarity
	ros::Subscriber gpsData;
	
	int curInt; // This is the current number (R to L, trailing zeroes for extra space on left side) that indicates the current timestamp
	string curIntStr; // This is curInt, translated over to a string to be placed within a file. 
	
	jaguar4x4_2014::GPSInfo gpsBuffer;
};
