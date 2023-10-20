#include <stdio.h> // Required to perform input/output operations.
#include <sstream> // Required for processing stringstream.
#include <boost/thread/thread.hpp> // Required for multithreading.
#include <ros/ros.h> // Required for ROS.
#include <std_msgs/String.h> // Required for ROS standard message handling and processing.
#include <fstream> // Required for file processing.
#include <cassert> // Required to check if file exists.
#include <iostream> // Required for standard in and out.
#include <jaguar4x4_2014/GPSInfo.h> // Required to handle and process GPS message data from the Jaguar robot.
#include <jaguar4x4_2014/IMUData.h> // Required to handle and process IMU message data from the Jaguar robot.
#include <cmath>


class jaguarGps{
public:
	jaguarGps(){
		pb = nh.advertise<std_msgs::String>("jaguar_gps", 1);

		// Subscribing to the Jaguar's GPS data topic.
		gpsData = nh.subscribe<jaguar4x4_2014::GPSInfo>("drrobot_gps", 1, boost::bind(&jaguarGps::gps_cb, this, _1));
		
		// Subscribing to the Jaguar's IMU data topic.
		imuData = nh.subscribe<jaguar4x4_2014::IMUData>("drrobot_imu", 1, boost::bind(&jaguarGps::imu_cb, this, _1));
	}
	
	~jaguarGps(){};

	/* gps_cb is the callback function for whenever the Jaguar's 
	GPS data topic updates. */
	void gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo);
	
	/* imu_cb is the callback function for whenever the Jaguar's 
	IMU data topic updates. */
	void imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo);
	
	void followWaypoint();
	
	void calculateYaw();
	
	void publishBuffer(std::string command);
	
	bool fixYaw();

private:
	ros::NodeHandle nh;
	ros::Publisher pb;
	ros::Subscriber gpsData; // Jaguar GPS data subscriber. 
	ros::Subscriber imuData; // Jaguar IMU data subscriber.

	double latiBuffer;
	double longiBuffer;
	double latiDest;
	double longiDest;
	double yawDest;
	double latiCur;
	double longiCur;
	double yawCur;

	std_msgs::String commands;
	std::stringstream cmnd_input;
	
	std::string buffer;
	std::fstream gpsFile;
};

/* (CALLBACK) This function updates yawData whenever the Jaguar updates 
its yaw data. */
void jaguarGps::imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo){
	yawCur = imuInfo->yaw; /* Updating yawData with the new Jaguar
	yaw data. */
}

/* (CALLBACK) This function updates latiData and longiData whenever the 
Jaguar updates its latitude and longitude data. */
void jaguarGps::gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo){
	latiCur = gpsInfo->latitude; /* Updating latiData with the new 
	Jaguar latitude data. */
	longiCur = gpsInfo->longitude; /* Updating longiData with the 
	new Jaguar longitude data. */
}

void jaguarGps::followWaypoint(){
	gpsFile.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/waypoints.dat");
	assert(gpsFile);
	
	while (!gpsFile.eof()) {
		getline(gpsFile, buffer, ' ');
		latiDest = std::stod(buffer);
		getline(gpsFile, buffer);
		longiDest = std::stod(buffer);
		buffer.clear();
		
		std::cout << "Next waypoint: " << latiDest << " latitude, " << longiDest << " longitude." << std::endl;
		
		std::cout << "Travelling to next waypoint..." << std::endl;
		
		this->fixYaw();
		
		this->publishBuffer("MMW !M 200 -200");
		while ( !(latiCur >= (latiDest - (latiDest * 0.1)) && latiCur <= (latiDest + (latiDest * 0.1))) || 
		!(longiCur >= (longiDest - (longiDest * 0.1)) && longiCur <= (longiDest + (longiDest * 0.1))) ){
			if (this->fixYaw()){
				this->publishBuffer("MMW !M 200 -200");
			}
		}
		this->publishBuffer("MMW !M 0 0");
		
		std::cout << "Arrived at waypoint." << std::endl;
	}
}

void jaguarGps::calculateYaw(){
	ros::spinOnce(); // Updating positional data.
	
	latiBuffer = latiDest - latiCur;
	longiBuffer = longiDest - longiCur;
	latiCur = 0;
	longiCur = 0;
		
	yawDest = atan( (longiBuffer/latiBuffer) );
}

void jaguarGps::publishBuffer(std::string command){
	buffer = command;
	cmnd_input << buffer;
	buffer.clear();
	commands.data = cmnd_input.str();
	cmnd_input.str("");
	cmnd_input.clear();
	pb.publish(commands);
}

bool jaguarGps::fixYaw(){
	this->calculateYaw();
	if ( !( yawCur >= (yawDest - (yawDest * 0.15)) && yawCur <= (yawDest + (yawDest * 0.15)) ) ){
		this->publishBuffer("MMW !M 0 0");
				
		while ( yawCur < (yawDest - (yawDest * 0.05)) ){
			this->publishBuffer("MMW !M -200 -200");
			ros::spinOnce();
		}
		
		while (  yawCur > (yawDest + (yawDest * 0.05))  ){
			this->publishBuffer("MMW !M 200 200");
			ros::spinOnce();
		}
				
		this->publishBuffer("MMW !M 0 0");
		
		return true;
	} else { return false; }
}

int main(int argc, char** argv){
	ros::Rate loopRate(50); /* Setting the ROS rate to 50 Hz like in 
	the main/drrobot_player node. */
	return 0;
}
