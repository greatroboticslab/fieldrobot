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
#include <chrono>
#include <thread>
#include <stdlib.h>


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
	
	void tellHeading();
	
	void tellGyro();
	
	bool pubSubReady();
	
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
	double cogCur;
	double gyroXCur;
	double gyroYCur;
	double gyroZCur;
	
	
	double accuracyRange = 0.628318531; // Equal to (2 * M_PI) * (1/10)

	std_msgs::String commands;
	std::stringstream cmnd_input;
	
	std::string buffer;
	std::fstream gpsFile;
	std::ofstream logFile;
};

/* (CALLBACK) This function updates yawData whenever the Jaguar updates 
its yaw data. */
void jaguarGps::imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo){
	yawCur = imuInfo->yaw; /* Updating yawData with the new Jaguar
	yaw data. */
	
	yawCur *= -1; // Invert the direction.
	yawCur += 3; // Add three to make it positive (0 to 6).
	yawCur *= (M_PI / 3); // Multiply by conversion factor.
	
	gyroXCur = imuInfo->gyro_x;
	gyroYCur = imuInfo->gyro_y;
	gyroZCur = imuInfo->gyro_z;
}

/* (CALLBACK) This function updates latiData and longiData whenever the 
Jaguar updates its latitude and longitude data. */
void jaguarGps::gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo){
	latiCur = gpsInfo->latitude; /* Updating latiCur with the new 
	Jaguar latitude data. */
	longiCur = gpsInfo->longitude; /* Updating longiCur with the 
	new Jaguar longitude data. */
	cogCur = gpsInfo->cog; /* Updating cogCur with the new Jaguar 
	course over ground data. */
}

void jaguarGps::followWaypoint(){
	gpsFile.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/waypoints.dat");
	assert(gpsFile);
	
	std::cout << std::fixed << std::setprecision(13);
	
	while (!gpsFile.eof()) {
		std::cout << "In loop\n";
		getline(gpsFile, buffer, ' ');
		latiDest = std::stod(buffer);
		getline(gpsFile, buffer);
		longiDest = std::stod(buffer);
		buffer.clear();
		
		std::cout << "Next waypoint: " << latiDest << " latitude, " << longiDest << " longitude." << std::endl;
		
		std::cout << "Travelling to next waypoint..." << std::endl;
		
		this->fixYaw();
		
		this->publishBuffer("MMW !M 400 -400");
		while ( (latiCur != latiDest) && (longiCur != longiDest)){
		
			std::cout << "Current coordinates: " << latiCur << " latitude, " << longiCur << " longitude." << std::endl;
			std::cout << "Destination coordinates: " << latiDest << " latitude, " << longiDest << " longitude." << std::endl;
		
			if (this->fixYaw()){
				std::cout << "Moving ahead again...\n";
				this->publishBuffer("MMW !M 400 -400");
			}
			
			//std::this_thread::sleep_for(std::chrono::seconds(3));
			
		}
		this->publishBuffer("MMW !M 0 0");
		
		std::cout << "Arrived at waypoint." << std::endl;
	}
	
	gpsFile.close();
}

void jaguarGps::calculateYaw(){
	ros::spinOnce(); // Updating positional data.
	std::cout << "\n";
	std::cout << "Current heading is: " << yawCur << std::endl;
	
	/*
	if (cogCur != yawCur){
		std::cout << "Warning: COG is off from heading.\n";
	}
	*/
	
	latiBuffer = latiDest - latiCur;
	longiBuffer = longiDest - longiCur;
	latiCur = 0;
	longiCur = 0;
		
	yawDest = atan( (longiBuffer/latiBuffer) );
	
	if (yawDest < 0) {
		yawDest += (2 * M_PI);
	} else if (yawDest > (2 * M_PI)){
		yawDest -= (2 * M_PI);
	}
	
	std::cout << "Calculated heading is: " << yawDest << std::endl;
	std::cout << "\n";
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
	double lowerBound = yawDest - accuracyRange;
	double upperBound = yawDest + accuracyRange;
	
	if (lowerBound < 0){
		lowerBound += (2 * M_PI);
	}
	
	if (upperBound > (2 * M_PI)){
		upperBound -= (2 * M_PI);
	}
	
	if ( (yawCur < lowerBound) || (yawCur > upperBound) ){
		std::cout << "\n";
		std::cout << "Correcting heading...\n";
		this->publishBuffer("MMW !M 0 0");
		
		while ( yawCur < yawDest ){
			std::cout << "Turning left to correct current yaw...\n";
			this->publishBuffer("MMW !M -400 -400");
			ros::spinOnce();
			
			std::cout << "Current heading is: " << yawCur << std::endl;
			std::cout << "Intended heading is: " << yawDest << std::endl;
		}
		
		while ( yawCur > yawDest ){
			std::cout << "Turning right to correct current yaw...\n";
			this->publishBuffer("MMW !M 400 400");
			ros::spinOnce();
			
			std::cout << "Current heading is: " << yawCur << std::endl;
			std::cout << "Intended heading is: " << yawDest << std::endl;
		}
					
		this->publishBuffer("MMW !M 0 0");
		
		std::cout << "Heading has been corrected. Corrected heading is: " << yawCur << std::endl;
		std::cout << "Intended heading is: " << yawDest << std::endl;
		std::cout << "\n";
		std::cout << "Current coordinates: " << latiCur << " latitude, " << longiCur << " longitude." << std::endl;
		std::cout << "Destination coordinates: " << latiDest << " latitude, " << longiDest << " longitude." << std::endl;
		std::this_thread::sleep_for(std::chrono::seconds(1));
		
		return true;
		
	} else { return false; }
}

bool jaguarGps::pubSubReady(){
	/* Checking if the drrobot_gps topic is ready with its 
	publisher. */
	if (gpsData.getNumPublishers() < 1){
		return false;
	}
	
	/* Checking if the drrobot_imu topic is ready with its 
	publisher. */
	if (imuData.getNumPublishers() < 1){
		return false;
	}
	
	/* Checking whether the main/drrobot_player node is ready. */
	if (pb.getNumSubscribers() < 1){
		return false;
	} else { return true; }
}

void jaguarGps::tellHeading(){
	std::cout << std::fixed << std::setprecision(13);

	while (this->pubSubReady()){
		ros::spinOnce();
		std::cout << yawCur << std::endl;
	
	}
}

void jaguarGps::tellGyro(){
	std::system("gnome-terminal -- sh -c \"tail -f ~/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/gyroData.log 2> /dev/null\"");
	
	while (this->pubSubReady()){
		logFile.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/gyroData.log");
		assert(logFile);
		
		logFile << std::fixed << std::setprecision(13);
		
		logFile << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
		
		ros::spinOnce();
		logFile << "Gyro X is: " << gyroXCur << "\nGyro Y is: " << gyroYCur << "\nGyro Z is: " << gyroZCur << std::endl;
		
		logFile.close();
	}
	
}

int main(int argc, char** argv){
	ros::init(argc,argv,"jaguar_gps", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	
	jaguarGps gpsRunner;
	
	while (!gpsRunner.pubSubReady());
	
	ros::Rate loopRate(50); /* Setting the ROS rate to 50 Hz like in 
	the main/drrobot_player node. */
	
	//gpsRunner.followWaypoint();
	//gpsRunner.tellHeading();
	gpsRunner.tellGyro();
	
	std::cout << "Jaguar has finished following waypoints.\n";
	return 0;
}
