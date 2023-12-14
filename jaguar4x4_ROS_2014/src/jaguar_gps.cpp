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
#include <cmath> // Required for the M_PI constant used for the conversion factor, as well as atan(), or inverse tangent.
#include <chrono> // Required to stop the program for certain periods of time.
#include <stdlib.h> // Required to stop the program for certain periods of time.
#include <thread> // Will be used later to allow debug functions to run while the main routines are running.

/* The jaguarGps class is responsible for handling a multitude of GPS and 
navigation related functions, such as the obvious GPS navigation, but 
also some debug navigation functions. */
class jaguarGps{
public:
	/* This is the constructor for the jaguarGps class. It 
	initializes the publisher to advertise to the topic 
	"jaguar_gps", subscribes to the Jaguar's GPS data topic, and 
	subscribes to the Jaguar's IMU data topic.*/
	jaguarGps(){
		pb = nh.advertise<std_msgs::String>("jaguar_gps", 1);

		// Subscribing to the Jaguar's GPS data topic.
		gpsData = nh.subscribe<jaguar4x4_2014::GPSInfo>("drrobot_gps", 1, boost::bind(&jaguarGps::gps_cb, this, _1));
		
		// Subscribing to the Jaguar's IMU data topic.
		imuData = nh.subscribe<jaguar4x4_2014::IMUData>("drrobot_imu", 1, boost::bind(&jaguarGps::imu_cb, this, _1));
	}
	
	~jaguarGps(){}; // Destructor for the jaguarGps class.

	/* gps_cb is the callback function for whenever the Jaguar's 
	GPS data topic updates. */
	void gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo);
	
	/* imu_cb is the callback function for whenever the Jaguar's 
	IMU data topic updates. */
	void imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo);
	
	/* followWaypoint is the function responsible for handling the 
	GPS navigation for the jaguar_gps node, travelling to each 
	waypoint listed in the waypoints.dat file. */
	void followWaypoint();
	
	/* calculateYaw is a helper function for followWaypoint, 
	calculating the yaw/heading it must have to be in the correct 
	direction to navigate to the next waypoint in the waypoints.dat 
	file. */
	void calculateYaw();
	
	/* publishBuffer is a helper function for followWaypoint, 
	publishing/sending the current movement command issued by the 
	followWaypoint function to the jaguar_gps topic, which is then 
	read by the main node/drrobot_player and performed. */
	void publishBuffer(std::string command);
	
	/* tellHeading is a debug function that prints out the current 
	heading of the Jaguar robot to the current console that the
	jaguar_gps node is currently running in. */
	void tellHeading();
	
	/* tellGyro is a debug function that writes to the gyroData.log 
	file, which allows the user to see the current gyro of the 
	Jaguar. */
	void tellGyro();
	
	/* pubSubReady checks whether the main/drrobot_player node is 
	actively subscribed to the jaguar_gps topic and checks whether 
	the drrobot_gps and drrobot_imu topics are publishing; 
	pubSubReady prevents the jaguar_gps node from completely 
	activating until these conditions are met. */
	bool pubSubReady();
	
	/* fixYaw is a helper function for followWaypoint, using 
	calculateYaw to first find the heading that the Jaguar must 
	turn to from its current yaw/heading, then attempting to turn 
	to that yaw/heading. */
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
	
	
	double accuracyRange = 0.628318531; // Equal to (2 * M_PI) * (1/10).

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
	
	gyroXCur = imuInfo->gyro_x; /* Updating gyroXCur with the new 
	Jaguar X gyro data. */
	gyroYCur = imuInfo->gyro_y; /* Updating gyroYCur with the new 
	Jaguar Y gyro data. */
	gyroZCur = imuInfo->gyro_z; /* Updating gyroZCur with the new 
	Jaguar Z gyro data. */
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

/* This function is effectively the main routine of the jaguar_gps node, 
as it handles all of the GPS navigation for the Jaguar robot, reading in 
each GPS coordinate from the waypoints.dat file and attempting to travel 
to each one. */
void jaguarGps::followWaypoint(){
	/* Opening the waypoints.dat file and performing assertion; this 
	will be used to travel to the listed/specified GPS coordinates.  */
	gpsFile.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/waypoints.dat");
	assert(gpsFile);
	
	/* Setting the precision of the decimal points to 13. */
	std::cout << std::fixed << std::setprecision(13);
	
	/* Until each GPS coordinate has been travelled to... */
	while (!gpsFile.eof()) {
		std::cout << "In loop\n"; // Debug statement.
		
		/* Reading in the latitude value of the current GPS 
		coordinate to be navigated to. */
		getline(gpsFile, buffer, ' ');
		latiDest = std::stod(buffer);
		
		/* Reading in the longitude value of the current GPS
		coordinate to be navigated to. */
		getline(gpsFile, buffer);
		longiDest = std::stod(buffer);
		
		buffer.clear(); // Clearing the buffer.
		
		/* Printing out the coordinates of the next waypoint 
		to be navigated to. */
		std::cout << "Next waypoint: " << latiDest << " latitude, " << longiDest << " longitude." << std::endl;
		
		// Debug statement.
		std::cout << "Travelling to next waypoint..." << std::endl;
		
		this->fixYaw(); /* Attempting to correct the current 
		yaw/heading. */
		
		// Moving forward once yaw/heading is corrected.
		this->publishBuffer("MMW !M 400 -400");
		
		/* While the current latitude position and the current 
		longitude position do not match their respective 
		destinations... */
		while ( (latiCur != latiDest) && (longiCur != longiDest)){
		
			/* Printing out the current coordinates of the 
			Jaguar robot, then printing out the destination 
			coordinates. */
			std::cout << "Current coordinates: " << latiCur << " latitude, " << longiCur << " longitude." << std::endl;
			std::cout << "Destination coordinates: " << latiDest << " latitude, " << longiDest << " longitude." << std::endl;
			
			/* If the yaw/heading becomes incorrect while 
			moving, attempt to correct it; once finished, 
			state that the Jaguar is moving ahead again and 
			send a "move forward" motor command. */
			if (this->fixYaw()){
				std::cout << "Moving ahead again...\n";
				this->publishBuffer("MMW !M 400 -400");
			}
			
			//std::this_thread::sleep_for(std::chrono::seconds(3));
			
		}
		
		// Stopping once it has arrived at the current waypoint.
		this->publishBuffer("MMW !M 0 0");
		
		// Debug statement.
		std::cout << "Arrived at waypoint." << std::endl;
	}
	
	gpsFile.close(); /* Closing file once all coordinates have been 
	navigated to. */
}

/* This function calculates the yaw/heading that the Jaguar must have 
in order to navigate to the next waypoint listed in the waypoints.dat 
file. */
void jaguarGps::calculateYaw(){
	ros::spinOnce(); // Updating positional data.
	
	// Debug statement.
	std::cout << "\nCurrent heading is: " << yawCur << std::endl;
	
	/*
	if (cogCur != yawCur){
		std::cout << "Warning: COG is off from heading.\n";
	}
	*/
	
	/* Setting latiBuffer and longiBuffer to the respective 
	latitude and longitude coordinates subtracted by the 
	current respective latitude and longitude coordinates. This is 
	done to treat the current coordinates as the origin. */
	latiBuffer = latiDest - latiCur;
	longiBuffer = longiDest - longiCur;
	//latiCur = 0;
	//longiCur = 0;
	
	/* Using the atan() function to calculate the inverse tangent 
	of longiBuffer/latiBuffer, which will give the yaw/heading in 
	which the Jaguar must turn to. */
	yawDest = atan( (longiBuffer/latiBuffer) );
	
	/* Translating heading if it happens to go out of bounds; if it 
	goes below 0 degrees, translate it back to a positive value; if 
	it goes above 360 degrees, translate it back to a value between 
	0 to 360 degrees. */
	if (yawDest < 0) {
		yawDest += (2 * M_PI);
	} else if (yawDest > (2 * M_PI)){
		yawDest -= (2 * M_PI);
	}
	
	// Debug statement.
	std::cout << "Calculated heading is: " << yawDest << "\n\n";
}

/* This function publishes any given motor command to the jaguar_gps 
topic to be read by the main/drrobot_player node. */
void jaguarGps::publishBuffer(std::string command){
	buffer = command; // Setting buffer to the inputted command.
	cmnd_input << buffer; // Translating buffer into a stringstream.
	buffer.clear(); // Clearing buffer, which is now in cmnd_input.
	commands.data = cmnd_input.str(); // Translating to ROS string.
	cmnd_input.str(""); // Clearing cmnd_input.
	cmnd_input.clear(); // Clearing cmnd_input.
	pb.publish(commands); // Publishing it to the jaguar_gps topic.
}

/* This function attempts to correct the yaw/heading of the Jaguar robot 
to properly turn it towards the current GPS waypoint/coordinate to be 
navigated to. Returns false if the current yaw/heading needs to be 
corrected; returns true if the current yaw/heading has been corrected. */
bool jaguarGps::fixYaw(){
	/* Calculating the current yaw/heading to be corrected, then 
	creating an upper and lower bound for it using accuracyRange. 
	This makes it to where the yaw/heading is corrected only when 
	it is no longer within this range, rather than when it 
	deviates even slightly from the yaw/heading it must adhere to. */
	this->calculateYaw();
	double lowerBound = yawDest - accuracyRange;
	double upperBound = yawDest + accuracyRange;
	
	/* If the lower bound is negative, translate it to a positive 
	value within the range of 0 to 360 degrees. */
	if (lowerBound < 0){
		lowerBound += (2 * M_PI);
	}
	
	/* If the upper bound is not within 0 to 360 degrees, translate 
	it to a value within that range. */
	if (upperBound > (2 * M_PI)){
		upperBound -= (2 * M_PI);
	}
	
	/* If the current yaw/heading has gone out of its current 
	upper range bound or its lower range bound... */
	if ( (yawCur < lowerBound) || (yawCur > upperBound) ){
		std::cout << "\nCorrecting heading...\n"; // Debug statement.
		
		// Sending a motor command to stop the robot.
		this->publishBuffer("MMW !M 0 0"); 
		
		/* While the current yaw/heading is less than the intended 
		yaw/heading... */
		while ( yawCur < yawDest ){
			// Debug statement.
			std::cout << "Turning left to correct current yaw...\n";
			/* Sending a turn left command, then updating 
			positional data. */
			this->publishBuffer("MMW !M -400 -400");
			ros::spinOnce();
			
			// Debug statements.
			std::cout << "Current heading is: " << yawCur << std::endl;
			std::cout << "Intended heading is: " << yawDest << std::endl;
		}
		
		/* While the current yaw/heading is greater than the 
		intended yaw/heading... */
		while ( yawCur > yawDest ){
			// Debug statement.
			std::cout << "Turning right to correct current yaw...\n";
			/* Sending a turn right command, then updating 
			positional data. */
			this->publishBuffer("MMW !M 400 400");
			ros::spinOnce();
			
			// Debug statements.
			std::cout << "Current heading is: " << yawCur << std::endl;
			std::cout << "Intended heading is: " << yawDest << std::endl;
		}
		
		// Sending a stop command to stop the robot.
		this->publishBuffer("MMW !M 0 0");
		
		// Debug statements...
		std::cout << "Heading has been corrected. Corrected heading is: " << yawCur << std::endl;
		std::cout << "Intended heading is: " << yawDest << std::endl;
		std::cout << "\n";
		std::cout << "Current coordinates: " << latiCur << " latitude, " << longiCur << " longitude." << std::endl;
		std::cout << "Destination coordinates: " << latiDest << " latitude, " << longiDest << " longitude." << std::endl;
		
		// Stopping the robot for 1 second before moving again.
		std::this_thread::sleep_for(std::chrono::seconds(1));
		
		return true;
		
	} else { return false; }
}

/* This function ensures that the publishers and subscribers for the 
jaguar_gps node are ready before the jaguar_gps node actually begins 
its main routines, returning false if they aren't ready. */
bool jaguarGps::pubSubReady(){
	/* Checking if the drrobot_gps topic is ready with its 
	publisher. If not, prevent the main routines from starting. */
	if (gpsData.getNumPublishers() < 1){
		return false;
	}
	
	/* Checking if the drrobot_imu topic is ready with its 
	publisher. If not, prevent the main routines from starting. */
	if (imuData.getNumPublishers() < 1){
		return false;
	}
	
	/* Checking whether the main/drrobot_player node is ready. 
	If not, prevent the main routines from starting. */
	if (pb.getNumSubscribers() < 1){
		return false;
	} else { return true; }
}

/* This is a debug function that prints out the current heading of the 
Jaguar robot to the current console that the jaguar_gps node is 
currently running in. */
void jaguarGps::tellHeading(){
	/* Setting the precision of the decimal points to 13. */
	std::cout << std::fixed << std::setprecision(13);
	
	/* While the subscribers and publishers for the jaguar_gps node 
	are still active, each run of the loop, call the callback 
	functions to update the current IMU and GPS information of the 
	Jaguar, then print out the current yaw/heading to the console 
	that the jaguar_gps node is currently running in. */
	while (this->pubSubReady()){
		ros::spinOnce();
		std::cout << yawCur << std::endl;
	
	}
}

/* This is a debug function that writes to the gyroData.log file, which 
allows the user to see the current gyro of the Jaguar. */
void jaguarGps::tellGyro(){
	/* Opens a terminal that prints the contents of gyroData.log to 
	a terminal whenever the file updates to allow the user to see 
	the current gyro data of the Jaguar. */
	std::system("gnome-terminal -- sh -c \"tail -f ~/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/gyroData.log 2> /dev/null\"");
	
	/* While the subscribers and publishers for the jaguar_gps node 
	are still active, repetitively open and close the gyroData.log 
	file; each time that the file is opened, rewrite the file, 
	updating it with the most current gyro information. Since the 
	terminal opened by the previous command is tailing the file, 
	the user will be able to see the current gyro data. */
	while (this->pubSubReady()){
		/* Opening gyroData.log and performing assertion. */
		logFile.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/gyroData.log");
		assert(logFile);
		
		/* Setting the precision of the decimal points to 13. */
		logFile << std::fixed << std::setprecision(13);
		
		/* Printing spacing for formatting. */
		logFile << "\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n";
		
		/* Calling the callback functions to update the current
		IMU and GPS information of the Jaguar, then writing 
		out the current X, Y, and Z gyro values to the log file. */
		ros::spinOnce();
		logFile << "Gyro X is: " << gyroXCur << "\nGyro Y is: " << gyroYCur << "\nGyro Z is: " << gyroZCur << std::endl;
		
		logFile.close(); // Closing the file.
	}
	
}

int main(int argc, char** argv){
	ros::init(argc,argv,"jaguar_gps", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	
	jaguarGps gpsRunner;
	
	while (!gpsRunner.pubSubReady());
	
	ros::Rate loopRate(50); /* Setting the ROS rate to 50 Hz like in 
	the main/drrobot_player node. */
	
	/* Comment out the functions you do not want running at the moment; will make this multithreaded in a later patch. */
	//gpsRunner.followWaypoint();
	//gpsRunner.tellHeading();
	gpsRunner.tellGyro();
	
	// Debug statement.
	std::cout << "Jaguar has finished following waypoints.\n";
	return 0;
}
