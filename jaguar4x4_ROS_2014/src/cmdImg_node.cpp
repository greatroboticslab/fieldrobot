#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/poll.h>
#include <sstream> // Required for processing stringstream
#include <boost/thread/thread.hpp> // Required for multithreading
#include <ros/ros.h> // Required for ros
#include <std_msgs/String.h> // Required for ros
#include <fstream> // Required for file processing.
#include <cassert> // Required to check if file exists.
#include <iostream> // Required for standard in and out
#include <stdlib.h> // This is required for the system() call.
#include <jaguar4x4_2014/GPSInfo.h> // This is required to process GPS data.
#include <jaguar4x4_2014/IMUData.h> // This is required to process IMU data.
#include <chrono> // This is required for the timestamp on each command/image.


// This all handles image processing.
// cv_bridge and opencv2 need to be installed manually
#include <cv_bridge/cv_bridge.h> // https://github.com/ros-perception/vision_opencv.git
#include <opencv2/imgproc/imgproc.hpp> // https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html or https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/ (MAKE SURE TO UPDATE DIRECTORY IN CMAKELISTS.TXT. WILL SAVE YOU A LOT OF TIME.)
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h> // https://github.com/ros-perception/image_common.git


/* READ ME FIRST BEFORE OPERATING NODE
For the system() methods to operate properly, assuming you're running on 
Ubuntu 20.04 (which you should be for ROS1 Noetic to work properly), you need to reconfigure your shell - which is currently configured to 
/bin/dash - to /bin/bash, as you will otherwise get a message such as this: 

"sh: 1: source: not found"

As well as all the system() methods not functioning.

To fix this, open up a new terminal window and enter in:

"sudo dpkg-reconfigure dash"

Enter in your password as normal.

This will then ask whether you want dash to be your default system shell; 
select "No", and then bash will be configured as your shell (or, /bin/sh
will be pointing to /bin/bash).


Keep in mind that the jaguarRecord.dat, commandImage.dat, and 
commandRead.dat files should be emptied before running, unless you want 
older data mixed in with the new data. The counter should make it easier 
to distinguish when one record ends and when another begins, when 
applicable.
*/

class cmdImgSync{

public:
	cmdImgSync() : ih(nh){
	
		motor_cmd_sub_ = nh.subscribe<std_msgs::String>("drrobot_motor_cmd", 1, boost::bind(&cmdImgSync::jagCmnd_cb, this, _1));
		
		gpsData = nh.subscribe<jaguar4x4_2014::GPSInfo>("drrobot_gps", 1, boost::bind(&cmdImgSync::gps_cb, this, _1));
		
		imuData = nh.subscribe<jaguar4x4_2014::IMUData>("drrobot_imu", 1, boost::bind(&cmdImgSync::imu_cb, this, _1));
		
		imgData = ih.subscribe("axis/image_raw", 1, boost::bind(&cmdImgSync::img_cb, this, _1));
	}
	
	~cmdImgSync(){}
	
	void jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd);
	void gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo);
	void imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo);
	void img_cb(const sensor_msgs::ImageConstPtr& img);
	
	void updateData();

	std::string timeConverter(std::chrono::milliseconds timeSinceEpoch);
	bool pubReady();
	
	
private:
	ros::NodeHandle nh;
	image_transport::ImageTransport ih;
	ros::Subscriber motor_cmd_sub_; // Same name as in drrobot_player.cpp for clarity
	ros::Subscriber gpsData;
	ros::Subscriber imuData;
	image_transport::Subscriber imgData;
	
	double latiData;
	double longiData;
	double yawData;

	std::string curCmnd = "N/A";
	bool cmndUpdated = false;
	std::string curTime;

	int counter = 0;
	
	std::fstream jaguarRecord;
	std::fstream commandRecord;
	std::fstream commandImage;
	
	std::chrono::milliseconds tse;
	int msEpoch;
	int secEpoch;
	int minEpoch;
	int hrEpoch;
};

// ------------------------------------------------------------

void cmdImgSync::jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd){
	curCmnd = cmnd->data;
	cmndUpdated = true;
}

void cmdImgSync::gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo){
	latiData = gpsInfo->latitude;
	longiData = gpsInfo->longitude;
}

void cmdImgSync::imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo){
	yawData = imuInfo->yaw;
}

void cmdImgSync::img_cb(const sensor_msgs::ImageConstPtr& img){
	commandImage.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/commandImage.dat", std::ios::app);
	
	assert(commandImage);
	
	tse = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
	curTime = this->timeConverter(tse);
	
	std::string fileDest = "/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/camera_images/" + curTime + ".png";
		
	// Converting a sensor_msgs::Image message into a CVImage
	cv_bridge::CvImagePtr cv_image;
	
	cv_image = cv_bridge::toCvCopy(img);
	
	cv::imwrite(fileDest, cv_image->image);
	
	commandImage << std::setfill(' ') << std::left;
	
	/* Use this one below instead of the if-else statement if you
	are planning to use the commandImage.dat file for the 
	replay_node; keep in mind that this will be slightly less 
	accurate, as it is recording at 30 Hz (due to the 30 FPS 
	camera). Using the if-else statement for the replay may result 
	in missing motor commands. */
	// commandImage << std::setw(18) << curTime + ".png" << counter << "|" << curCmnd << std::endl;
	
	if (cmndUpdated){
		commandImage << std::setw(18) << fileDest << counter << "|" << curCmnd << std::endl;
	} else {
		commandImage << std::setw(18) << fileDest << counter << "|" << "NO_CHANGE" << std::endl;
	}
	
	commandImage.close();
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
	
	if (imgData.getNumPublishers() < 1){
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

void cmdImgSync::updateData(){
	jaguarRecord.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/jaguarRecord.dat", std::ios::app);
	commandRecord.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/commandRecord.dat", std::ios::app);

	assert(jaguarRecord);
	assert(commandRecord);
	
	tse = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
	curTime = this->timeConverter(tse);

	commandRecord << std::setfill(' ') << std::left;

	if (cmndUpdated){
		jaguarRecord << curCmnd << ", ";
		commandRecord << std::setw(18) << curTime << counter << "|" << curCmnd << std::endl;
		cmndUpdated = false;
	} else {
		jaguarRecord << "No current command, ";
		commandRecord << std::setw(18) << curTime << counter << "|" << "NO_CHANGE" << std::endl;
	}

	jaguarRecord << std::fixed << std::setprecision(13); // This is set to 13 decimal points of precision, after the decimal point, to keep the GPS data accurate.
	jaguarRecord << yawData << ", " << latiData << ", " << longiData << ", "; 
	jaguarRecord << 0 << std::endl; // This line is simply for the altitude, which the robot does not record, but is required for the GUI program.

	counter++;
	
	jaguarRecord.close();
	commandRecord.close();
}

// ------------------------------------------------------------

int main(int argc, char** argv){
	// Initializing the node.
	ros::init(argc,argv,"cmdImg_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

	cmdImgSync syncer; // Declaring cmdImgSync object for processing
	boost::thread t1; // Declaring boost thread for multithreading
	
	/* Making two system calls; one to access the camera of the 
	Jaguar robot, the other to republish the compressed images of 
	the camera to another topic to make them accessible. Also 
	decompresses them for processing. */
	std::system("gnome-terminal -- sh -c \"source /opt/ros/noetic/setup.bash && source ~/.bashrc && cd ~/catkin_rbt_ws/ && source devel/setup.bash && roslaunch axis_camera axis.launch hostname:=192.168.0.65:8081 username:=root password:=drrobot encrypted:=true\"");
	std::system("gnome-terminal -- sh -c \"echo 'Do NOT close this terminal. It republishes the compressed images from /axis/image_raw/compressed to /axis/image_raw and decompresses them in the process.' && rosrun image_transport republish compressed in:=/axis/image_raw raw out:=/axis/image_raw\"");
	
	while(!syncer.pubReady()); // Loop until the jaguar4x4_2014_node node responds
	
	ros::Rate loopRate(50); /* Setting the ROS rate to 50 Hz like 
	main/drrobot_player node. */
	
	/* While the publishers for every subscribed topic are still 
	active, or until the user [CTRL + C]'s... */
	while(syncer.pubReady()){
		t1 = boost::thread(boost::bind(&cmdImgSync::updateData, &syncer)); // Initialize thread for updateData call on syncer object.
		t1.join(); // Wait until the thread is finished

		
		// Spinning the node once
		ros::spinOnce();
		loopRate.sleep();
	}
	
	std::cout << "Stopped running.\n"; // Once stopped, declare to user that it has stopped running.
	
	return(0); // Confirming successful program execution. 
}
