#include <iostream> // Required for standard in and out.
#include <stdio.h> // Required to perform input/output operations.
#include <stdlib.h> // Required for the system() call.
#include <sstream> // Required for processing stringstream.
#include <boost/thread/thread.hpp> // Required for multithreading.
#include <ros/ros.h> // Required for ROS.
#include <std_msgs/String.h> // Required for ROS standard message handling and processing.
#include <fstream> // Required for file processing.
#include <cassert> // Required to check if file exists.
#include <jaguar4x4_2014/GPSInfo.h> // Required to handle and process GPS message data from the Jaguar robot.
#include <jaguar4x4_2014/IMUData.h> // Required to handle and process IMU message data from the Jaguar robot.
#include <chrono> // Required for the timestamp on each command/image.


/* The preprocessor directories below all handle image processing and 
handling */
/* cv_bridge and opencv2 need to be installed manually. They are 
required to process the images from the Jaguar robot; cv_bridge 
is used specifically to convert between ROS and OpenCV images. */
#include <cv_bridge/cv_bridge.h> // https://github.com/ros-perception/vision_opencv.git
#include <opencv2/imgproc/imgproc.hpp> // https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html or https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h> // Required for ROS standard message handling and processing.
#include <image_transport/image_transport.h> // https://github.com/ros-perception/image_common.git

/* There is no namespace being used here. Please refrain from doing, 
"using namespace [your_namepspace_here];", as it makes the code much 
harder to maintain in the future. */

/* READ ME FIRST BEFORE OPERATING NODE
For the system() methods to operate properly, assuming you're running on 
Ubuntu 20.04 (which you should be for ROS1 Noetic to work properly), you 
need to reconfigure your shell - which is currently configured to 
/bin/dash - to /bin/bash, as you will otherwise get a message such as 
this: 

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
to distinguish when one record ends and when another begins in the same, 
file, when applicable.
*/

/* If you have any questions regarding this node, please email Kevin 
Kongmanychanh at kck3m@mtmail.mtsu.edu. */


// -----------------------------------------------------------------------


/* The cmdImgSynce class is responsible for performing all data 
handling and processing. */
class cmdImgSync{

public:
	/* Class constructor for cmdImgSync. The "ih(nh)" seen here is 
	an initialization list, used to initialize the ImageTransport 
	object (ih, standing for Image Handler) with the NodeHandle 
	object (nh, standing for Node Handler) */
	cmdImgSync() : ih(nh){
		
		// Subscribing to the Jaguar's motor commands topic.
		motor_cmd_sub_ = nh.subscribe<std_msgs::String>("drrobot_motor_cmd", 1, boost::bind(&cmdImgSync::jagCmnd_cb, this, _1));
		
		// Subscribing to the Jaguar's GPS data topic.
		gpsData = nh.subscribe<jaguar4x4_2014::GPSInfo>("drrobot_gps", 1, boost::bind(&cmdImgSync::gps_cb, this, _1));
		
		// Subscribing to the Jaguar's IMU data topic.
		imuData = nh.subscribe<jaguar4x4_2014::IMUData>("drrobot_imu", 1, boost::bind(&cmdImgSync::imu_cb, this, _1));
		
		/* Subscribing to the Jaguar's camera topic, provided 
		by the axis_camera package and its constituents. */
		imgData = ih.subscribe("axis/image_raw", 1, boost::bind(&cmdImgSync::img_cb, this, _1));
	}
	
	~cmdImgSync(){} // Class destructor for cmdImgSync
	
	/* Declaring function prototypes below... */
	/* If you do not understand ROS callback functions, check here
	for more information http://wiki.ros.org/roscpp/Overview/ */
	
	/* jagCmnd_cb is the callback function for whenever the 
	Jaguar's motor command topic updates. */
	void jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd);
	
	/* gps_cb is the callback function for whenever the Jaguar's 
	GPS data topic updates. */
	void gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo);
	
	/* imu_cb is the callback function for whenever the Jaguar's 
	IMU data topic updates. */
	void imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo);
	
	/* img_cb is the callback function for whenever the Jaguar's 
	camera posts a new image on the axis/image_raw topic. */
	void img_cb(const sensor_msgs::ImageConstPtr& img);
	
	/* updateData writes any new command/GPS/IMU data recieved 
	from the callback functions to the jaguarRecord.dat and 
	commandRecord.dat files. */
	void updateData();
	
	/* timeConverter converts the current time since epoch to 
	a string timestamp that can be easily understood. */
	std::string timeConverter(std::chrono::milliseconds timeSinceEpoch);
	
	/* pubReady checks whether all the ROS topics that the 
	subscribers are subscribed to have active publishers 
	publishing to them; pubReady prevents the cmdImg_node from 
	completely activating until the drrobot_player node is 
	ready. */
	bool pubReady();
	
	
private:
	ros::NodeHandle nh; // NodeHandle to initialize ROS subscribers.
	image_transport::ImageTransport ih; /* ImageTransport to 
	initialize the image_transport subscriber. */
	ros::Subscriber motor_cmd_sub_; /* Jaguar motor command 
	subscriber. Same name as in drrobot_player.cpp for clarity */
	ros::Subscriber gpsData; // Jaguar GPS data subscriber. 
	ros::Subscriber imuData; // Jaguar IMU data subscriber.
	image_transport::Subscriber imgData; /* Jaguar camera image 
	subscriber */
	
	double latiData; // Variable to hold latitude data.
	double longiData; // Variable to hold longitude data.
	double yawData; // Variable to hold yaw data.

	std::string curCmnd = "NO_CHANGE"; /* Variable to hold the 
	current Jaguar command. */
	bool cmndUpdated = false; /* Variable to indicate whether a 
	new command has been recieved or not. If true, indicates that 
	a new command (different from the last) has came in. */
	std::string curTime; /* Variable to hold the current time 
	calculated from the timeConverter function */

	int counter = 0; /* Variable that indicates the current command 
	and image pair in the commandRecord.dat and commandImage.dat 
	files. */
	
	/* File stream variables for each respective .dat file, used 
	to append to each respective .dat file with new data. */
	std::fstream jaguarRecord;
	std::fstream commandRecord;
	std::fstream commandImage;
	
	// These are all used for the timeConverter.
	std::chrono::milliseconds tse; // Time since epoch in ms.
	int msEpoch; /* The current millisecond from the tse. */
	int secEpoch; /* The current second from the tse. */
	int minEpoch; /* The current minute from the tse. */
	int hrEpoch; /* The current hour from the tse. */
};


// -----------------------------------------------------------------------


/* (CALLBACK) This function updates curCmnd and cmndUpdated whenever 
a new Jaguar command is recieved. */
void cmdImgSync::jagCmnd_cb(const std_msgs::String::ConstPtr& cmnd){
	curCmnd = cmnd->data; /* Updating curCmnd with the new Jaguar 
	command. */
	cmndUpdated = true; /* Setting cmndUpdated to true, indicating 
	a new command has been recieved. */
}

/* (CALLBACK) This function updates latiData and longiData whenever the 
Jaguar updates its latitude and longitude data. */
void cmdImgSync::gps_cb(const jaguar4x4_2014::GPSInfo::ConstPtr& gpsInfo){
	latiData = gpsInfo->latitude; /* Updating latiData with the new 
	Jaguar latitude data. */
	longiData = gpsInfo->longitude; /* Updating longiData with the 
	new Jaguar longitude data. */
}

/* (CALLBACK) This function updates yawData whenever the Jaguar updates 
its yaw data. */
void cmdImgSync::imu_cb(const jaguar4x4_2014::IMUData::ConstPtr& imuInfo){
	yawData = imuInfo->yaw; /* Updating yawData with the new Jaguar
	yaw data. */
}

/* (CALLBACK) This function saves any new images that the Jaguar's 
camera collects to the specified file destination (fileDest) It 
handles all of the image handling and processing. */
void cmdImgSync::img_cb(const sensor_msgs::ImageConstPtr& img){
	// Opening commandImage.dat to append mode to add new data.
	commandImage.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/commandImage.dat", std::ios::app);
	
	assert(commandImage); /* Asserting commandImage to ensure file
	exists */
	
	// Setting tse to the current time since epoch.
	tse = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
	
	curTime = this->timeConverter(tse); /* Creating a timestamp, 
	using timeConverter to convert tse. */
	
	/* Setting the file destination, using curTime as the name of 
	the image file. */
	std::string fileDest = "/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/camera_images/" + curTime + ".png";
		
	cv_bridge::CvImagePtr cv_image; /* Creating a CVImagePtr to 
	hold the sensor_msgs::Image message when it is converted into 
	an OpenCV image. */
	
	cv_image = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);/* Converting a 
	sensor_msgs::Image message into an OpenCV image. */
	
	cv::imwrite(fileDest, cv_image->image); /* Saving the image to 
	the file destination specified in fileDest. */
	
	commandImage << std::setfill(' ') << std::left; /* Setting 
	the format specifiers in the file to be lefmost bound and to 
	have padding set to be whitespace (spaces). */
	
	/* Use this one below instead of the if-else statement if you
	are planning to use the commandImage.dat file for the 
	replay_node; keep in mind that this will be slightly less 
	accurate, as it is recording at 30 Hz (due to the 30 FPS 
	camera). Using the if-else statement for the replay may result 
	in missing motor commands. */
	// commandImage << std::setw(100) << curTime + ".png" << counter << "|" << curCmnd << std::endl;
	
	/* If there is a new command whenever a new image is recieved 
	by the Jaguar's camera, save the file destination to 
	commandImage.dat with the current counter along with the 
	current command, with the right of the file destination padded 
	to a maximum of 100 spaces from the start of the line.  */
	if (cmndUpdated){ 
		commandImage << std::setw(100) << fileDest << counter << "|" << curCmnd << std::endl;
		
	/* If not, replace the current command/curCmnd with NO_CHANGE 
	to indicate that no new command came in when the image was 
	saved. */
	} else {
		commandImage << std::setw(100) << fileDest << counter << "|" << "NO_CHANGE" << std::endl;
	}
	
	commandImage.close(); // Closing commandImage.dat.
}

/* This function ensures that all of the publishers that the 
cmdImg_node is subscribed to are ready by the time that the cmdImg_node 
begins its normal routines, returning false if any of the publishers 
are not ready and true if they are all ready. */
bool cmdImgSync::pubReady(){
	/* Checking if the drrobot_motor_cmd topic is ready with its 
	publisher. */
	if (motor_cmd_sub_.getNumPublishers() < 1){
		return false;
	}
	
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
	
	/* Checking if the axis/image_raw topic is ready with its 
	publisher. */
	if (imgData.getNumPublishers() < 1){
		return false;
	}
	
	return true; // Returning true if all topic publishers are ready.
}

/* This function converts the current time since epoch to a timestamp 
with the format of HOUR:MIN:SECOND:MILLISECOND, returning that 
timestamp as a string. This takes the current time since epoch in 
milliseconds as input. */
std::string cmdImgSync::timeConverter(std::chrono::milliseconds timeSinceEpoch){	
	/* Calculating the current timestamp given the time since 
	epoch, with each unit of time calculated separately. */
    	int msEpoch = timeSinceEpoch.count() % 1000;
    	int secEpoch = (timeSinceEpoch.count() / 1000) % 60;
    	int minEpoch = (timeSinceEpoch.count() / 60000 ) % 60;
    	int hrEpoch = (timeSinceEpoch.count() / 86400000) % 24;
    	
    	/* Concatenating all of the units of time together to form the 
    	timestamp. */
    	std::string timestamp = std::to_string(hrEpoch) + ":" + std::to_string(minEpoch) + ":" + std::to_string(secEpoch) + ":" + std::to_string(msEpoch);
    	
	return timestamp; // Returning the timestamp.
}

/* This function updates the jaguarRecord.dat file and the 
commandRecord.dat file every time before the callback functions are 
called through ros::spinOnce, along with updating any associated 
variables related to this process. */
void cmdImgSync::updateData(){
	/* Opening jaguarRecord.dat and commandRecord.dat to append 
	new data. */
	jaguarRecord.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/jaguarRecord.dat", std::ios::app);
	commandRecord.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/commandRecord.dat", std::ios::app);
	
	/* Asserting jaguarRecord and commandRecord to ensure that both 
	files exist. */
	assert(jaguarRecord);
	assert(commandRecord);
	
	/* Setting tse to the current time since epoch, then passing 
	tse through timeConverter to convert it to a timestamp; 
	storing the timestamp in curTime. */
	tse = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now().time_since_epoch());
	curTime = this->timeConverter(tse);

	commandRecord << std::setfill(' ') << std::left; /* Setting 
	the format specifiers in commandRecord.dat to be lefmost bound 
	and to have padding set to be whitespace (spaces). */

	/* If there is a new command, save that command to 
	jaguarRecord.dat, adding a comma and space after it. To 
	commandRecord.dat, first save curTime to the file, with the 
	right of curTime padded with a maximum of 18 spaces from the 
	start of the line, then save the current counter alongside 
	the current command. Update cmndUpdated to false.*/
	if (cmndUpdated){
		jaguarRecord << curCmnd << ", ";
		commandRecord << std::setw(18) << curTime << counter << "|" << curCmnd << std::endl;
		cmndUpdated = false;
		
	/* If not, state in jaguarRecord that there is no new command. 
	In commandRecord, do the same as if there was a new command, 
	but set where curCmnd typically is to NO_CHANGE. */
	} else {
		jaguarRecord << "No current command, ";
		commandRecord << std::setw(18) << curTime << counter << "|" << "NO_CHANGE" << std::endl;
	}
	
	/* Setting the format specifiers of jaguarRecord to have a 
	fixed precision, set to 13 decimal points. This is to keep the 
	GPS and IMU data accurate. */
	jaguarRecord << std::fixed << std::setprecision(13);
	
	/* Recording the yaw data, the latidute data, and the longitude 
	data to the jaguarRecord.dat file afte the current listed 
	command, separated by a space and a comma for each entry. */
	jaguarRecord << yawData << ", " << latiData << ", " << longiData << ", ";
	
	jaguarRecord << 0 << std::endl; /* This line is simply for the 
	altitude, which the robot does not record, but is required 
	for the GUI program. */

	counter++; // Updating counter.
	
	// Closing jaguarRecord.dat and commandRecord.dat.
	jaguarRecord.close();
	commandRecord.close();
}


// -----------------------------------------------------------------------


int main(int argc, char** argv){
	// Initializing the node.
	ros::init(argc,argv,"cmdImg_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);

	cmdImgSync syncer; // Declaring cmdImgSync object for processing.
	boost::thread t1; // Declaring boost thread for multithreading.
	
	/* Making two system calls; one to access the camera of the 
	Jaguar robot, the other to republish the compressed images of 
	the camera to another topic to make them accessible. Also 
	decompresses them for processing. */
	std::system("gnome-terminal -- sh -c \"source /opt/ros/noetic/setup.bash && source ~/.bashrc && cd ~/catkin_rbt_ws/ && source devel/setup.bash && roslaunch axis_camera axis.launch hostname:=192.168.0.65:8081 username:=root password:=drrobot encrypted:=true\"");
	std::system("gnome-terminal -- sh -c \"echo 'Do NOT close this terminal. It republishes the compressed images from /axis/image_raw/compressed to /axis/image_raw and decompresses them in the process.' && rosrun image_transport republish compressed in:=/axis/image_raw raw out:=/axis/image_raw\"");
	
	while(!syncer.pubReady()); // Loop until the jaguar4x4_2014_node node responds
	
	ros::Rate loopRate(50); /* Setting the ROS rate to 50 Hz like in 
	the main/drrobot_player node. */
	
	/* While the publishers for every subscribed topic are still 
	active, or until the user [CTRL + C]'s... */
	while(syncer.pubReady()){
		// Initialize thread for updateData call on syncer object.
		t1 = boost::thread(boost::bind(&cmdImgSync::updateData, &syncer));
		t1.join(); // Wait until the thread is finished.

		
		// Spinning the node once.
		ros::spinOnce();
		loopRate.sleep();
	}
	
	std::cout << "Stopped running.\n"; // Once stopped, declare to user that it has stopped running.
	
	return(0); // Confirming successful program execution. 
}
