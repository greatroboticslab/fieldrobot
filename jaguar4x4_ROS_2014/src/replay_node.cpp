#include <stdio.h> // Required to perform input/output operations.
#include <sstream> // Required for processing stringstream.
#include <boost/thread/thread.hpp> // Required for multithreading.
#include <ros/ros.h> // Required for ROS.
#include <std_msgs/String.h> // Required for ROS standard message handling and processing.
#include <fstream> // Required for file processing.
#include <cassert> // Required to check if file exists.
#include <iostream> // Required for standard in and out.

/* There is no namespace being used here. Please refrain from doing, 
"using namespace [your_namepspace_here];", as it makes the code much 
harder to maintain in the future. */

/* If you have any questions regarding this node, please email Kevin 
Kongmanychanh at kck3m@mtmail.mtsu.edu. */


// -----------------------------------------------------------------------


/* The ReplayNode class is responsible for handling the replaying of data 
files containing commands meant to control the robot, either recorded 
or manually inputted into the file. */
class ReplayNode{
	public:
		/* This is the constructor for the ReplayNode class. 
		It simply initializes the publisher to advertise to the 
		topic "rep_cmnd." */
		ReplayNode(){
			pb = nh.advertise<std_msgs::String>("rep_cmnd", 1);
			ros::NodeHandle privPart("~");
		}
		
		~ReplayNode(){} // Destructor for ReplayNode class.
		
		/* subReady checks whether the main/drrobot_player node 
		is actively subscribed to the rep_cmnd topic; subReady 
		prevents the replay_node from completely activating 
		until the drrobot_player node is ready. */
		bool subReady();
		
		/* replay_file replays and publishes the data to the 
		rep_cmnd topic for the jaguar4x4_2014_node/drrobot_player 
		node to subscribe to (see drrobot_player.cpp). */
		void replay_file();
		
		/* finished returns endFile, indicating whether the EOF 
		has been reached or not. */
		bool finished();
		
		
	private:
		/* Declaring the NodeHandle (nh) and the Publisher (pb) 
		these two variables are responsible for handling the 
		publishing of the data read in through the .dat or .txt 
		file fed into this program. */
		ros::NodeHandle nh;
		ros::Publisher pb;
		
		/* Declaring msg (commands) of String type and 
		stringstream (cmnd_input). These handle/hold the commands 
		read in by the replay_file() function to be published. */
		std_msgs::String commands;
		std::stringstream cmnd_input;
		
		/* Declaring String (buffer) to handle in any raw data 
		coming in from the file. If the data read in the file is 
		a command (one that is meant to be read), it will be put 
		into cmnd_input (and subsequently put into commands.data) 
		to be published. */
		std::string buffer = "";
		std::string prevBuf = buffer;
		
		int lastPos = 0; /* Variable to hold the latest/last known 
		position in the file that must be read. */
		int curPos = 0; /* Variable to hold the current line in 
		the file, used to traverse the file until lastPos is 
		located. */
		
		bool endFile = false; /* Variable to indicate when EOF is 
		hit */
		
		/* This is the fstream object that will be used to open 
		the file for processing. */
		std::fstream cmnd_file;
};


// -----------------------------------------------------------------------


/* This function ensures that the main/drrobot_player node is ready to 
recieve data from the replay_node before the replay_node begins its 
normal routines, returning false if the main/drrobot_player node is not 
ready and true if it is ready. */
bool ReplayNode::subReady(){
	/* Checking whether the main/drrobot_player node is ready. */
	if (pb.getNumSubscribers() < 1){
		return false;
	} else { return true; }
}

/* This function returns the endFile variable to indicate to an object 
of the ReplayNode class that the EOF has been reached. */
bool ReplayNode::finished(){
	return endFile;
}

/* This function handles all the file reading and processing for the file 
that contains commands intended to be read to the Jaguar robot. It takes 
no arguments, but recieves a file as input and publishes the valid 
commands it finds in that file to the robot. */
void ReplayNode::replay_file(){
	/* Given that the main/drrobot_player node is subscribed to the 
	rep_cmnd topic and that the EOF has not been reached... */
	if ( (pb.getNumSubscribers() >= 1) && !endFile ){
		/* Open the commandRead.dat file, then perform an 
		assertion to ensure that it exists. */
		cmnd_file.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/commandRead.dat", std::ios::in);
		assert(cmnd_file);
		
		/* While the current position in the file is not the 
		last known position that must be read, and given that the 
		EOF has not been reached, traverse each line until the 
		latest position that must be read as indicated by lastPos 
		is found. */
		while ( (curPos != lastPos) && !cmnd_file.eof() ){
			cmnd_file.ignore(100, '\n'); /* Ignoring the 
			lines that are not the line of the last 
			position. */
			curPos++; // Incrementing curPos.
		}
		
		/* Once the last known position that must be read is 
		found, read that line's command (ignore anything 
		preceding the '|' character) and save it to buffer. */
		cmnd_file.ignore(100, '|');
		getline(cmnd_file, buffer);
		
		/* If the buffer indicates that there has been a change 
		in commands, set the value of cmnd_input to whatever 
		value is in buffer, set the data of commands to what was 
		just in cmnd_input, then clear cmnd_input. Once this is 
		completed, publish the commands to the topic to be read 
		by the main/drrobot_player/jaguar4x4_2014_node node. */
		if (buffer != "NO_CHANGE"){
			prevBuf = buffer;
			cmnd_input << buffer;
			
			commands.data = cmnd_input.str();
			cmnd_input.str("");
			cmnd_input.clear();
			
			pb.publish(commands);
		}
		
		buffer.clear(); // Clearing buffer.
		
		cmnd_file.close(); /* Closing the file once processing is 
		done. */ 
		
		/* If there are still lines to be read in the file, 
		reset curPos to 0 and increment lastPos. */
		if (!cmnd_file.eof()){
			curPos = 0;
			lastPos++;
			
		// If not, set endFile to true to indicate EOF.
		} else { endFile = true; }
	}
}


// -----------------------------------------------------------------------


int main(int argc, char** argv){	
	// Initializing the node.
	ros::init(argc,argv,"replay_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	
	/* Declaring ReplayNode object replayer to handle processing the 
	data. */
	ReplayNode replayer;
	
	/* Loop until the jaguar4x4_2014_node/drrobot_player node 
	responds. */
	while(!replayer.subReady());
	
	ros::Rate loopRate(50); /* Setting the ROS rate to 50 Hz like in 
	the main/drrobot_player node. */
	
	/* While the main/drrobot_player node is still active and 
	subscribed to the rep_cmnd topic, given that the EOF has 
	not been reached in the file, and until the user [CTRL + C]'s 
	... */
	while(replayer.subReady() && !replayer.finished()){
		/* Initializing a thread object (rep_thread), then calling 
		the replay_file() function on the replayer object through 
		it. */
		boost::thread rep_thread(boost::bind(&ReplayNode::replay_file, &replayer));
		
		// Spinning the node once.
		ros::spinOnce();
		
		/* Asking rep_thread to stop, wait for it to finish 
		running first. */
		rep_thread.interrupt();
		rep_thread.join();
		
		loopRate.sleep();
	}
	
	return(0); // Confirming successful program execution. 
}
