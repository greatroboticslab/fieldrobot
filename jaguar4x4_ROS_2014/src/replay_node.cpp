// Defining preprocessor directories.
#include <termios.h>
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

/* There is no namespace being used here. Please refrain from doing, 
   "using namespace [your_namepspace_here];", as it makes the code much
   harder to maintain in the future. */

/* The ReplayNode class is responsible for handling the replaying of data 
   files containing commands meant to control the robot, either recorded 
   or manually inputted into the file. */
class ReplayNode{
	public:
		/* This is the constructor for the ReplayNode class.
		   It simply initializes the publisher to advertise to 
		   the topic "rep_cmnd." */
		ReplayNode(){
			pb = nh.advertise<std_msgs::String>("rep_cmnd", 1);
			ros::NodeHandle privPart("~");
		}
		
		// Destructor for ReplayNode class.
		~ReplayNode(){}
		
		bool subReady();
		
		/* This function is defined outside of the class
		   instead of inline. Refer below for specific 
		   documentation.
		   This function replays and publishes the data to the 
		   rep_cmnd topic for the jaguar4x4_2014_node node to 
		   subscribe to (see drrobot_player.cpp). */
		void replay_file();
		
	private:
		/* Declaring the NodeHandle (nh) and the Publisher (pb) 
		   these two variables are responsible for handling the
		   publishing of the data read in through the .dat or 
		   .txt file fed into this program. */
		ros::NodeHandle nh;
		ros::Publisher pb;
		
		/* Declaring msg (commands) of String type and 
		   stringstream (cmnd_input). These handle/hold the 
		   commands read in by the replay_file() function to 
		   be published. */
		std_msgs::String commands;
		std::stringstream cmnd_input;
		
		/* Declaring String (buffer) to handle in any raw data 
		   coming in from the file. If the data read in the file 
		   is a command (one that is meant to be read), it will 
		   be put into cmnd_input (and subsequently put into 
		   commands.data) to be published. */
		std::string buffer = "";
		std::string prevBuf = buffer;
		
		int curPos = 0;
		int posBuf = 0;
		
		bool endFile = false;
		
		/* This is the fstream object that will be used to open 
		   the file for processing. */
		std::fstream cmnd_file;
};


bool ReplayNode::subReady(){
	if (pb.getNumSubscribers() < 1){
		return false;
	} else { return true; }
}

/* This function handles all the file reading and processing for the 
   file that contains commands intended to be read to the Jaguar robot. 
   It takes no arguments, but recieves a file as input and publishes 
   the valid commands it finds in that file to the robot. */
void ReplayNode::replay_file(){
	/* Opening the file. This file can be either a .dat or .txt 
	file. Please note the direct directory path; this is required 
	by ROS, as if not, it will write it in a hidden ROS 
	directory. Thsi path should be set to the path that is on 
	my computer; change this directory to the path in which you 
	need it to read from.
	This is set to read mode as to not alter the data.
	Once the file is open, performing assertion; if you get an 
	error while trying to run or compile this, this is why, as 
	you likely have an incorrect path or one that does not exist. */

	if ( (pb.getNumSubscribers() >= 1) && !endFile ){
		cmnd_file.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/commandRead.dat", std::ios::in);
		assert(cmnd_file);
		
		while ( (posBuf != curPos) && !cmnd_file.eof() ){
			cmnd_file.ignore(100, '\n');
			posBuf++;
		}
		
		cmnd_file.ignore(100, '|');
		getline(cmnd_file, buffer);
		
		/* If the buffer indicates that there has been a change 
		   in commands, set the value of cmnd_input to whatever 
		   value is in buffer, set the data of commands to 
		   what was just in cmnd_input, then clear cmnd_input. 
		   Once this is completed, publish the commands to the 
		   topic to be read by the jaguar4x4_2014_node node. */
		if (buffer != "NO_CHANGE"){
			prevBuf = buffer;
			cmnd_input << buffer;
			
			commands.data = cmnd_input.str();
			cmnd_input.str("");
			cmnd_input.clear();
			
			pb.publish(commands);
		}
		
		buffer.clear(); // Clearing buffer.
		
		cmnd_file.close(); // Closing the file once processing is done. 
		if (!cmnd_file.eof()){
			posBuf = 0;
			curPos++;
		} else { endFile = true; }
	}
}


// Defining main.
int main(int argc, char** argv){	
	// Initializing the node.
	ros::init(argc,argv,"replay_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	
	/* Declaring ReplayNode object replayer to handle processing the 
	   data */
	ReplayNode replayer;
	
	// Loop until the jaguar4x4_2014_node node responds
	while(!replayer.subReady());
	
	ros::Rate loopRate(50);
	
	while(replayer.subReady()){
		/* Initializing a thread object (rep_thread), then calling 
		   the replay_file() function on the replayer object 
		   through it. */
		boost::thread rep_thread(boost::bind(&ReplayNode::replay_file, &replayer));
		
		// Spinning the node once
		ros::spinOnce();
		
		// Asking rep_thread to stop, wait for it to finish running first.
		rep_thread.interrupt();
		rep_thread.join();
		
		loopRate.sleep();
	}
	
	return(0); // Confirming successful program execution. 
}
