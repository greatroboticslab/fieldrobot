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

class ReplayNode{
	public:
		ReplayNode(){
			pb = nh.advertise<std_msgs::String>("rep_cmnd", 1);
			ros::NodeHandle privPart("~");
		}
		
		~ReplayNode(){}
		
		void replay_file();
		
	private:
		ros::NodeHandle nh;
		ros::Publisher pb;
		
		std_msgs::String commands;
		std::stringstream cmnd_input;
		
		std::string buffer = "";
		int curLine = 0;
		int lnInFl = 0; // Total lines in file.
		
		std::fstream cmnd_file;
};


void ReplayNode::replay_file(){
	cmnd_file.open("/home/jackal/catkin_rbt_ws/src/jaguar4x4_ROS_2014/src/commandRead.dat", std::ios::in);
	assert(cmnd_file);
	
	while(getline(cmnd_file, buffer)){
		lnInFl++;
	}
	
	cmnd_file.clear();
	cmnd_file.seekg(0, std::ios::beg);
	
	for(int pos = 0; pos < lnInFl; pos++){
		cmnd_file.ignore(100, '|');
		getline(cmnd_file, buffer);
		
		if(buffer != "NO_CHANGE"){
			cmnd_input << buffer;
			
			commands.data = cmnd_input.str();
			cmnd_input.str("");
			cmnd_input.clear();
			
			pb.publish(commands);
		}
		
		buffer.clear();
	}
	
	cmnd_file.close();
}

int main(int argc, char** argv){
	ReplayNode replayer;
	
	ros::init(argc,argv,"replay_node", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
	
	boost::thread rep_thread = boost::thread(boost::bind(&ReplayNode::replay_file, &replayer));
	
	ros::spin();
	
	rep_thread.interrupt();
	rep_thread.join();
	
	return(0);
}
