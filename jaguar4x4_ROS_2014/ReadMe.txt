This ROS sample code is for Jaguar4x4 robot 2014 mode from Dr Robot Inc.
By default, the robot IP is "192.168.0.60", the main port number is "10001". The motor drive board is working in open loop or close loop velocity control mode.

After successfull connection, the program will publish IMU, GPS, motor sensor(encoder, temperature, ...), motor driver baord info.
You could use following cmd to check these messages:

rostopic echo /drrobot_imu
rostopic echo /drrobot_gps
rostopic echo /drrobot_motor
rostopic echo /drrobot_motorboard

This program will also subscribe to receive motor driving command.
Below command will release motor drive board EStop:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !MG'

Below command will drive robot forward:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !M 200 -200'

Below command will stop robot:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !M 0 0'


Below command will drive robot backward:
rostopic pub /drrobot_motor_cmd std_msgs/String -- 'MMW !M -200 200'

or you could use keyboard to control the robot:

rosrun jaguar4x4_2014 drrobot_keyboard_teleop_node

This program will use "WASD" key to move the robot, and release the key will stop the robot.
Before driving, you need press "z" key to release the robot from "EStop" state.


---------------------------------------------------------------------------

Past this line are any notes that are not part of the Dr. Robot development team and are part of the fieldrobot development team. If you modify this file, please put your name (first and last) above where you modified. Any whitespace between names indicate that this note was made by someone else.


(Kevin Kongmanychanh)
In any terminal intended to run any ROS code, make sure you run this first:
'source /opt/ros/noetic/setup.bash && source ~/.bashrc && cd ~/[Insert name of your catkin workspace here]/ && source devel/setup.bash'
To run any node under the Jaguar4x4 robot (any .cpp or .py file under the jaguar4x4_ROS_2014/src directory), assuming that you are already within your catkin workspace, simply input:
'rosrun jaguar4x4_2014 [put specified node name here without brackets]'
to any given terminal to run that node.

(Kevin Kongmanychanh)
The Jaguar4x4 robot, in ROS under normal keyboard operation, is composed of two nodes: jaguar4x4_2014_node (the drrobot_player.cpp file) and drrobot_keyboard_teleop_node (the drrobot_keyboard_teleop.cpp file). These all run on your PC in which ROS is running; there is no ROS code actually running in the robot itself. Note that each node must be running in its own terminal, including the master node (roscore).
The jaguar4x4_2014_node handles all of the robot's general functions, e.g. IMU, recieiving movement commands, etc.. Without this node running, the robot will not be able to move or record any data. The current drrobot_player.cpp file has been modified from the original to be able to record and save data, that data being movement commands, current direction (in radians) and its current GPS coordinates. These are saved to two different files: commands.dat and jaguarRecord.dat. These two files are being appended to right as this node is run, so if you want these two files to be fresh, make sure you erase them manually each time whenever this node is run (until a better fix is found).
The drrobot_keyboard_teleop_node allows you to control the robot through the terminal in which the node is run; it is not required for the robot to be recording and saving data, however, as the jaguar4x4_2014_node handles that. This node outputs movement commands. Whenever you press a key, it sends a corresponding movement command to the robot; holding down a key does not send another command. Releasing the key sends a stop command to the robot. The robot replays the last command sent to it, meaning that, if you were to hold down a key, since it isn't sending any new movement commands, it simply plays that key's command over and over until the key is released, in which the stop command is played.
There are currently 3 data files directly associated with the Jaguar robot: commandRead.dat, commands.dat, and jaguarRecord.dat. These each have their own specific purpose, whether it be saving or replaying data for the robot:
1. jaguarRecord.dat is a record file for the current movement command, direction, latitude, and longitude data of the robot, recorded in that order. This file updates every time the robot refreshes (every 20 ms, as the robot runs at 50Hz). This file does NOT have a timestamp when the info is recorded, but it is in order. Each new line indicates that the robot had been refreshed, therefore new data recorded.
2. commands.dat is a record file only for the current movement command. Before each movement command, a number is placed, indicating the order in which the command came in. This is to make it easier to sync up with other types of data, e.g. camera images. The data found in this file may be input into commandRead.dat directly.
3. commandRead.dat is used to replay particular movement commands to the robot. The commands within this file are directly sent to the jaguar4x4_2014_node through the replay_node (replay_node.cpp) which I created, which publishes to the /rep_cmnd topic. The movement commands within this file may be inputted manually or taken from commands.dat.
The replay_node (replay_node.cpp file) replays any given movement commands within the commandRead.dat file immediately upon being run. Do NOT run this while the drrobot_keyboard_teleop_node is running (or vice versa), as they will interfere with one another. Keep in mind that, if this node were to be stopped with [ctrl + C], the robot will replay the last given command sent by this node, so you might have to run over to the robot to turn it off manually. Once there are no movement commands left to run within the commandRead.dat file itself, the robot should stop moving, and the replay_node node may be closed using [ctrl + C].

(Kevin Kongmanychanh)
Check each node and read their comments; they often will tell you information about package and library dependencies that may not be listed here and must be manually downloaded, e.g. OpenCV (specifically opencv2) for the cmdImg_node.
