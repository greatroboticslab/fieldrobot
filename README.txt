----------------------------------------------------------------------------------------------------

Before attempting to download/install these packages to work with the Jaguar robot, please 
ensure that:

1. You have properly installed the Ubuntu OS (specifically Ubuntu 20.04) on the desktop/laptop 
that you are using to work on; if not, please refer to:
https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview

Install the Ubuntu OS on a unformatted partition/separate disk as a dual boot if you do not want 
to lose your current OS and files or have file conflicts/problems. If you do not know how to set up 
an unformatted partition on your disk, refer to:
https://support.microsoft.com/en-us/windows/create-and-format-a-hard-disk-partition-bbb8e185-1bda-ecd1-3465-c9728f7d7d2e

The Ubuntu installation guide should tell you how to install Ubuntu as a dual boot. Make sure you 
do not select fresh installation if you are attempting to install Ubuntu as a dual boot.


2. You have properly installed and configured the correct version of ROS for the Jaguar 
(ROS 1 Noetic); if not, please refer to:
https://wiki.ros.org/noetic/Installation/Ubuntu


3. You have properly configured your catkin workspace; if not, please refer to 
https://wiki.ros.org/catkin/Tutorials/create_a_workspace

----------------------------------------------------------------------------------------------------

Once you have downloaded the files, make sure that you have unzipped any of the .zip files; 
these are packages that are required dependencies for some of the other packages that the 
Jaguar relies on. Make sure that these unzipped files are now within your catkin workspace's 
src directory.

Once this is done, open a new gnome-terminal by pressing and holding down:
[CTRL] + [ALT] + [T]

Then, in your new terminal, type:
source /opt/ros/noetic/setup.bash && source ~/.bashrc && cd ~/[NAME_OF_YOUR_WORKSPACE]/ && source devel/setup.bash
(Make sure you replace the bracketed section with your workspace's name)
This will configure your ROS sources and set up your workspace environment. Make sure you run 
this command on every new terminal you open if you are working with ROS or your catkin workspace 
in that terminal.

Then type:
catkin_make

This will do two main things:
1. Link any dependencies between packages and their nodes/source code.
2. Compile any nodes/source code.

If there are any compilation issues in the source code, or if there are missing/unlisted 
dependencies, catkin_make will throw an error.

If you edit or add any source code within a package or add a new package, always do 
catkin_make; also make sure that you have updated the CMakeLists.txt file and the package.xml 
file in the respective package if you have just added a new source file, node, or dependency. 
If you do not know what these two files are, refer to these two links for more information:
http://wiki.ros.org/catkin/CMakeLists.txtf
http://wiki.ros.org/catkin/package.xml

A dependency may be listed as missing if CMakeLists.txt or package.xml is not properly configured.

Some dependencies cannot be installed normally by just placing them within the src folder; 
if you are working with any node/source code developed by the fieldrobot team, refer to the 
comments in the code or the respective README.txt file, as it will often contain information 
on how to install that specific dependency/dependencies.

----------------------------------------------------------------------------------------------------

*** PLEASE REFER HERE FOR DEPENDENCIES ***

The Jaguar (jaguar4x4_ROS_2014) is able to operate standalone, assuming that you are not attempting 
to collect data through the cmdImg_node. If you are attempting to do so, there are 8 dependencies 
that are required to be installed. Two of these must be installed manually if they aren't already 
installed on your desktop/laptop (i.e. OpenCV, libturbojpeg), and aren't listed directly on the 
repository. Please refer below for more information on how to install OpenCV and libturbojpeg.

To operate the Jaguar's camera for image processing and general access through ROS, the 
Jaguar relies on these dependencies to operate:
1. axis_camera-master
2. vision_opencv-noetic (cv_bridge)
3. OpenCV

OpenCV, as stated before, must be installed manually. Please refer to one of the links below to 
install it if it hasn't already been installed on your system.
https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html
https://www.geeksforgeeks.org/how-to-install-opencv-in-c-on-linux/

The axis_camera-master package relies on these dependencies to operate:
1. camera_info_manager_py
2. image_pipeline-noetic
3. image_transport_plugins-noetic-devel

The image_pipeline-noetic package relies on these dependencies to operate:
1. common_msgs-noetic-devel (specifically sensor_msgs)
2. geometry-noetic-devel (image_geometry)
3. nodelet_core-noetic-devel
3. vision_opencv-noetic (cv_bridge)

The image_transport_plugins-noetic-devel package relies on libturbojpeg to operate, which must 
be installed manually. To install libturbojpeg, open up a new gnome-terminal and run the 
following command:
sudo apt install libturbojpeg0-dev
Alternatively, you may install from source through the following link, though this is more 
difficult:
https://github.com/libjpeg-turbo/libjpeg-turbo

If you experience any errors related to missing packages, please contact:
kck3m@mtmail.mtsu.edu for any inquiries.

----------------------------------------------------------------------------------------------------

*** HOW TO POWER ON/OFF THE JAGUAR ROBOT AND CONNECT TO IT ***

!!! WHEN HANDLING THE BATTERY AND/OR WORKING WITH ANY INTERNAL PARTS OF THE JAGUAR, ENSURE THAT THE ROBOT IS POWERED OFF. READ BELOW FOR MORE INFORMATION. !!!
!!! ENSURE YOU ALWAYS TURN OFF THE ROBOT AND REMOVE AND DISCONNECT THE BATTERY WHEN DONE OPERATING IT !!!


To turn the Jaguar robot on:

1. Ensure that the battery is plugged in properly. The battery fits into a slot on the
top of the robot, towards the back (in front of the tow hitch). This battery should have 
two main wire cords running in/out of it; one should be a white, 7 pin Tamiya (female) 
connector, while the other should be a yellow, Amass XT90 (female) connector. For the 
Jaguar to operate, the Amass connector should be connected to its male counterpart, 
located somewhere inside of the slot. It should also be yellow. Connect these two Amass
connectors together, then insert the battery inside of the slot. This may not fit 
properly at first, so you may have to fiddle around with the wire positions a bit and/or 
wiggle the battery in/out of place.

2. Navigate to the back plate of the Jaguar robot (where the tow hitch is located). 

3. Locate the labeled OFF and ON switch, located in the upper right corner of the Jaguar's back 
panel, and the labeled START button, located directly below the OFF and ON switch.

4. Switch the switch to ON (white line on switch is pointing towards ON). Then, press the START 
button for around half a second to a second, then release. This will power on the robot.


To turn the Jaguar robot off:

1. Switch the OFF and ON switch to OFF (white line on switch is pointing towards OFF).

2. Remove the battery from the battery slot, then disconnect the male and female Amass XT90 
connectors.


To connect to the Jaguar robot:

1. Assuming you have just turned on the robot, wait for the Jaguar to completely start up. 
In the meantime, open up the Settings on your system and navigate to Wi-Fi. When the Jaguar 
is ready, under your Visible Networks, you will see DriJaguar.

(IGNORE STEPS 2-5 IF YOU ARE ON THE LAB'S UBUNTU DESKTOP OR HAVE ALREADY CONFIGURED THE NETWORK SETTINGS)

2. Tap on the Settings/cog icon to the right of the DriJaguar tab and navigate to the IPv4 
settings. Set the IPv4 Method to Manual.

3. Configure the Address (under Addresses) to 192.168.0.104, and the Netmask to 255.255.255.0

4. Configure the DNS (under DNS) to 192.168.0.104

5. Click apply in the upper right hand corner of the tab window.

6. Click on DriJaguar and connect to it (assuming it has not done this already).

----------------------------------------------------------------------------------------------------

*** HOW TO CHARGE THE JAGUAR'S BATTERY ***

!!! WHEN HANDLING THE BATTERY AND/OR WORKING WITH ANY INTERNAL PARTS OF THE JAGUAR, ENSURE THAT THE ROBOT IS POWERED OFF. !!!
!!! WHEN CHARGING THE BATTERY OF THE JAGUAR, ALWAYS KEEP IT WITHIN DIRECT OBSERVATION, NEVER LEAVE THE BATTERY UNATTENDED, AND NEVER LEAVE THE ROOM WHILE IT IS CHARGING UNLESS DIRECTED BY DR. HONGBO ZHANG. ALWAYS REMEMBER TO DISCONNECT THE BATTERY FROM THE CHARGER ONCE FINISHED OR WHEN LEAVING. FAILURE TO HEED THESE INSTRUCTIONS/WARNINGS WILL RESULT IN A FIRE AND/OR HARM TO SELF AND/OR OTHERS. !!!


To charge the Jaguar robot's battery:

1. Turn the Jaguar off.

2. Remove the battery from the battery slot within the Jaguar and disconnect the yellow Amass XT90 
connectors from one another.

3. Retrieve the AC/DC power adapter (MODEL: LJH074-120100WI, INPUT: 100-240V~50/60Hz, 
OUTPUT: 12V⎓10A) and the TENERGY TB6B Multifunctional Balance Charger (DC INPUT 11-18V).

4. Connect the adapter to the TENERGY charger through the small, round port on the left side of the 
charger (directly to the left of the red TENERGY logo on the left side of the charger).

5. Connect the battery's Amass XT90 (female) connector to the TENERGY charger's Amass XT90 (male) 
connector.

6. Connect the battery's 7 pin Tamiya (female) connector to the TENERGY charger's 7 pin Tamiya 
(male) connector.

4. Connect the adapter plug to the wall outlet/socket. This will power on the TENERGY charger. 
The charger will beep, and then the screen should display:
LiIo CHARGE
3.0A   21.6V(6S)
If this does not display, or the settings are incorrect, please contact Dr. Hongbo Zhang.

5. Hold down the rightmost button (ENTER/START). This will perform a battery check. Tap the button 
again to confirm. The battery should now be charging.


To stop charging the battery:

1. Disconnect the adapter plug from the wall outlet. DO THIS FIRST.

2. Disconnect the adapter from the TENERGY charger.

3. Disconnect the connectors from the battery.

----------------------------------------------------------------------------------------------------
