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

Please refer here for dependencies:

The Jaguar (jaguar4x4_ROS_2014) is able to operate standalone, assuming that you are not attempting 
to collect data through the cmdImg_node. If you are attempting to do so, there are 8 dependencies 
that are required to be installed. One of these must be installed manually if it is not already 
installed on your desktop/laptop (OpenCV), and is not listed directly on the repository. 
Please refer to the comments in the cmdImg_node or read below; it will give you information on 
how to install OpenCV.

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

The image_pipeline-noetic package relies on these dependencies to operate:
1. common_msgs-noetic-devel (specifically sensor_msgs)
2. geometry-noetic-devel (image_geometry)
3. nodelet_core-noetic-devel
3. vision_opencv-noetic (cv_bridge)

If you experience any errors related to missing packages, please contact:
kck3m@mtmail.mtsu.edu for any inquiries.

----------------------------------------------------------------------------------------------------
