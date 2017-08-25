a# mcts-hardware-trials
repo for hardware trials

# Pre-departure Checklist
* Router, ethernet cables, charged batteries, LiPo Monitors
* TX2s set to autoconnect to correct SSID (RDML24G)
* Charged Matrice batteries
* Charged controllers
* Charged Tablets
* Launch files prepped
* Laptop for SSH charged
* Friends and family notified

# Launch File Explainations 
* **zed.launch**
	- Used to launch the ZED ROS Wrapper, and ZED SDK. This must be launched to use the ZED Camera.
	- [ZED Params](https://www.stereolabs.com/documentation/guides/using-zed-with-ros/ZED_node.html "ZED PARAMS")
	- [ZED ROS WIKI](http://wiki.ros.org/zed-ros-wrapper "ZED ROS WIKI")
	- Also can change the names for the ZED topics in this file

* **transforms.launch**
	- Sets up all of the static TF transforms for the Matrice and ZED.
	- Links all zed frames to a _zed\_base_ frame, and the _zed\_base_ is linked to the _quad_ frame.
	- The _quad_ frame is then linked to the world frame via a transform broadcaster (see src/dji\_odom\_transform.cpp) 

* **rtabmap.launch**
	- Used to launch RTABMap, and manipulate its runtime parameters.
	- For this implementation RTABMap_ROS version 0.13.2, and Stand Alone RTABMap libs version 0.13.2 
	- **NOTE:** _rtabmap\_mapping.launch_ is a depricated version of this launch file.
	- The following are requirements for running RTABMap:
		- Depth Image, RGB Image, RGB Camera Info, a transform broadcaster between the world frame and quad frame. See (see src/dji\_odom\_transform.cpp)
	- To see a full list of parameters use:
		- *roscore*
		- *source devel/setup.bash*
		- *rosrun rtabmap_ros rtabmap --params*
	- Some useful information for using RTABMap
		- [ROS WIKI](http://wiki.ros.org/rtabmap_ros "RTABMap_ROS WIKI")
		- [RTABMap Forum](http://official-rtab-map-forum.67519.x6.nabble.com/ "Forum")
		- [Stand Alone LIB](https://github.com/introlab/rtabmap "Stand Alone Library")

* **mapping.launch**
	- This is a high level launch file for running all of the components necessary for RGB-D costmap generations. 

* **record_bag.launch**
	- This runs the same launch files as **mapping.launch**, and also records a bag file of desired topics.
	- By default this launch file writes the bag to the TX2's SSD (/media/nvidia/).
	- Topics required for running RTABMap off of the bag (**rtab_from_bag.launch**).
	- TODO: Switch the call mapping.launch from this launch file rather than doing individual calls to each launch file since they are the same.

* **rtab_from_bag.launch**
	- This launch file is used to playback recorded bags that are stored in the TX2's SSD
	- Edit the _bag\_path_ argument in the launch file to change which bag is played

* **sdk_manifold.launch**
	- This isn't actually called anywhere, but it is a working launch file for running the DJI SDK. 

# Launch case instructions:
* Before any of these instructions:
	- *source devel/setup.bash*
* To launch all of the components neccessary for RTABMap to work in real time with DJI odometry and a ZED Camera.
	- *roslaunch mcts\_hardware\_trials mapping.launch*
* To run everything that **mapping.launch** runs and also record a bag file.
	- *roslaunch 	mcts\_hardware\_trials record_bag.launch*
* To playback a bag and run a variety of different nodes.
	- Pick what you want to run depending on the bag contents by editing the arg values in the 'launch options' section of **rtab_from_bag.launch**
	- *roslaunch mcts\_hardware\_trials rtab_from_bag.launch*


# Node Explainations
* **dji_odom_transform.cpp**
	- This node subscribes to an odometry topic, and runs a TF transform broadcaster for two frames defined by that odometry relation. In this case, the odom links the _world_ and _quad_ frames.

* **goal_spotter.cpp**
	- This node is used to determine whether the search target is within the agents frame of vision.
	- **NOTE:** This node uses HSV filtering which is somewhat light condition sensitive. Thus it will work much better if tuned in the field.
	- For tuning use the **filter_gui.py** script.

* **filter_gui.py**
	- This script is loosely based on this [range detector](https://github.com/jrosebr1/imutils/blob/master/bin/range-detector "range detector")
	- Also see: [Ball Tracking](http://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/ "ball tracking")
	- Usage Instructions:
		- Point the camera at the target
		- *source devel/setup.bash*
		- *roslaunch mcts\_hardware\_trials zed.launch*
		- *rosrun mcts\_hardware\_trials filter_gui.py*
		- Two image frames and a set of sliders should appear on the screen.
		- Move the sliders until the 'filtered' is white only on the target.
		- Write down the values for each of the sliders.
		- Input the values from the slider in the cv::inRange (line 53 right now) line of **goal_spotter.cpp** in the format cv::Scalar(H\_low,S\_low,V\_low),cv::Scalar(H\_high,S\_high,V\_high)

* **vel_controller.cpp**
	- This is used to calculate proportional velocities for quads based on obstacle proximity.
	- Subscribes to a depth image topic and publishes a DJI\_Bridge\_Travel\_Speed\_MSG
	- Any object with distance 0-_lowerb_ (mm)will cause an emergency stop flag.
	- Any object with distance _lowerb_ - _upperb_ (mm) will change the published velocity on a linear scale. EX: if the nominal velocity (no obstructions visible) is 5 (m/s), and an obstacle is detected at halfway between the _lowerb_ - _upperb_ then the output velocity will be 2.5 (m/s).

# Miscellaneous
* This package was tested on TX2s flashed using JetPack 3.1
* See the issues.txt file for possible bugs and solutions that pertain to this package, and its necessary dependencies.
* For the U-Blox GPS:
	-Install: 
		- In ~/catkin_ws/src
		- *git clone https://github.com/KumarRobotics/ublox.git*
		- *catkin_make*
		- Edit launch file */catkin_ws/src/ublox_gps/ublox_device.launch* 
		- Change the *param_file_name* arg value to *nmea.yaml*
		- Change the *node_name* arg to *ublox*
	- Launching the U-Blox node:
		- *roslaunch ublox_gps ublox_device.launch*

















