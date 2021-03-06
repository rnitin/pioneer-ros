
# pioneer-ros
ROS packages to control Pioneer 3 mobile robots and their simulations in MobileSim and Gazebo. Packages created during my internship at ACM Lab, Dalhousie University.

## 1. Environment
- Ubuntu 16.04, ROS Kinetic Kame 
- Ubuntu 18.04, ROS Melodic Morenia 
  
## 2. Package Documentation

**RosAria**  
https://wiki.ros.org/ROSARIA 
  
**ARIA**  
- https://www.eecs.yorku.ca/course_archive/2009-10/W/4421/doc/pioneer/aria/main.html 
- https://web.archive.org/web/20181005213856/http://robots.mobilerobots.com/wiki/ARIA 

**MobileSim** 
- https://web.archive.org/web/20181006012429/http://robots.mobilerobots.com/wiki/MobileSim 
- http://vigir.missouri.edu/~gdesouza/Research/MobileRobotics/Software/MobileSim/README.html 

## 3. Package Installation

- **RosAria**  
	1. Bring RosAria into workspace  
    `cd ~/catkin_ws/src`  
    `git clone https://github.com/amor-ros-pkg/rosaria.git`  
  2. Install ARIA   
	  - From mobilerobots site archive  
		  1.  Install g++  
		 `sudo apt install make g++`  
		 2.  Download ARIA package libaria_2.9.4+ubuntu16_amd64.deb
		      `wget https://web.archive.org/web/20181005213856/http://robots.mobilerobots.com/ARIA/download/current/libaria_2.9.4+ubuntu16_amd64.deb`
		 3. Install the package  
		 `sudo dpkg -i libaria_2.9.4+ubuntu16_amd64.deb`
	  - From  alternate AriaCode fork  
	  Refer https://github.com/reedhedges/AriaCoda  
  4. Build catkin workspace directory  
    `cd ~/catkin_ws`  
    `catkin_make`  
  5. Open RosAria  
    `rosrun rosaria RosAria`  
      
- **MobileSim**   
	1. Download MobileSim package  mobilesim_0.9.8+ubuntu16_amd64.deb from archive  
	`wget https://web.archive.org/web/20181006012429/http://robots.mobilerobots.com/MobileSim/download/current/mobilesim_0.9.8+ubuntu16_amd64.deb`  
	2. Install MobileSim package  
	`sudo dpkg -i mobilesim_0.9.8+ubuntu16_amd64.deb`
	3. Launch MobileSim from terminal  
	`MobileSim -nomap`  
	
- **Pioneer P3-DX Model on Gazebo**  
	1.  Clone the pioneer_p3dx_model package from  https://github.com/mario-serna/pioneer_p3dx_model  
	`cd ~/catkin_ws/src`  
	`git clone https://github.com/mario-serna/pioneer_p3dx_model.git`  
	2.  Build catkin workspace directory  
	`cd ~/catkin_ws`  
	`catkin_make`  
	3.  Launch Gazebo  
	`roslaunch gazebo_ros empty_world.launch`  
	4.  Spawn the P3-DX model on Gazebo  
	`roslaunch p3dx_gazebo p3dx.launch`

- **pioneer-ros**  
	1. Bring *pioneer-ros* package into workspace  
    `cd ~/catkin_ws/src`  
    `git clone https://github.com/rnitin/pioneer-ros.git`  
  2. Build catkin workspace directory  
    `cd ~/catkin_ws`  
    `catkin_make`

## 4. Executing ROS Packages
- **Waypoint Navigation on MobileSim**  
	Package: *p3dx-mobilesim*
	1. Start MobileSim from terminal  
	`MobileSim --nomap`  
	2. Execute the launch file from another terminal  
	`roslaunch p3dx-mobilesim waypoint.launch`  

- **Waypoint Navigation on Gazebo**  	
	Package: *p3dx-gazebo*
	Execute the launch file from terminal  
	`roslaunch p3dx-gazebo waypointgz.launch`
	 
- **Waypoint Navigation on the Pioneer 3 robot**  	
	Package: *p3dx-robot*
	1. Connect the robot to the computer
	2. Enable USB permission for the port (ttyUSB0 here)  
	`sudo chmod 777 -R /dev/ttyUSB0`  
	3. Execute the launch file from terminal  
	`roslaunch p3dx-robot waypointr.launch` 
