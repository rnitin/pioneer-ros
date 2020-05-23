# pioneer-ros
ROS packages to control Pioneer 3 mobile robots and their simulations in MobileSim and Gazebo

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
    - From mobilerobots site
    
  
  
  
  
  
  
  
  
    
    1. Install g++  
        `sudo apt install make g++`\  
       2. Download ARIA package libaria_2.9.4+ubuntu16_amd64.deb  
        `wget https://web.archive.org/web/20181005213856/http://robots.mobilerobots.com/ARIA/download/current/libaria_2.9.4+ubuntu16_amd64.deb`\
       3. Install ARIA package  
        `sudo dpkg -i libaria_2.9.4+ubuntu16_amd64.deb`        
      - From alternate AriaCode fork
        Refer https://github.com/reedhedges/AriaCoda 

    3. Build catkin workspace directory  
    `cd ~/catkin_ws`\ 
    `catkin_make` 

    4. Open RosAria  
    `rosrun rosaria RosAria` 
