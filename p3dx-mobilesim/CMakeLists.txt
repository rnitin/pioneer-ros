#Version of CMake required
cmake_minimum_required(VERSION 2.8.3)

#Name of this package
project(p3dx-mobilesim)

# Find catkin build system, and any other packages on which we depend
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs tf)

#Declare catkin package
catkin_package()

# Specify locations of header files
include_directories( include  ${catkin_INCLUDE_DIRS})

#Delare the executable, along with its source files.
add_executable (waypoint waypoint.cpp)
add_executable (waypoint_pioneer waypoint_pioneer.cpp)

#Specify libraries against which to link.
target_link_libraries (waypoint ${catkin_LIBRARIES})
target_link_libraries (waypoint_pioneer ${catkin_LIBRARIES})
