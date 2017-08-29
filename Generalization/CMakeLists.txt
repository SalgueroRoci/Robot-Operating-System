cmake_minimum_required(VERSION 2.8.3)
project(hw4)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
  include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
  	add_executable(hw4 hw4.cpp)  
	add_executable(hw4test hw4test.cpp)  

## Specify libraries to link a library or executable target against
 	target_link_libraries(hw4test ${catkin_LIBRARIES})
	target_link_libraries(hw4 ${catkin_LIBRARIES})

