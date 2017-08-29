cmake_minimum_required(VERSION 2.8.3)
project(turtlecatch)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
  include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
  add_executable(turtlecatch turtlebump.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(turtlecatch ${catkin_LIBRARIES})

