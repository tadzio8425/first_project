cmake_minimum_required(VERSION 2.8.3)
project(first_project)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs tf message_generation)

add_message_files(
 FILES
 sector_times.msg

 )

generate_messages(
   DEPENDENCIES
  std_msgs
 )

## Declare a catkin package
catkin_package( CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})


## Build odometry node
add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES})

## Build gps odometry node
add_executable(gps_odometer src/gps_odometer.cpp)
target_link_libraries(gps_odometer ${catkin_LIBRARIES})

## Build sector node
add_executable(sector_times src/sector_times.cpp)
target_link_libraries(sector_times ${catkin_LIBRARIES})
add_dependencies(sector_times first_project_generate_messages_cpp)

