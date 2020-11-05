cmake_minimum_required(VERSION 2.8.3)
project(hydrone_aerial_underwater_gazebo)

add_definitions(-std=c++11)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
)

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)
