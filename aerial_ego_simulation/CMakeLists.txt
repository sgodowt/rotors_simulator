cmake_minimum_required(VERSION 3.0.2)
project(aerial_ego_simulation)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  CATKIN_DEPENDS roscpp sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(bridge_yigle_rotorS src/bridge_yigle_rotorS.cpp)
add_dependencies(bridge_yigle_rotorS ${catkin_EXPORTED_TARGETS})
target_link_libraries(bridge_yigle_rotorS ${catkin_LIBRARIES})