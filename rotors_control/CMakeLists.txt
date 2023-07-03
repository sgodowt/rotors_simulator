cmake_minimum_required(VERSION 2.8.3)
project(rotors_control)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  mav_msgs
  nav_msgs
  roscpp
  sensor_msgs
  cmake_modules
  tf2
  tf2_ros
)

find_package(Eigen3 REQUIRED)

#add dynamic reconfigure api
#find_package(catkin REQUIRED dynamic_reconfigure)
generate_dynamic_reconfigure_options(
  cfg/tune.cfg
  #...
)

catkin_package(
  INCLUDE_DIRS include ${Eigen3_INCLUDE_DIRS}
  LIBRARIES lee_position_controller position_controller roll_pitch_yawrate_thrust_controller
  CATKIN_DEPENDS geometry_msgs mav_msgs nav_msgs roscpp sensor_msgs
  DEPENDS Eigen3
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(position_controller
  src/library/position_controller.cpp
)

add_library(roll_pitch_yawrate_thrust_controller
  src/library/roll_pitch_yawrate_thrust_controller.cpp
)

add_library(position_controller_RPYT
  src/library/position_controller_RPYT.cpp
)

add_library(attitude_thrust_controller
  src/library/attitude_thrust_controller.cpp
)

target_link_libraries(position_controller ${catkin_LIBRARIES})
add_dependencies(position_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})
add_dependencies(roll_pitch_yawrate_thrust_controller ${catkin_EXPORTED_TARGETS})

target_link_libraries(position_controller_RPYT ${catkin_LIBRARIES})
add_dependencies(position_controller_RPYT ${catkin_EXPORTED_TARGETS})

target_link_libraries(attitude_thrust_controller ${catkin_LIBRARIES})
add_dependencies(attitude_thrust_controller ${catkin_EXPORTED_TARGETS})

add_executable(position_controller_node src/nodes/position_controller_node.cpp)
add_dependencies(position_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(position_controller_node
  position_controller ${catkin_LIBRARIES})

add_executable(roll_pitch_yawrate_thrust_controller_node
  src/nodes/roll_pitch_yawrate_thrust_controller_node.cpp)
add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_pitch_yawrate_thrust_controller_node
  roll_pitch_yawrate_thrust_controller ${catkin_LIBRARIES})

  add_executable(position_controller_node_RPYT src/nodes/position_controller_node_RPYT.cpp)
  add_dependencies(position_controller_node_RPYT ${catkin_EXPORTED_TARGETS})
  target_link_libraries(position_controller_node_RPYT
    position_controller_RPYT ${catkin_LIBRARIES})

# make sure configure headers are built before any node using them
add_dependencies(position_controller_node_RPYT ${PROJECT_NAME}_gencfg)


  add_executable(attitude_thrust_controller_node
  src/nodes/attitude_thrust_controller_node.cpp)
add_dependencies(attitude_thrust_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(attitude_thrust_controller_node
attitude_thrust_controller ${catkin_LIBRARIES})

install(TARGETS position_controller roll_pitch_yawrate_thrust_controller attitude_thrust_controller
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS position_controller_node roll_pitch_yawrate_thrust_controller_node attitude_thrust_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
