cmake_minimum_required(VERSION 3.0.2)
project(mppi_controller_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  angles
  base_local_planner
  costmap_2d
  mbf_costmap_core
  mbf_msgs
  nav_core
  pluginlib
  roscpp
  nav_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
)

find_package(Eigen3 REQUIRED)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

generate_dynamic_reconfigure_options(
   cfg/MPPIControllerRos.cfg
 )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mppi_controller_ros
  CATKIN_DEPENDS
  angles
  base_local_planner
  costmap_2d
  mbf_costmap_core
  mbf_msgs
  nav_core
  pluginlib
  roscpp
  nav_msgs
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamic_reconfigure
)

add_library(mppi_controller_ros SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/noise_generator.cpp
  src/optimizer.cpp
  src/parameters_handler.cpp
  src/path_handler.cpp
  src/trajectory_visualizer.cpp
)

target_link_libraries(mppi_controller_ros ${catkin_LIBRARIES})

add_library(mppi_critics SHARED
  src/critics/constraint_critic.cpp
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/obstacles_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

target_link_libraries(mppi_critics ${catkin_LIBRARIES})

install(TARGETS mppi_controller_ros mppi_critics
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
    mppic.xml
    critics.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
  
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE
)

