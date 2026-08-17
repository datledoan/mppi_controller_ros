cmake_minimum_required(VERSION 3.0.2)
project(mppi_controller_ros)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

add_compile_options(-O3)
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

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

# catkin build mppi_controller_ros --cmake-args -DMPPI_BUILD_BENCHMARKS=ON
option(MPPI_BUILD_BENCHMARKS "Build the benchmark/cpu_sweep_benchmark executable" OFF)
if(MPPI_BUILD_BENCHMARKS)
  add_executable(cpu_sweep_benchmark benchmark/cpu_sweep_benchmark.cpp)
  target_link_libraries(cpu_sweep_benchmark mppi_controller_ros mppi_critics ${catkin_LIBRARIES})
endif()

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

