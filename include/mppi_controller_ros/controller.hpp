// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MPPI_CONTROLLER_ROS__CONTROLLER_HPP_
#define MPPI_CONTROLLER_ROS__CONTROLLER_HPP_

#include <string>
#include <memory>

#include "mppi_controller_ros/tools/path_handler.hpp"
#include "mppi_controller_ros/optimizer.hpp"
#include "mppi_controller_ros/tools/trajectory_visualizer.hpp"
#include "mppi_controller_ros/models/constraints.hpp"
#include "mppi_controller_ros/tools/utils.hpp"

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/goal_functions.h>
#include <base_local_planner/costmap_model.h>

#include <mbf_costmap_core/costmap_controller.h>
#include <mbf_msgs/ExePathResult.h>

namespace mppi_controller_ros
{

using namespace mppi;  // NOLINT

/**
 * @class mppi::MPPIControllerROS
 * @brief Main plugin controller for MPPI Controller
 */
class MPPIControllerROS : public nav_core::BaseLocalPlanner, public mbf_costmap_core::CostmapController
{
public:
  /**
    * @brief Constructor for mppi::MPPIControllerROS
    */
  MPPIControllerROS() = default;
  /**
    * @brief Destructor for mppi::MPPIControllerROS
    */
  ~MPPIControllerROS() = default;

  /**
    * @brief Configure controller on bringup
    * @param name Name of plugin
    * @param tf TF buffer to use
    * @param costmap_ros Costmap2DROS object of environment
    */
  void initialize(
    std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros) override;

  /**
   * @brief move_base_flex api compute the best command given the current pose and velocity, with possible debug information
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param cmd_vel   Best command
   * @param message   Debug information
   * @return          move_base_flex result code
   */
  uint32_t computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                  const geometry_msgs::TwistStamped& velocity,
                                  geometry_msgs::TwistStamped& cmd_vel,
                                  std::string& message) override;
  
  /**
   * @brief move_base api compute the best command given the current pose and velocity
   * @param pose      Current robot pose
   * @param velocity  Current robot velocity
   * @param cmd_vel   Best command
   * @return          true if a valid command was found, false otherwise
   */
  bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel) override;

  bool cancel() { 
    return false; 
  };

  /**
   * @brief Sets the global plan
   */
  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& plan);

protected:
  /**
    * @brief Visualize trajectories
    * @param transformed_plan Transformed input plan
    */
  void visualize(
    nav_msgs::Path transformed_plan,
    const ros::Time & cmd_stamp);

    /**
   * @brief move_base api whether the goal has been reached
   * @return Whether the goal has been reached
   */
  bool isGoalReached();

  /**
   * @brief move_base_flex api whether the goal has been reached
   * @return Whether the goal has been reached
   */
  bool isGoalReached(double xy_tolerance, double yaw_tolerance); 
  bool isThetaGoalReached(double dtheta, double angle_tolerance, double max_angular_vel, double dt);
  
    /** @brief Create a nav_msgs::Path message from a vector of PoseStamped messages
   * @param plan The input vector of PoseStamped messages
   * @param path The output nav_msgs::Path message
   */
  void createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path);

  const bool& isInitialized()
  {
    return initialized_;
  }

  bool goal_reached_ = false;
  bool initialized_ = false;
  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;
  double max_angular_vel_;
  double theta_stopped_vel_;
  double trans_stopped_vel_;
  double control_duration_;

  std::string name_;
  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<ros::NodeHandle> private_node_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  costmap_2d::Costmap2D * costmap_;
  base_local_planner::OdometryHelperRos odom_helper_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
  // TODO Fix hardcoded odom topic
  std::string odom_topic_{"odom"};

  std::unique_ptr<ParametersHandler> parameters_handler_;
  Parameters * params_;
  Optimizer optimizer_;
  PathHandler path_handler_;
  TrajectoryVisualizer trajectory_visualizer_;

  bool visualize_;
};

}  // namespace mppi_controller_ros

#endif  // MPPI_CONTROLLER_ROS__CONTROLLER_HPP_
