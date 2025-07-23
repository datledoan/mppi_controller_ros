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

#ifndef MPPI_CONTROLLER_ROS__OPTIMIZER_HPP_
#define MPPI_CONTROLLER_ROS__OPTIMIZER_HPP_

#include <Eigen/Dense>

#include <string>
#include <memory>

#include <ros/ros.h>

#include <costmap_2d/costmap_2d_ros.h>
// #include "nav2_core/goal_checker.hpp"
// #include "nav2_core/controller_exceptions.hpp"

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>

#include "mppi_controller_ros/models/optimizer_settings.hpp"
#include "mppi_controller_ros/motion_models.hpp"
#include "mppi_controller_ros/critic_manager.hpp"
#include "mppi_controller_ros/models/state.hpp"
#include "mppi_controller_ros/models/trajectories.hpp"
#include "mppi_controller_ros/models/path.hpp"
#include "mppi_controller_ros/tools/noise_generator.hpp"
#include "mppi_controller_ros/tools/parameters_handler.hpp"
#include "mppi_controller_ros/tools/utils.hpp"

namespace mppi
{

/**
 * @class mppi::Optimizer
 * @brief Main algorithm optimizer of the MPPI Controller
 */
class Optimizer
{
public:
  /**
    * @brief Constructor for mppi::Optimizer
    */
  Optimizer() = default;

  /**
   * @brief Destructor for mppi::Optimizer
   */
  ~Optimizer() {shutdown();}


  /**
   * @brief Initializes optimizer on startup
   * @param parent WeakPtr to node
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param dynamic_parameter_handler Parameter handler object
   */
  void initialize(
    std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node, const std::string & name,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros,
    Parameters * parameters);

  /**
   * @brief Shutdown for optimizer at process end
   */
  void shutdown();

  /**
   * @brief Compute control using MPPI algorithm
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal Given Goal pose to reach.
   * @param goal_checker Object to check if goal is completed
   * @return TwistStamped of the MPPI control
   */
  geometry_msgs::TwistStamped evalControl(
    const geometry_msgs::PoseStamped & robot_pose,
    const geometry_msgs::Twist & robot_speed, const nav_msgs::Path & plan,
    const geometry_msgs::Pose & goal);

  /**
   * @brief Get the trajectories generated in a cycle for visualization
   * @return Set of trajectories evaluated in cycle
   */
  models::Trajectories & getGeneratedTrajectories();

  /**
   * @brief Get the optimal trajectory for a cycle for visualization
   * @return Optimal trajectory
   */
  Eigen::ArrayXXf getOptimizedTrajectory();

  /**
   * @brief Reset the optimization problem to initial conditions
   */
  void reset();

protected:
  /**
   * @brief Main function to generate, score, and return trajectories
   */
  void optimize();

  /**
   * @brief Prepare state information on new request for trajectory rollouts
   * @param robot_pose Pose of the robot at given time
   * @param robot_speed Speed of the robot at given time
   * @param plan Path plan to track
   * @param goal_checker Object to check if goal is completed
   */
  void prepare(
    const geometry_msgs::PoseStamped & robot_pose,
    const geometry_msgs::Twist & robot_speed,
    const nav_msgs::Path & plan,
    const geometry_msgs::Pose & goal);

  /**
   * @brief Obtain the main controller's parameters
   */
  void getParams();

  /**
   * @brief Set the motion model of the vehicle platform
   * @param model Model string to use
   */
  void setMotionModel(const std::string & model);

  /**
   * @brief Shift the optimal control sequence after processing for
   * next iterations initial conditions after execution
   */
  void shiftControlSequence();

  /**
   * @brief updates generated trajectories with noised trajectories
   * from the last cycle's optimal control
   */
  void generateNoisedTrajectories();

  /**
   * @brief Apply hard vehicle constraints on control sequence
   */
  void applyControlSequenceConstraints();

  /**
   * @brief  Update velocities in state
   * @param state fill state with velocities on each step
   */
  void updateStateVelocities(models::State & state) const;

  /**
   * @brief  Update initial velocity in state
   * @param state fill state
   */
  void updateInitialStateVelocities(models::State & state) const;

  /**
   * @brief predict velocities in state using model
   * for time horizon equal to timesteps
   * @param state fill state
   */
  void propagateStateVelocitiesFromInitials(models::State & state) const;

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
  void integrateStateVelocities(
    models::Trajectories & trajectories,
    const models::State & state) const;

  /**
   * @brief Rollout velocities in state to poses
   * @param trajectories to rollout
   * @param state fill state
   */
  void integrateStateVelocities(
    Eigen::Array<float, Eigen::Dynamic, 3> & trajectories,
    const Eigen::ArrayXXf & state) const;

  /**
   * @brief Update control sequence with state controls weighted by costs
   * using softmax function
   */
  void updateControlSequence();

  /**
   * @brief Convert control sequence to a twist commant
   * @param stamp Timestamp to use
   * @return TwistStamped of command to send to robot base
   */
  geometry_msgs::TwistStamped getControlFromSequenceAsTwist(const ros::Time & stamp);

  /**
   * @brief Whether the motion model is holonomic
   * @return Bool if holonomic to populate `y` axis of state
   */
  bool isHolonomic() const;

  /**
   * @brief Using control frequencies and time step size, determine if trajectory
   * offset should be used to populate initial state of the next cycle
   */
  void setOffset(double controller_frequency);

  /**
   * @brief Perform fallback behavior to try to recover from a set of trajectories in collision
   * @param fail Whether the system failed to recover from
   */
  bool fallback(bool fail);

protected:
  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<ros::NodeHandle> private_node_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  costmap_2d::Costmap2D * costmap_;
  std::string name_;

  std::shared_ptr<MotionModel> motion_model_;

  Parameters * parameters_;
  CriticManager critic_manager_;
  NoiseGenerator noise_generator_;

  models::OptimizerSettings settings_;

  models::State state_;
  models::ControlSequence control_sequence_;
  std::array<mppi::models::Control, 4> control_history_;
  models::Trajectories generated_trajectories_;
  models::Path path_;
  geometry_msgs::Pose goal_;
  Eigen::ArrayXf costs_;

  CriticData critics_data_ = {
    state_, generated_trajectories_, path_, goal_,
    costs_, settings_.model_dt, false, nullptr,
    std::nullopt, std::nullopt};  /// Caution, keep references

};

}  // namespace mppi

#endif  // MPPI_CONTROLLER_ROS__OPTIMIZER_HPP_
