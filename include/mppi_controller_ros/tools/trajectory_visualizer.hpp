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

#ifndef MPPI_CONTROLLER_ROS__TOOLS__TRAJECTORY_VISUALIZER_HPP_
#define MPPI_CONTROLLER_ROS__TOOLS__TRAJECTORY_VISUALIZER_HPP_

#include <Eigen/Dense>

#include <memory>
#include <string>

#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mppi_controller_ros/tools/parameters_handler.hpp"
#include "mppi_controller_ros/tools/utils.hpp"
#include "mppi_controller_ros/models/trajectories.hpp"

namespace mppi
{

/**
 * @class mppi::TrajectoryVisualizer
 * @brief Visualizes trajectories for debugging
 */
class TrajectoryVisualizer
{
public:
  /**
    * @brief Constructor for mppi::TrajectoryVisualizer
    */
  TrajectoryVisualizer() = default;

  /**
    * @brief Configure trajectory visualizer
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param frame_id Frame to publish trajectories in
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node, const std::string & name,
    const std::string & frame_id, Parameters * parameters);

  /**
    * @brief Add an optimal trajectory to visualize
    * @param trajectory Optimal trajectory
    */
  void add(
    const Eigen::ArrayXXf & trajectory, const std::string & marker_namespace,
    const ros::Time & cmd_stamp);

  /**
    * @brief Add candidate trajectories to visualize
    * @param trajectories Candidate trajectories
    */
  void add(const models::Trajectories & trajectories, const std::string & marker_namespace);

  /**
    * @brief Visualize the plan
    * @param plan Plan to visualize
    */
  void visualize(const nav_msgs::Path & plan);

  /**
    * @brief Reset object
    */
  void reset();

protected:
  std::string frame_id_;
  ros::Publisher trajectories_publisher_;
  ros::Publisher transformed_path_pub_;
  ros::Publisher optimal_path_pub_;

  nav_msgs::Path optimal_path_;
  visualization_msgs::MarkerArray points_;
  int marker_id_ = 0;

  Parameters * parameters_;

  size_t trajectory_step_{0};
  size_t time_step_{0};
};

}  // namespace mppi

#endif  // MPPI_CONTROLLER_ROS__TOOLS__TRAJECTORY_VISUALIZER_HPP_
