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

#ifndef MPPI_CONTROLLER_ROS__CRITIC_MANAGER_HPP_
#define MPPI_CONTROLLER_ROS__CRITIC_MANAGER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <pluginlib/class_loader.hpp>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <ros/ros.h>

#include "mppi_controller_ros/tools/parameters_handler.hpp"
#include "mppi_controller_ros/tools/utils.hpp"
#include "mppi_controller_ros/critic_data.hpp"
#include "mppi_controller_ros/critic_function.hpp"

namespace mppi
{

/**
 * @class mppi::CriticManager
 * @brief Manager of objective function plugins for scoring trajectories
 */
class CriticManager
{
public:
  typedef std::vector<std::unique_ptr<critics::CriticFunction>> Critics;
  /**
    * @brief Constructor for mppi::CriticManager
    */
  CriticManager() = default;


  /**
    * @brief Virtual Destructor for mppi::CriticManager
    */
  virtual ~CriticManager() = default;

  /**
    * @brief Configure critic manager on bringup and load plugins
    * @param parent WeakPtr to node
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node, const std::string & name,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros, Parameters * parameters);

  /**
    * @brief Score trajectories by the set of loaded critic functions
    * @param CriticData Struct of necessary information to pass to the critic functions
    */
  void evalTrajectoriesScores(CriticData & data) const;

protected:
  /**
    * @brief Load the critic plugins
    */
  virtual void loadCritics();

  /**
    * @brief Get full-name namespaced critic IDs
    */
  std::string getFullName(const std::string & name);

protected:
  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<ros::NodeHandle> private_node_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  std::string name_;

  Parameters * parameters_;
  std::vector<std::string> critic_names_;
  std::unique_ptr<pluginlib::ClassLoader<critics::CriticFunction>> loader_;
  Critics critics_;

};

}  // namespace mppi

#endif  // MPPI_CONTROLLER_ROS__CRITIC_MANAGER_HPP_
