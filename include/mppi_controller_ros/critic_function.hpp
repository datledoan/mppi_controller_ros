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

#ifndef MPPI_CONTROLLER_ROS__CRITIC_FUNCTION_HPP_
#define MPPI_CONTROLLER_ROS__CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>

#include "mppi_controller_ros/tools/parameters_handler.hpp"
#include "mppi_controller_ros/critic_data.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost
{
  float cost{0};
  bool using_footprint{false};
};

/**
 * @class mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction
{
public:
  /**
    * @brief Constructor for mppi::critics::CriticFunction
    */
  CriticFunction() = default;

  /**
    * @brief Destructor for mppi::critics::CriticFunction
    */
  virtual ~CriticFunction() = default;

  /**
    * @brief Configure critic on bringup
    * @param parent WeakPtr to node
    * @param parent_name name of the controller
    * @param name Name of plugin
    * @param costmap_ros Costmap2DROS object of environment
    * @param dynamic_parameter_handler Parameter handler object
    */
  void on_configure(
    std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node,
    const std::string & parent_name,
    const std::string & name,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros,
    Parameters * params)
  {
    node_ = node;
    private_node_ = private_node;
    parent_name_ = parent_name;
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    params_ = params;
    enabled_ = params_->enable;

    initialize(params_);
  }

  /**
    * @brief Main function to score trajectory
    * @param data Critic data to use in scoring
    */
  virtual void score(CriticData & data) = 0;

  /**
    * @brief Initialize critic
    */
  virtual void initialize(Parameters * params) = 0;

  /**
    * @brief Get name of critic
    */
  std::string getName()
  {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<ros::NodeHandle> private_node_;
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros_;
  costmap_2d::Costmap2D * costmap_{nullptr};
  Parameters * params_;
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER_ROS__CRITIC_FUNCTION_HPP_
