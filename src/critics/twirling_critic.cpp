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

#include "mppi_controller_ros/critics/twirling_critic.hpp"

#include <Eigen/Dense>

namespace mppi::critics
{

void TwirlingCritic::initialize(Parameters * params)
{
  power_ = params->twirling_cost_power;
  weight_ = params->twirling_cost_weight;
  xy_goal_tolerance_ = params->xy_goal_tolerance;
  ROS_INFO("TwirlingCritic instantiated with %d power and %f weight.", power_, weight_);
}

void TwirlingCritic::score(CriticData & data)
{
  if (!enabled_ ||
    utils::withinPositionGoalTolerance(xy_goal_tolerance_, data.state.pose.pose, data.goal))
  {
    return;
  }

  if (power_ > 1u) {
    data.costs += ((data.state.wz.abs().rowwise().mean()) * weight_).pow(power_).eval();
  } else {
    data.costs += ((data.state.wz.abs().rowwise().mean()) * weight_).eval();
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::TwirlingCritic,
  mppi::critics::CriticFunction)
