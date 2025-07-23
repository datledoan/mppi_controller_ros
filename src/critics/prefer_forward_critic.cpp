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

#include "mppi_controller_ros/critics/prefer_forward_critic.hpp"

#include <Eigen/Dense>

namespace mppi::critics
{

void PreferForwardCritic::initialize(Parameters * params)
{
  power_ = params->prefer_forward_cost_power;
  weight_ = params->prefer_forward_cost_weight;
  threshold_to_consider_ = params->prefer_forward_threshold_to_consider;

  ROS_INFO("PreferForwardCritic instantiated with %d power and %f weight.", power_, weight_);
}

void PreferForwardCritic::score(CriticData & data)
{
  if (!enabled_ || utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.goal))
  {
    return;
  }

  if (power_ > 1u) {
    data.costs += (
      (data.state.vx.unaryExpr([&](const float & x){return std::max(-x, 0.0f);}) *
      data.model_dt).rowwise().sum() * weight_).pow(power_);
  } else {
    data.costs += (data.state.vx.unaryExpr([&](const float & x){return std::max(-x, 0.0f);}) *
      data.model_dt).rowwise().sum() * weight_;
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::PreferForwardCritic,
  mppi::critics::CriticFunction)
