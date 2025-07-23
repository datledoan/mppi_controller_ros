// Copyright (c) 2022 Samsung Research America, @artofnothingness Alexey Budyakov
// Copyright (c) 2023 Open Navigation LLC
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

#include "mppi_controller_ros/critics/goal_critic.hpp"

namespace mppi::critics
{


void GoalCritic::initialize(Parameters * params)
{
    power_ = params->goal_cost_power;
    weight_ = params->goal_cost_weight;
    threshold_to_consider_ = params->goal_threshold_to_consider;

    ROS_INFO(
        "GoalCritic instantiated with %d power and %f weight.",
        power_, weight_);
}

void GoalCritic::score(CriticData & data)
{
  if (!enabled_ || !utils::withinPositionGoalTolerance(
      threshold_to_consider_, data.state.pose.pose, data.goal))
  {
    return;
  }

  const auto & goal_x = data.goal.position.x;
  const auto & goal_y = data.goal.position.y;

  const auto delta_x = data.trajectories.x - goal_x;
  const auto delta_y = data.trajectories.y - goal_y;

  if(power_ > 1u) {
    data.costs += (((delta_x.square() + delta_y.square()).sqrt()).rowwise().mean() *
      weight_).pow(power_);
  } else {
    data.costs += (((delta_x.square() + delta_y.square()).sqrt()).rowwise().mean() *
      weight_).eval();
  }
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::GoalCritic, mppi::critics::CriticFunction)
