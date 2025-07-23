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

#include "mppi_controller_ros/critics/velocity_deadband_critic.hpp"

namespace mppi::critics
{

void VelocityDeadbandCritic::initialize(Parameters * params)
{
    power_ = params->velocity_deadband_cost_power;
    weight_ = params->velocity_deadband_cost_weight;
    std::vector<double> deadband_velocities{
        params->velocity_deadband_velocities_1,
        params->velocity_deadband_velocities_2,
        params->velocity_deadband_velocities_3
    };
    std::transform(
        deadband_velocities.begin(), deadband_velocities.end(), deadband_velocities_.begin(),
        [](double d) {return static_cast<float>(d);});
    
    ROS_INFO_STREAM(
        "VelocityDeadbandCritic instantiated with "
        << power_ << " power, " << weight_ << " weight, deadband_velocity ["
        << deadband_velocities_.at(0) << "," << deadband_velocities_.at(1) << ","
        << deadband_velocities_.at(2) << "]");
}

void VelocityDeadbandCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  if (data.motion_model->isHolonomic()) {
    if (power_ > 1u) {
      data.costs += ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
        (fabs(deadband_velocities_[1]) - data.state.vy.abs()).max(0.0f) +
        (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
        data.model_dt).rowwise().sum() * weight_).pow(power_).eval();
    } else {
      data.costs += ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
        (fabs(deadband_velocities_[1]) - data.state.vy.abs()).max(0.0f) +
        (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
        data.model_dt).rowwise().sum() * weight_).eval();
    }
    return;
  }

  if (power_ > 1u) {
    data.costs += ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
      (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
      data.model_dt).rowwise().sum() * weight_).pow(power_).eval();
  } else {
    data.costs += ((((fabs(deadband_velocities_[0]) - data.state.vx.abs()).max(0.0f) +
      (fabs(deadband_velocities_[2]) - data.state.wz.abs()).max(0.0f)) *
      data.model_dt).rowwise().sum() * weight_).eval();
  }
  return;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(mppi::critics::VelocityDeadbandCritic, mppi::critics::CriticFunction)
