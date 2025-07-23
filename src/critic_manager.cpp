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

#include "mppi_controller_ros/critic_manager.hpp"

namespace mppi
{

 void CriticManager::on_configure(
    std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node, const std::string & name,
    std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros, Parameters * parameters)
{
  node_ = node;
  private_node_ = private_node;
  costmap_ros_ = costmap_ros;
  name_ = name;
  parameters_ = parameters;

  critic_names_ = parameters_->critic_manager_critics;
  loadCritics();
}

void CriticManager::loadCritics()
{
  if (!loader_) {
    loader_ = std::make_unique<pluginlib::ClassLoader<critics::CriticFunction>>(
      "mppi_controller_ros", "mppi::critics::CriticFunction");
  }

  critics_.clear();
  for (auto name : critic_names_) {
    std::string fullname = getFullName(name);
    auto instance = std::unique_ptr<critics::CriticFunction>(
      loader_->createUnmanagedInstance(fullname));
    critics_.push_back(std::move(instance));
    critics_.back()->on_configure(
      node_, private_node_, name_, name_ + "." + name, costmap_ros_,
      parameters_);
    ROS_INFO("Critic loaded : %s", fullname.c_str());
  }
}

std::string CriticManager::getFullName(const std::string & name)
{
  return "mppi::critics::" + name;
}

void CriticManager::evalTrajectoriesScores(
  CriticData & data) const
{
  for (const auto & critic : critics_) {
    if (data.fail_flag) {
      break;
    }
    critic->score(data);
  }
}

}  // namespace mppi
