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

#include <cmath>
#include "mppi_controller_ros/critics/cost_critic.hpp"

namespace mppi::critics
{

void CostCritic::initialize(Parameters * params)
{
  consider_footprint_ = params->cost_consider_footprint;
  power_ = params->cost_power;
  weight_ = params->cost_weight;
  critical_cost_ = params->cost_critical_cost;
  collision_cost_ = params->cost_collision_cost;
  near_goal_distance_ = params->cost_near_goal_distance;
  inflation_layer_name_ = params->cost_inflation_layer_name;
  trajectory_point_step_ = params->cost_trajectory_point_step;
  
  collision_checker_ = new base_local_planner::CostmapModel(*costmap_);
  possible_collision_cost_ = findCircumscribedCost(costmap_ros_);

  if (possible_collision_cost_ < 1.0f) {
    ROS_ERROR(
      "Inflation layer either not found or inflation is not set sufficiently for "
      "optimized non-circular collision checking capabilities. It is HIGHLY recommended to set"
      " the inflation radius to be at MINIMUM half of the robot's largest cross-section. See "
      "github.com/ros-planning/navigation2/tree/main/nav2_smac_planner#potential-fields"
      " for full instructions. This will substantially impact run-time performance.");
  }

  ROS_INFO(
    "InflationCostCritic instantiated with %d power and %f / %f weights. "
    "Critic will collision check based on %s cost.",
    power_, critical_cost_, weight_,
    consider_footprint_ ? "footprint" : "circular");
}

bool CostCritic::inCollision(float cost, float x, float y, float theta)
{
  float score_cost = cost;
  if (consider_footprint_ &&
      (cost >= possible_collision_cost_ || possible_collision_cost_ < 1.0f))
    {
      score_cost = static_cast<float>(collision_checker_->footprintCost(
          static_cast<double>(x), static_cast<double>(y), static_cast<double>(theta),
          costmap_ros_->getRobotFootprint()));
    }

    switch (static_cast<unsigned char>(score_cost)) {
      case (costmap_2d::LETHAL_OBSTACLE):
        return true;
      case (costmap_2d::INSCRIBED_INFLATED_OBSTACLE):
        return consider_footprint_ ? false : true;
      case (costmap_2d::NO_INFORMATION):
        return is_tracking_unknown_ ? false : true;
    }

    return false;
}

float CostCritic::findCircumscribedCost(
  std::shared_ptr<costmap_2d::Costmap2DROS> costmap)
{
  double result = -1.0;
  const double circum_radius = costmap->getLayeredCostmap()->getCircumscribedRadius();
  if (static_cast<float>(circum_radius) == circumscribed_radius_) {
    // early return if footprint size is unchanged
    return circumscribed_cost_;
  }

  // check if the costmap has an inflation layer
  costmap_2d::InflationLayer* inflation_layer = nullptr;
  const std::vector<boost::shared_ptr<costmap_2d::Layer> >* layers =
      costmap->getLayeredCostmap()->getPlugins();

  for (size_t i = 0; i < layers->size(); ++i)
  {
    inflation_layer = dynamic_cast<costmap_2d::InflationLayer*>((*layers)[i].get());
    if (inflation_layer != nullptr)
    {
      break;
    }
  }

  if (inflation_layer != nullptr) {
    const double resolution = costmap->getCostmap()->getResolution();
    result = inflation_layer->computeCost(circum_radius / resolution);
  } else {
    ROS_WARN(
      "No inflation layer found in costmap configuration. "
      "If this is an SE2-collision checking plugin, it cannot use costmap potential "
      "field to speed up collision checking by only checking the full footprint "
      "when robot is within possibly-inscribed radius of an obstacle. This may "
      "significantly slow down planning times and not avoid anything but absolute collisions!");
  }

  circumscribed_radius_ = static_cast<float>(circum_radius);
  circumscribed_cost_ = static_cast<float>(result);

  return circumscribed_cost_;
}

void CostCritic::score(CriticData & data)
{
  if (!enabled_) {
    return;
  }

  // Setup cost information for various parts of the critic
  is_tracking_unknown_ = costmap_ros_->getLayeredCostmap()->isTrackingUnknown();
  origin_x_ = static_cast<float>(costmap_->getOriginX());
  origin_y_ = static_cast<float>(costmap_->getOriginY());
  resolution_ = static_cast<float>(costmap_->getResolution());
  size_x_ = costmap_->getSizeInCellsX();
  size_y_ = costmap_->getSizeInCellsY();

  if (consider_footprint_) {
    // footprint may have changed since initialization if user has dynamic footprints
    possible_collision_cost_ = findCircumscribedCost(costmap_ros_);
  }

  // If near the goal, don't apply the preferential term since the goal is near obstacles
  bool near_goal = false;
  if (utils::withinPositionGoalTolerance(near_goal_distance_, data.state.pose.pose, data.goal)) {
    near_goal = true;
  }

  Eigen::ArrayXf repulsive_cost(data.costs.rows());
  repulsive_cost.setZero();
  bool all_trajectories_collide = true;

  int strided_traj_cols = floor((data.trajectories.x.cols() - 1) / trajectory_point_step_) + 1;
  int strided_traj_rows = data.trajectories.x.rows();
  int outer_stride = strided_traj_rows * trajectory_point_step_;

  const auto traj_x = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_y = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));
  const auto traj_yaw = Eigen::Map<const Eigen::ArrayXXf, 0,
      Eigen::Stride<-1, -1>>(data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
      Eigen::Stride<-1, -1>(outer_stride, 1));

  for (int i = 0; i < strided_traj_rows; ++i) {
    bool trajectory_collide = false;
    float pose_cost = 0.0f;
    float & traj_cost = repulsive_cost(i);

    for (int j = 0; j < strided_traj_cols; j++) {
      float Tx = traj_x(i, j);
      float Ty = traj_y(i, j);
      unsigned int x_i = 0u, y_i = 0u;

      // The getCost doesn't use orientation
      // The footprintCostAtPose will always return "INSCRIBED" if footprint is over it
      // So the center point has more information than the footprint
      if (!worldToMapFloat(Tx, Ty, x_i, y_i)) {
        if (!is_tracking_unknown_) {
          traj_cost = collision_cost_;
          trajectory_collide = true;
          break;
        }
        pose_cost = 255.0f;  // NO_INFORMATION in float
      } else {
        pose_cost = static_cast<float>(costmap_->getCost(x_i, y_i));
        if (pose_cost < 1.0f) {
          continue;  // In free space
        }
        if (inCollision(pose_cost, Tx, Ty, traj_yaw(i, j))) {
          traj_cost = collision_cost_;
          trajectory_collide = true;
          break;
        }
      }

      // Let near-collision trajectory points be punished severely
      // Note that we collision check based on the footprint actual,
      // but score based on the center-point cost regardless
      if (pose_cost >= 253.0f /*INSCRIBED_INFLATED_OBSTACLE in float*/) {
        traj_cost += critical_cost_;
      } else if (!near_goal) {  // Generally prefer trajectories further from obstacles
        traj_cost += pose_cost;
      }
    }

    if (!trajectory_collide) {
      all_trajectories_collide = false;
    }
  }

  if (power_ > 1u) {
    data.costs += (repulsive_cost *
      (weight_ / static_cast<float>(strided_traj_cols))).pow(power_);
  } else {
    data.costs += repulsive_cost * (weight_ / static_cast<float>(strided_traj_cols));
  }

  data.fail_flag = all_trajectories_collide;
}

}  // namespace mppi::critics

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  mppi::critics::CostCritic,
  mppi::critics::CriticFunction)
