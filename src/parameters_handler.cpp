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

#include "mppi_controller_ros/tools/parameters_handler.hpp"

namespace mppi
{

ParametersHandler::ParametersHandler(std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node)
{
  node_ = node;
  private_node_ = private_node;
  
  // Controller parameters
  private_node_->param<bool>("enable", params_.enable, true);
  private_node_->param<float>("vx_max", params_.vx_max, 0.5f);
  private_node_->param<float>("vy_max", params_.vy_max, 0.0f);
  private_node_->param<float>("vx_min", params_.vx_min, -0.35f);
  private_node_->param<float>("wz_max", params_.wz_max, 1.9f);
  private_node_->param<float>("ax_max", params_.ax_max, 3.0f);
  private_node_->param<float>("ax_min", params_.ax_min, -3.0f);
  private_node_->param<float>("ay_max", params_.ay_max, 3.0f);
  private_node_->param<float>("az_max", params_.az_max, 3.5f);
  private_node_->param<float>("vx_std", params_.vx_std, 0.2f);
  private_node_->param<float>("vy_std", params_.vy_std, 0.2f);
  private_node_->param<float>("wz_std", params_.wz_std, 0.4f);
  private_node_->param<float>("xy_goal_tolerance", params_.xy_goal_tolerance, 0.25f);
  private_node_->param<float>("yaw_goal_tolerance", params_.yaw_goal_tolerance, 0.25f);
  private_node_->param<double>("theta_stopped_vel", params_.theta_stopped_vel, 0.1);
  private_node_->param<double>("trans_stopped_vel", params_.trans_stopped_vel, 0.1);
  node_->param<double>("controller_frequency", params_.controller_frequency, 20.0f);
  private_node_->param<float>("ackermann_constraints_min_turning_r", params_.ackermann_constraints_min_turning_r, 0.2f);

  // Constrain critic parameters
  private_node_->param<int>("constrain_cost_power", params_.constrain_cost_power, 1);
  private_node_->param<float>("constrain_cost_weight", params_.constrain_cost_weight, 4.0f);

  // Cost critic parameters
  private_node_->param<int>("cost_power", params_.cost_power, 1);
  private_node_->param<float>("cost_weight", params_.cost_weight, 3.81f);
  params_.cost_weight /= 254.0f;  // Normalize by cost value to put in same regime as other weights
  private_node_->param<float>("cost_critical_cost", params_.cost_critical_cost, 300.0f);
  private_node_->param<float>("cost_collision_cost", params_.cost_collision_cost, 1000000.0f);
  private_node_->param<float>("cost_near_goal_distance", params_.cost_near_goal_distance, 0.5f);
  private_node_->param<std::string>("cost_inflation_layer_name", params_.cost_inflation_layer_name, std::string(""));
  private_node_->param<int>("cost_trajectory_point_step", params_.cost_trajectory_point_step, 2);
  private_node_->param<bool>("cost_consider_footprint", params_.cost_consider_footprint, false);

  // Goal angle critic parameters
  private_node_->param<int>("goal_angle_cost_power", params_.goal_angle_cost_power, 1);
  private_node_->param<float>("goal_angle_cost_weight", params_.goal_angle_cost_weight, 3.0f);
  private_node_->param<float>("goal_angle_threshold_to_consider", params_.goal_angle_threshold_to_consider, 0.5f);

  // Goal critic parameters
  private_node_->param<int>("goal_cost_power", params_.goal_cost_power, 1);
  private_node_->param<float>("goal_cost_weight", params_.goal_cost_weight, 5.0f);
  private_node_->param<float>("goal_threshold_to_consider", params_.goal_threshold_to_consider, 1.4f);

  // Obstacles critic parameters
  private_node_->param<bool>("obstacles_consider_footprint", params_.obstacles_consider_footprint, false);
  private_node_->param<int>("obstacles_cost_power", params_.obstacles_cost_power, 1);
  private_node_->param<float>("obstacles_repulsion_weight", params_.obstacles_repulsion_weight, 1.5f);
  private_node_->param<float>("obstacles_critical_weight", params_.obstacles_critical_weight, 20.0f);
  private_node_->param<float>("obstacles_collision_cost", params_.obstacles_collision_cost, 100000.0f);
  private_node_->param<float>("obstacles_collision_margin_distance", params_.obstacles_collision_margin_distance, 0.10f);
  private_node_->param<float>("obstacles_near_goal_distance", params_.obstacles_near_goal_distance, 0.5f);
  private_node_->param<std::string>("obstacles_inflation_layer_name", params_.obstacles_inflation_layer_name, std::string(""));
  node_->param<float>("local_costmap/inflation_layer/inflation_radius", params_.obstacles_inflation_radius, 0.5f);
  node_->param<float>("local_costmap/inflation_layer/cost_scaling_factor", params_.obstacles_inflation_scale_factor, 1.0f);

  // Path align critic parameters
  private_node_->param<int>("path_align_offset_from_furthest", params_.path_align_offset_from_furthest, 20);
  private_node_->param<int>("path_align_trajectory_point_step", params_.path_align_trajectory_point_step, 4);
  private_node_->param<float>("path_align_threshold_to_consider", params_.path_align_threshold_to_consider, 0.5f);
  private_node_->param<float>("path_align_max_path_occupancy_ratio", params_.path_align_max_path_occupancy_ratio, 0.07f);
  private_node_->param<bool>("path_align_use_path_orientations", params_.path_align_use_path_orientations, false);
  private_node_->param<int>("path_align_cost_power", params_.path_align_cost_power, 1);
  private_node_->param<float>("path_align_cost_weight", params_.path_align_cost_weight, 10.0f);

  // Path angle critic parameters
  private_node_->param<int>("path_angle_offset_from_furthest", params_.path_angle_offset_from_furthest, 4);
  private_node_->param<int>("path_angle_cost_power", params_.path_angle_cost_power, 1);
  private_node_->param<float>("path_angle_cost_weight", params_.path_angle_cost_weight, 2.2f);
  private_node_->param<float>("path_angle_max_angle_to_furthest", params_.path_angle_max_angle_to_furthest, 0.785398f);
  private_node_->param<float>("path_angle_threshold_to_consider", params_.path_angle_threshold_to_consider, 0.5f);
  private_node_->param<int>("path_angle_mode", params_.path_angle_mode, 0);

  // Path follow critic parameters
  private_node_->param<float>("path_follow_threshold_to_consider", params_.path_follow_threshold_to_consider, 1.4f);
  private_node_->param<int>("path_follow_offset_from_furthest", params_.path_follow_offset_from_furthest, 6);
  private_node_->param<int>("path_follow_cost_power", params_.path_follow_cost_power, 1);
  private_node_->param<float>("path_follow_cost_weight", params_.path_follow_cost_weight, 5.0f);

  // Prefer forward critic parameters
  private_node_->param<int>("prefer_forward_cost_power", params_.prefer_forward_cost_power, 1);
  private_node_->param<float>("prefer_forward_cost_weight", params_.prefer_forward_cost_weight, 5.0f);
  private_node_->param<float>("prefer_forward_threshold_to_consider", params_.prefer_forward_threshold_to_consider, 0.5f);

  // Twirling critic parameters
  private_node_->param<int>("twirling_cost_power", params_.twirling_cost_power, 1);
  private_node_->param<float>("twirling_cost_weight", params_.twirling_cost_weight, 10.0f);

  // Velocity deadband critic parameters
  private_node_->param<int>("velocity_deadband_cost_power", params_.velocity_deadband_cost_power, 1);
  private_node_->param<float>("velocity_deadband_cost_weight", params_.velocity_deadband_cost_weight, 35.0f);
  private_node_->param<double>("velocity_deadband_velocities_1", params_.velocity_deadband_velocities_1, 0.0);
  private_node_->param<double>("velocity_deadband_velocities_2", params_.velocity_deadband_velocities_2, 0.0);
  private_node_->param<double>("velocity_deadband_velocities_3", params_.velocity_deadband_velocities_3, 0.0);

  // Path handler parameters
  double map_height, map_width;
  node_->param<double>("local_costmap/height", map_height, 3.0);
  node_->param<double>("local_costmap/width", map_width, 3.0);
  double max_costmap_dist = std::max(map_height, map_width) * 0.5;
  private_node_->param<double>("path_handler_max_robot_pose_search_dist", params_.path_handler_max_robot_pose_search_dist, max_costmap_dist);
  private_node_->param<double>("path_handler_prune_distance", params_.path_handler_prune_distance, 1.5);
  private_node_->param<double>("path_handler_transform_tolerance", params_.path_handler_transform_tolerance, 0.1);
  private_node_->param<bool>("path_handler_enforce_path_inversion", params_.path_handler_enforce_path_inversion, false);
  private_node_->param<double>("path_handler_inversion_xy_tolerance", params_.path_handler_inversion_xy_tolerance, 0.2);
  private_node_->param<double>("path_handler_inversion_yaw_tolerance", params_.path_handler_inversion_yaw_tolerance, 0.4);

  // Trajectory visualizer parameters
  private_node_->param<bool>("trajectory_visualizer_visualize", params_.trajectory_visualizer_visualize, true);
  private_node_->param<int>("trajectory_visualizer_trajectory_step", params_.trajectory_visualizer_trajectory_step, 5);
  private_node_->param<int>("trajectory_visualizer_time_step", params_.trajectory_visualizer_time_step, 3);

  // Noise generator parameters
  private_node_->param<bool>("noise_generator_regenerate_noises", params_.noise_generator_regenerate_noises, false);

  // Critic manager
  std::string critic_manager_critics_param;
  private_node_->param<std::string>("critic_manager_critics", critic_manager_critics_param, "PathAlignCritic,PathFollowCritic,PreferForwardCritic,GoalCritic,GoalAngleCritic,PathAngleCritic,TwirlingCritic,CostCritic,ConstraintCritic,VelocityDeadbandCritic");
  try
  {
    params_.critic_manager_critics = splitStringSimple(critic_manager_critics_param);
  }
  catch (const std::exception & e)
  {
    ROS_ERROR(
      "Failed to parse critic_manager_critics parameter: %s. Using default critics.",
      e.what());
    params_.critic_manager_critics = {"PathAlignCritic", "PathFollowCritic", "PreferForwardCritic", "GoalCritic", "GoalAngleCritic", "PathAngleCritic", "TwirlingCritic", "CostCritic", "ConstraintCritic", "VelocityDeadbandCritic"};
  }

  // Optimizer parameters
  private_node_->param<int>("optimizer_iteration_count", params_.optimizer_iteration_count, 1);
  private_node_->param<float>("optimizer_model_dt", params_.optimizer_model_dt, 0.05f);
  private_node_->param<int>("optimizer_time_steps", params_.optimizer_time_steps, 56);
  private_node_->param<int>("optimizer_batch_size", params_.optimizer_batch_size, 1000);
  private_node_->param<float>("optimizer_temperature", params_.optimizer_temperature, 0.3f);
  private_node_->param<int>("optimizer_retry_attempt_limit", params_.optimizer_retry_attempt_limit, 1);
  private_node_->param<float>("optimizer_gamma", params_.optimizer_gamma, 0.015f);
  private_node_->param<std::string>("optimizer_motion_model", params_.optimizer_motion_model, "DiffDrive");
  
  // dynamic reconfigure server
  dynamic_reconfigure_server_ = std::make_shared<dynamic_reconfigure::Server<MPPIControllerRosConfig>>(*private_node_);
  dynamic_reconfigure_callback_ = boost::bind(&ParametersHandler::dynamicParamsCallback, this, _1, _2);
  dynamic_reconfigure_server_->setCallback(dynamic_reconfigure_callback_);
}

ParametersHandler::~ParametersHandler()
{

}

void ParametersHandler::dynamicParamsCallback(const MPPIControllerRosConfig& cfg, uint32_t level)
{
  std::lock_guard<std::mutex> lock(mutex_);

  // Controller parameters
  params_.vx_max = cfg.vx_max;
  params_.vy_max = cfg.vy_max;
  params_.vx_min = cfg.vx_min;
  params_.wz_max = cfg.wz_max;
  params_.ax_max = cfg.ax_max;
  params_.ax_min = cfg.ax_min;
  params_.ay_max = cfg.ay_max;
  params_.az_max = cfg.az_max;
  params_.vx_std = cfg.vx_std;
  params_.vy_std = cfg.vy_std;
  params_.wz_std = cfg.wz_std;
  params_.xy_goal_tolerance = cfg.xy_goal_tolerance;
  params_.yaw_goal_tolerance = cfg.yaw_goal_tolerance;
  params_.theta_stopped_vel = cfg.theta_stopped_vel;
  params_.trans_stopped_vel = cfg.trans_stopped_vel;
  params_.controller_frequency = cfg.controller_frequency;
  params_.ackermann_constraints_min_turning_r = cfg.ackermann_constraints_min_turning_r;

  // Constrain critic parameters
  params_.constrain_cost_power = cfg.constrain_cost_power;
  params_.constrain_cost_weight = cfg.constrain_cost_weight;

  // Cost critic parameters
  params_.cost_power = cfg.cost_power;
  params_.cost_weight = cfg.cost_weight / 254.0f;  // Normalize by cost value to put in same regime as other weights
  params_.cost_critical_cost = cfg.cost_critical_cost;
  params_.cost_collision_cost = cfg.cost_collision_cost;
  params_.cost_near_goal_distance = cfg.cost_near_goal_distance;
  params_.cost_inflation_layer_name = cfg.cost_inflation_layer_name;
  params_.cost_trajectory_point_step = cfg.cost_trajectory_point_step;
  params_.cost_consider_footprint = cfg.cost_consider_footprint;

  // Goal angle critic parameters
  params_.goal_angle_cost_power = cfg.goal_angle_cost_power;
  params_.goal_angle_cost_weight = cfg.goal_angle_cost_weight;
  params_.goal_angle_threshold_to_consider = cfg.goal_angle_threshold_to_consider;

  // Goal critic parameters
  params_.goal_cost_power = cfg.goal_cost_power;
  params_.goal_cost_weight = cfg.goal_cost_weight;
  params_.goal_threshold_to_consider = cfg.goal_threshold_to_consider;

  // Obstacles critic parameters
  params_.obstacles_consider_footprint = cfg.obstacles_consider_footprint;
  params_.obstacles_cost_power = cfg.obstacles_cost_power;
  params_.obstacles_repulsion_weight = cfg.obstacles_repulsion_weight;
  params_.obstacles_critical_weight = cfg.obstacles_critical_weight;
  params_.obstacles_collision_cost = cfg.obstacles_collision_cost;
  params_.obstacles_collision_margin_distance = cfg.obstacles_collision_margin_distance;
  params_.obstacles_near_goal_distance = cfg.obstacles_near_goal_distance;
  params_.obstacles_inflation_layer_name = cfg.obstacles_inflation_layer_name;

  // Path align critic parameters
  params_.path_align_offset_from_furthest = cfg.path_align_offset_from_furthest;
  params_.path_align_trajectory_point_step = cfg.path_align_trajectory_point_step;
  params_.path_align_threshold_to_consider = cfg.path_align_threshold_to_consider;
  params_.path_align_max_path_occupancy_ratio = cfg.path_align_max_path_occupancy_ratio;
  params_.path_align_use_path_orientations = cfg.path_align_use_path_orientations;
  params_.path_align_cost_power = cfg.path_align_cost_power;
  params_.path_align_cost_weight = cfg.path_align_cost_weight;

  // Path angle critic parameters
  params_.path_angle_offset_from_furthest = cfg.path_angle_offset_from_furthest;
  params_.path_angle_cost_power = cfg.path_angle_cost_power;
  params_.path_angle_cost_weight = cfg.path_angle_cost_weight;
  params_.path_angle_max_angle_to_furthest = cfg.path_angle_max_angle_to_furthest;
  params_.path_angle_threshold_to_consider = cfg.path_angle_threshold_to_consider;
  params_.path_angle_mode = cfg.path_angle_mode;

  // Path follow critic parameters
  params_.path_follow_threshold_to_consider = cfg.path_follow_threshold_to_consider;
  params_.path_follow_offset_from_furthest = cfg.path_follow_offset_from_furthest;
  params_.path_follow_cost_power = cfg.path_follow_cost_power;
  params_.path_follow_cost_weight = cfg.path_follow_cost_weight;

  // Prefer forward critic parameters
  params_.prefer_forward_cost_power = cfg.prefer_forward_cost_power;
  params_.prefer_forward_cost_weight = cfg.prefer_forward_cost_weight;
  params_.prefer_forward_threshold_to_consider = cfg.prefer_forward_threshold_to_consider;

  // Twirling critic parameters
  params_.twirling_cost_power = cfg.twirling_cost_power;
  params_.twirling_cost_weight = cfg.twirling_cost_weight;

  // Velocity deadband critic parameters
  params_.velocity_deadband_cost_power = cfg.velocity_deadband_cost_power;
  params_.velocity_deadband_cost_weight = cfg.velocity_deadband_cost_weight;
  params_.velocity_deadband_velocities_1 = cfg.velocity_deadband_velocities_1;
  params_.velocity_deadband_velocities_2 = cfg.velocity_deadband_velocities_2;
  params_.velocity_deadband_velocities_3 = cfg.velocity_deadband_velocities_3;

  // Path handler parameters
  params_.path_handler_max_robot_pose_search_dist = cfg.path_handler_max_robot_pose_search_dist;
  params_.path_handler_prune_distance = cfg.path_handler_prune_distance;
  params_.path_handler_transform_tolerance = cfg.path_handler_transform_tolerance;
  params_.path_handler_enforce_path_inversion = cfg.path_handler_enforce_path_inversion;
  params_.path_handler_inversion_xy_tolerance = cfg.path_handler_inversion_xy_tolerance;
  params_.path_handler_inversion_yaw_tolerance = cfg.path_handler_inversion_yaw_tolerance;

  // Trajectory visualizer parameters
  params_.trajectory_visualizer_visualize = cfg.trajectory_visualizer_visualize;
  params_.trajectory_visualizer_trajectory_step = cfg.trajectory_visualizer_trajectory_step;
  params_.trajectory_visualizer_time_step = cfg.trajectory_visualizer_time_step;

  // Noise generator parameters
  params_.noise_generator_regenerate_noises = cfg.noise_generator_regenerate_noises;

  // Optimizer parameters
  params_.optimizer_iteration_count = cfg.optimizer_iteration_count;
  params_.optimizer_model_dt = cfg.optimizer_model_dt;
  params_.optimizer_time_steps = cfg.optimizer_time_steps;
  params_.optimizer_batch_size = cfg.optimizer_batch_size;
  params_.optimizer_temperature = cfg.optimizer_temperature;
  params_.optimizer_retry_attempt_limit = cfg.optimizer_retry_attempt_limit;
  params_.optimizer_gamma = cfg.optimizer_gamma;
  params_.optimizer_motion_model = cfg.optimizer_motion_model;
}

}  // namespace mppi
