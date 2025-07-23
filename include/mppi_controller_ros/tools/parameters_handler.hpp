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

#ifndef MPPI_CONTROLLER_ROS__TOOLS__PARAMETERS_HANDLER_HPP_
#define MPPI_CONTROLLER_ROS__TOOLS__PARAMETERS_HANDLER_HPP_

#include <functional>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include "mppi_controller_ros/MPPIControllerRosConfig.h"

namespace mppi
{

using namespace mppi_controller_ros;  // NOLINT

struct Parameters
{
  // controller
  bool enable{true};
  float vx_max;
  float vy_max;
  float vx_min;
  float wz_max;
  float ax_max;
  float ax_min;
  float ay_max;
  float az_max;
  float vx_std;
  float vy_std;
  float wz_std;
  float xy_goal_tolerance;
  float yaw_goal_tolerance;
  double trans_stopped_vel;
  double theta_stopped_vel;
  double controller_frequency;
  float ackermann_constraints_min_turning_r;

  //constrain critics
  int constrain_cost_power;
  float constrain_cost_weight;

  //cost critic
  int cost_power;
  float cost_weight;
  float cost_critical_cost;
  float cost_collision_cost;
  float cost_near_goal_distance;
  std::string cost_inflation_layer_name;
  int cost_trajectory_point_step;
  bool cost_consider_footprint;

  // goal angle critic
  int goal_angle_cost_power;
  float goal_angle_cost_weight;
  float goal_angle_threshold_to_consider;

  // goal critic
  int goal_cost_power;
  float goal_cost_weight;
  float goal_threshold_to_consider;

  // obstacles critic
  bool obstacles_consider_footprint;
  int obstacles_cost_power;
  float obstacles_repulsion_weight;
  float obstacles_critical_weight;
  float obstacles_collision_cost;
  float obstacles_collision_margin_distance;
  float obstacles_near_goal_distance;
  std::string obstacles_inflation_layer_name;
  float obstacles_inflation_scale_factor;
  float obstacles_inflation_radius;

  // path align critics
  int path_align_offset_from_furthest;
  int path_align_trajectory_point_step;
  float path_align_threshold_to_consider;
  float path_align_max_path_occupancy_ratio;
  bool path_align_use_path_orientations;
  int path_align_cost_power;
  float path_align_cost_weight;

  // path angle critic
  int path_angle_offset_from_furthest;
  int path_angle_cost_power;
  float path_angle_cost_weight;
  float path_angle_max_angle_to_furthest;
  float path_angle_threshold_to_consider;
  int path_angle_mode;

  // path follow critic
  float path_follow_threshold_to_consider;
  int path_follow_offset_from_furthest;
  int path_follow_cost_power;
  float path_follow_cost_weight;

  // prefer forward critic
  int prefer_forward_cost_power;
  float prefer_forward_cost_weight;
  float prefer_forward_threshold_to_consider;

  // twirling critic
  int twirling_cost_power;
  float twirling_cost_weight;

  // velocity deadband critic
  int velocity_deadband_cost_power;
  float velocity_deadband_cost_weight;
  double velocity_deadband_velocities_1;
  double velocity_deadband_velocities_2;
  double velocity_deadband_velocities_3;

  // path handler
  double path_handler_max_robot_pose_search_dist;
  double path_handler_prune_distance;
  double path_handler_transform_tolerance;
  bool path_handler_enforce_path_inversion;
  double path_handler_inversion_xy_tolerance;
  double path_handler_inversion_yaw_tolerance;

  // Trajectory visualizer
  bool trajectory_visualizer_visualize;
  int trajectory_visualizer_trajectory_step;
  int trajectory_visualizer_time_step;

  // Noise generator
  bool noise_generator_regenerate_noises;

  // Critic manager
  std::vector<std::string> critic_manager_critics;

  // Optimizer
  float optimizer_model_dt;
  int optimizer_time_steps;
  int optimizer_batch_size;
  int optimizer_iteration_count;
  float optimizer_temperature;
  float optimizer_gamma;
  int optimizer_retry_attempt_limit;
  std::string optimizer_motion_model;

};

/**
 * @class mppi::ParametersHandler
 * @brief Handles getting parameters and dynamic parameter changes
 */
class ParametersHandler
{
public:
  /**
   * @brief Constructor
   */
  ParametersHandler(std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node);

  /**
   * @brief Destructor
   */
  ~ParametersHandler();

  std::mutex * getLock() {return &mutex_;}

  Parameters * getParams() {return &params_;}

protected:
  std::shared_ptr<ros::NodeHandle> node_;
  std::shared_ptr<ros::NodeHandle> private_node_;
  Parameters params_;
  int map_height_;
  int map_width_;
  double map_resolution_;
  std::mutex mutex_;

  std::vector<std::string> splitStringSimple(const std::string& input, char delimiter = ',') {
    std::vector<std::string> result;
    std::stringstream ss(input);
    std::string item;
    
    while (std::getline(ss, item, delimiter)) {
        result.push_back(item);
    }
    
    return result;
  }

  void dynamicParamsCallback(const MPPIControllerRosConfig& cfg, uint32_t level);
  std::shared_ptr<dynamic_reconfigure::Server<MPPIControllerRosConfig> > dynamic_reconfigure_server_;
  dynamic_reconfigure::Server<MPPIControllerRosConfig>::CallbackType dynamic_reconfigure_callback_;
};

}  // namespace mppi

#endif  // MPPI_CONTROLLER_ROS__TOOLS__PARAMETERS_HANDLER_HPP_
