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

#include <stdint.h>
#include <chrono>
#include "mppi_controller_ros/controller.hpp"
#include "mppi_controller_ros/tools/utils.hpp"

// #define BENCHMARK_TESTING

namespace mppi_controller_ros
{

void MPPIControllerROS::initialize(
    std::string name, tf2_ros::Buffer* tf,
    costmap_2d::Costmap2DROS* costmap_ros)
{
  if (!isInitialized() ){
    node_ = std::make_shared<ros::NodeHandle>("~");
    private_node_ = std::make_shared<ros::NodeHandle>("~" + name);
    costmap_ros_ = std::shared_ptr<costmap_2d::Costmap2DROS>(costmap_ros);
    costmap_ = costmap_ros_->getCostmap();
    tf_buffer_ = std::shared_ptr<tf2_ros::Buffer>(tf);
    odom_helper_.setOdomTopic( odom_topic_ );
    
    // Parameters handler
    parameters_handler_ = std::make_unique<ParametersHandler>(node_, private_node_);
    params_ = parameters_handler_->getParams();

    visualize_ = params_->trajectory_visualizer_visualize;
    theta_stopped_vel_ = params_->theta_stopped_vel;
    trans_stopped_vel_ = params_->trans_stopped_vel;
    xy_goal_tolerance_ = params_->xy_goal_tolerance;
    yaw_goal_tolerance_ = params_->yaw_goal_tolerance;
    max_angular_vel_ = params_->wz_max;
    control_duration_ = 1.0 / params_->controller_frequency;

    // Configure composed objects
    optimizer_.initialize(node_, private_node_, name, costmap_ros_, params_);
    path_handler_.initialize(node_, private_node_, name, costmap_ros_, tf_buffer_, params_);
    trajectory_visualizer_.on_configure(
      node_, private_node_, name,
      costmap_ros_->getGlobalFrameID(), params_);
    initialized_ = true;
    ROS_INFO("Configured MPPI Controller: %s", name.c_str());
  }
  else{
    ROS_WARN("[MPPI] MPPIControllerROS already initialized.");
  }

}

bool MPPIControllerROS::isThetaGoalReached(double dtheta, double angle_tolerance, 
                                                        double max_angular_vel, double dt)
{
    if (fabs(dtheta) < angle_tolerance || fabs(dtheta) < max_angular_vel * dt)
    {
        return true;
    }
    return false;
}

bool MPPIControllerROS::isGoalReached()
{
  if (goal_reached_){
      ROS_INFO("[MPPI] Goal Reached!");
      return true;
  }
  return false;
}

bool MPPIControllerROS::isGoalReached(double xy_tolerance, double yaw_tolerance)
{
  if (goal_reached_){
      ROS_INFO("[MPPI] Goal Reached!");
      return true;
  }
  return false;
}

uint32_t MPPIControllerROS::computeVelocityCommands(const geometry_msgs::PoseStamped& pose,
                                  const geometry_msgs::TwistStamped& velocity,
                                  geometry_msgs::TwistStamped& cmd_vel,
                                  std::string& message)
{
  if(!initialized_)
  {
      ROS_ERROR("[MPPI] MPPIControllerROS has not been initialized");
      message = "MPPIControllerROS has not been initialized";
      return mbf_msgs::ExePathResult::NOT_INITIALIZED;
  }
#ifdef BENCHMARK_TESTING
  auto start = std::chrono::system_clock::now();
#endif

  std::lock_guard<std::mutex> param_lock(*parameters_handler_->getLock());
  geometry_msgs::Pose goal = path_handler_.getTransformedGoal(pose.header.stamp).pose;

  costmap_2d::Costmap2D * costmap = costmap_ros_->getCostmap();
  std::unique_lock<costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  nav_msgs::Path transformed_plan = path_handler_.transformPath(pose);

  nav_msgs::Odometry base_odom;
  odom_helper_.getOdom(base_odom);
  geometry_msgs::Twist speed = base_odom.twist.twist;

  geometry_msgs::PoseStamped global_goal = transformed_plan.poses.back();
  double dx_2 = global_goal.pose.position.x * global_goal.pose.position.x;
  double dy_2 = global_goal.pose.position.y * global_goal.pose.position.y;
  double dtheta = angles::normalize_angle(tf2::getYaw(global_goal.pose.orientation));

  if(fabs(std::sqrt(dx_2 + dy_2)) < xy_goal_tolerance_ && isThetaGoalReached(dtheta, yaw_goal_tolerance_, max_angular_vel_, control_duration_) && base_local_planner::stopped(base_odom, theta_stopped_vel_, trans_stopped_vel_))
  {
      goal_reached_ = true;
      return mbf_msgs::ExePathResult::SUCCESS;
  }

  geometry_msgs::TwistStamped cmd = optimizer_.evalControl(pose, speed, transformed_plan, goal);

  cmd_vel = cmd;

#ifdef BENCHMARK_TESTING
  auto end = std::chrono::system_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  ROS_INFO("Control loop execution time: %ld [ms]", duration);
#endif

  if (visualize_) {
    visualize(std::move(transformed_plan), cmd.header.stamp);
  }
  return mbf_msgs::ExePathResult::SUCCESS;
}

bool MPPIControllerROS::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
{
  std::string dummy_message;
  geometry_msgs::PoseStamped dummy_pose;
  geometry_msgs::TwistStamped dummy_velocity, cmd_vel_stamped;
  costmap_ros_->getRobotPose(dummy_pose);
  uint32_t outcome = computeVelocityCommands(dummy_pose, dummy_velocity, cmd_vel_stamped, dummy_message);
  cmd_vel = cmd_vel_stamped.twist;
  return outcome == mbf_msgs::ExePathResult::SUCCESS;
}

void MPPIControllerROS::visualize(
    nav_msgs::Path transformed_plan,
    const ros::Time & cmd_stamp)
{
  trajectory_visualizer_.add(optimizer_.getGeneratedTrajectories(), "Candidate Trajectories");
  trajectory_visualizer_.add(optimizer_.getOptimizedTrajectory(), "Optimal Trajectory", cmd_stamp);
  trajectory_visualizer_.visualize(std::move(transformed_plan));
}

void MPPIControllerROS::createPathMsg(const std::vector<geometry_msgs::PoseStamped>& plan, nav_msgs::Path& path)
{
    path.header = plan[0].header;
    for (int i = 0; i < plan.size(); i++){
        path.poses.push_back(plan[i]);
    }
}


bool MPPIControllerROS::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan)
{
  if(!initialized_)
  {
    ROS_ERROR("[MPPI] MPPIControllerROS has not been initialized, please call initialize() before using this planner");
    return false;
  }
  nav_msgs::Path path;
  createPathMsg(plan, path);
  path_handler_.setPath(path);
  return true;
}

}  // namespace mppi_controller_ros

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(mppi_controller_ros::MPPIControllerROS, nav_core::BaseLocalPlanner)
PLUGINLIB_EXPORT_CLASS(mppi_controller_ros::MPPIControllerROS, mbf_costmap_core::CostmapController)
