#ifndef MPPI_CONTROLLER_ROS__CRITICS__COST_CRITIC_HPP_
#define MPPI_CONTROLLER_ROS__CRITICS__COST_CRITIC_HPP_

#include <memory>
#include <string>

#include <costmap_2d/footprint.h>
#include <costmap_2d/inflation_layer.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/odometry_helper_ros.h>

#include "mppi_controller_ros/critic_function.hpp"
#include "mppi_controller_ros/models/state.hpp"
#include "mppi_controller_ros/tools/utils.hpp"

namespace mppi::critics
{

/**
 * @class mppi::critics::CostCritic
 * @brief Critic objective function for avoiding obstacles using costmap's inflated cost
 */
class CostCritic : public CriticFunction
{
public:
  /**
    * @brief Initialize critic
    */
  void initialize(Parameters * params) override;

  /**
   * @brief Evaluate cost related to obstacle avoidance
   *
   * @param costs [out] add obstacle cost values to this tensor
   */
  void score(CriticData & data) override;

protected:
  /**
    * @brief Checks if cost represents a collision
    * @param cost Point cost at pose center
    * @param x X of pose
    * @param y Y of pose
    * @param theta theta of pose
    * @return bool if in collision
    */
  inline bool inCollision(float cost, float x, float y, float theta);

  /**
    * @brief Find the min cost of the inflation decay function for which the robot MAY be
    * in collision in any orientation
    * @param costmap Costmap2DROS to get minimum inscribed cost (e.g. 128 in inflation layer documentation)
    * @return double circumscribed cost, any higher than this and need to do full footprint collision checking
    * since some element of the robot could be in collision
    */
  inline float findCircumscribedCost(std::shared_ptr<costmap_2d::Costmap2DROS> costmap);

  /**
    * @brief An implementation of worldToMap fully using floats
    * @param wx Float world X coord
    * @param wy Float world Y coord
    * @param mx unsigned int map X coord
    * @param my unsigned into map Y coord
    * @return if successsful
    */
  inline bool worldToMapFloat(float wx, float wy, unsigned int & mx, unsigned int & my) const
  {
    if (wx < origin_x_ || wy < origin_y_) {
      return false;
    }

    mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
    my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

    if (mx < size_x_ && my < size_y_) {
      return true;
    }
    return false;
  }

  /**
    * @brief A local implementation of getIndex
    * @param mx unsigned int map X coord
    * @param my unsigned into map Y coord
    * @return Index
    */
  inline unsigned int getIndex(unsigned int mx, unsigned int my) const
  {
    return my * size_x_ + mx;
  }

  base_local_planner::CostmapModel* collision_checker_;
  float possible_collision_cost_;

  bool consider_footprint_{true};
  bool is_tracking_unknown_{true};
  float circumscribed_radius_{0.0f};
  float circumscribed_cost_{0.0f};
  float collision_cost_{0.0f};
  float critical_cost_{0.0f};
  float weight_{0};
  unsigned int trajectory_point_step_;

  float origin_x_, origin_y_, resolution_;
  unsigned int size_x_, size_y_;

  float near_goal_distance_;
  std::string inflation_layer_name_;

  unsigned int power_{0};
};

}  // namespace mppi::critics

#endif  // MPPI_CONTROLLER_ROS__CRITICS__COST_CRITIC_HPP_
