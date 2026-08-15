#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Path.h>

#include "mppi_controller_ros/optimizer.hpp"
#include "mppi_controller_ros/tools/parameters_handler.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <sys/resource.h>
#include <thread>
#include <atomic>
#include <vector>
#include <algorithm>
#include <string>

static unsigned char inflationCost(float dist_m, float inscribed_radius, float inflation_radius,
                                    float cost_scaling_factor)
{
  if (dist_m < 0.0f)
    return 254;
  if (dist_m <= inscribed_radius)
    return 253;
  if (dist_m > inflation_radius)
    return 0;
  return static_cast<unsigned char>(252.0f * expf(-cost_scaling_factor * (dist_m - inscribed_radius)));
}

struct SweepPointResult
{
  double wall_mean_ms, wall_median_ms, wall_p95_ms, wall_max_ms, cpu_mean_ms;
};

SweepPointResult runSweepPoint(std::shared_ptr<ros::NodeHandle> node, std::shared_ptr<ros::NodeHandle> private_node,
                                std::shared_ptr<costmap_2d::Costmap2DROS> costmap_ros, int batch_size,
                                int time_steps, int warmup, int reps, const geometry_msgs::PoseStamped& pose,
                                const geometry_msgs::Twist& speed, const nav_msgs::Path& path,
                                const geometry_msgs::Pose& goal)
{
  ros::param::set("/mppi_cpu_sweep_bench/optimizer_batch_size", batch_size);
  ros::param::set("/mppi_cpu_sweep_bench/optimizer_time_steps", time_steps);

  mppi::ParametersHandler params_handler(node, private_node);
  mppi::Parameters* params = params_handler.getParams();

  mppi::Optimizer optimizer;
  optimizer.initialize(node, private_node, "mppi_cpu_sweep_bench", costmap_ros, params);

  for (int i = 0; i < warmup; i++)
  {
    optimizer.evalControl(pose, speed, path, goal);
  }

  std::vector<double> wall_ms;
  std::vector<double> cpu_ms;
  wall_ms.reserve(reps);
  cpu_ms.reserve(reps);
  for (int i = 0; i < reps; i++)
  {
    struct rusage ru_before, ru_after;
    getrusage(RUSAGE_SELF, &ru_before);
    auto t0 = std::chrono::steady_clock::now();
    optimizer.evalControl(pose, speed, path, goal);
    auto t1 = std::chrono::steady_clock::now();
    getrusage(RUSAGE_SELF, &ru_after);
    wall_ms.push_back(std::chrono::duration<double, std::milli>(t1 - t0).count());
    double cpu_delta = (ru_after.ru_utime.tv_sec - ru_before.ru_utime.tv_sec) * 1000.0 +
                        (ru_after.ru_utime.tv_usec - ru_before.ru_utime.tv_usec) / 1000.0 +
                        (ru_after.ru_stime.tv_sec - ru_before.ru_stime.tv_sec) * 1000.0 +
                        (ru_after.ru_stime.tv_usec - ru_before.ru_stime.tv_usec) / 1000.0;
    cpu_ms.push_back(cpu_delta);
  }

  optimizer.shutdown();

  std::sort(wall_ms.begin(), wall_ms.end());
  SweepPointResult r;
  double wall_sum = 0.0;
  for (double v : wall_ms)
    wall_sum += v;
  r.wall_mean_ms = wall_sum / wall_ms.size();
  r.wall_median_ms = wall_ms[wall_ms.size() / 2];
  r.wall_p95_ms = wall_ms[static_cast<size_t>(wall_ms.size() * 0.95)];
  r.wall_max_ms = wall_ms.back();
  double cpu_sum = 0.0;
  for (double v : cpu_ms)
    cpu_sum += v;
  r.cpu_mean_ms = cpu_sum / cpu_ms.size();
  return r;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mppi_cpu_sweep_bench");
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Warn))
  {
    ros::console::notifyLoggerLevelsChanged();
  }

  auto node = std::make_shared<ros::NodeHandle>();
  auto private_node = std::make_shared<ros::NodeHandle>("~");

  int warmup = argc > 1 ? std::atoi(argv[1]) : 5;
  int reps = argc > 2 ? std::atoi(argv[2]) : 30;
  if (warmup < 0 || reps <= 0)
  {
    fprintf(stderr, "Usage: %s [warmup] [reps]  (warmup >= 0, reps >= 1; defaults 5 30)\n", argv[0]);
    fprintf(stderr, "Got warmup=%d reps=%d from argv -- these must be plain numbers, not the literal\n",
            warmup, reps);
    fprintf(stderr, "\"[warmup]\"/\"[reps]\" brackets shown in the README (those just mean \"optional\").\n");
    return 1;
  }

  const std::string node_name = ros::this_node::getName();
  const std::string costmap_name = "cpu_bench_costmap";
  const std::string costmap_ns = node_name + "/" + costmap_name;
  const int costmap_cells = 200;
  const float costmap_res = 0.025f;
  const float costmap_origin = -2.5f;
  XmlRpc::XmlRpcValue empty_plugins;
  empty_plugins.setSize(0);
  ros::param::set(costmap_ns + "/plugins", empty_plugins);
  ros::param::set(costmap_ns + "/global_frame", std::string("map"));
  ros::param::set(costmap_ns + "/robot_base_frame", std::string("base_link"));
  ros::param::set(costmap_ns + "/static_map", false);
  ros::param::set(costmap_ns + "/rolling_window", false);
  ros::param::set(costmap_ns + "/width", costmap_cells * costmap_res);
  ros::param::set(costmap_ns + "/height", costmap_cells * costmap_res);
  ros::param::set(costmap_ns + "/resolution", costmap_res);
  ros::param::set(costmap_ns + "/origin_x", costmap_origin);
  ros::param::set(costmap_ns + "/origin_y", costmap_origin);
  ros::param::set(costmap_ns + "/robot_radius", 0.105);
  ros::param::set(costmap_ns + "/publish_frequency", 0.0);
  ros::param::set(costmap_ns + "/update_frequency", 0.0);

  tf2_ros::Buffer tf_buffer(ros::Duration(10));
  tf2_ros::TransformListener tf_listener(tf_buffer);
  tf2_ros::TransformBroadcaster tf_broadcaster;
  std::atomic<bool> keep_broadcasting{ true };
  std::thread tf_thread([&]() {
    while (keep_broadcasting)
    {
      geometry_msgs::TransformStamped t;
      t.header.stamp = ros::Time::now() + ros::Duration(0.5);
      t.header.frame_id = "map";
      t.child_frame_id = "base_link";
      t.transform.rotation.w = 1.0;
      tf_broadcaster.sendTransform(t);
      ros::spinOnce();
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  });

  auto costmap_ros = std::make_shared<costmap_2d::Costmap2DROS>(costmap_name, tf_buffer);

  {
    costmap_2d::Costmap2D* costmap = costmap_ros->getCostmap();
    const float obst_wx = 1.0f, obst_wy = 0.15f;
    const float inscribed_radius = 0.105f, inflation_radius = 1.0f, cost_scaling_factor = 3.0f;
    for (unsigned int gy = 0; gy < costmap->getSizeInCellsY(); gy++)
    {
      for (unsigned int gx = 0; gx < costmap->getSizeInCellsX(); gx++)
      {
        double wx, wy;
        costmap->mapToWorld(gx, gy, wx, wy);
        float dist = std::sqrt((wx - obst_wx) * (wx - obst_wx) + (wy - obst_wy) * (wy - obst_wy)) - 0.15f;
        costmap->setCost(gx, gy, inflationCost(dist, inscribed_radius, inflation_radius, cost_scaling_factor));
      }
    }
  }

  const int path_n = 50;
  const float path_step = 0.1f;
  const float heading_offset_deg = 20.0f;
  const float theta = heading_offset_deg * static_cast<float>(M_PI) / 180.0f;

  geometry_msgs::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = ros::Time::now();
  pose.pose.orientation.w = 1.0;

  geometry_msgs::Twist speed;  // zero

  nav_msgs::Path path;
  path.header.frame_id = "map";
  path.header.stamp = ros::Time::now();
  for (int i = 0; i < path_n; i++)
  {
    float s = path_step * i;
    geometry_msgs::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = s * std::cos(theta);
    p.pose.position.y = s * std::sin(theta);
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  }
  geometry_msgs::Pose goal = path.poses.back().pose;

  std::vector<int> batch_sizes = { 256, 512, 1024, 2048, 4096, 8192};
  std::vector<int> timesteps_sweep = { 32, 56, 100 };  // matches the CUDA sweep's grid

  printf("batch_size,time_steps,wall_mean_ms,wall_median_ms,wall_p95_ms,wall_max_ms,cpu_mean_ms\n");
  fflush(stdout);
  for (int batch_size : batch_sizes)
  {
    for (int time_steps : timesteps_sweep)
    {
      SweepPointResult r =
          runSweepPoint(node, private_node, costmap_ros, batch_size, time_steps, warmup, reps, pose, speed, path, goal);
      printf("%d,%d,%.4f,%.4f,%.4f,%.4f,%.4f\n", batch_size, time_steps, r.wall_mean_ms, r.wall_median_ms,
             r.wall_p95_ms, r.wall_max_ms, r.cpu_mean_ms);
      fflush(stdout);
    }
  }

  keep_broadcasting = false;
  tf_thread.join();
  return 0;
}
