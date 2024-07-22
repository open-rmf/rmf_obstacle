/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#include "LaserscanDetector.hpp"

#include <rmf_obstacle_msgs/msg/obstacle.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================
namespace {

struct ScanObstacle
{

  using Point = geometry_msgs::msg::Vector3;
  // Start and end indices in the ranges field
  std::size_t begin;
  std::size_t end;
  std::size_t num_ranges;
  double mean_range;
  double size;
  geometry_msgs::msg::Pose pose;

  ScanObstacle(
    std::size_t begin_,
    std::size_t end_,
    const sensor_msgs::msg::LaserScan& scan)
  : begin(std::move(begin_)),
    end(std::move(end_))
  {
    num_ranges = end - begin + 1;

    // Compute average distance
    mean_range = 0.0;
    for (std::size_t i = begin; i <= end; ++i)
      mean_range += scan.ranges[i];
    mean_range /= num_ranges;

    auto get_point =
      [&](const std::size_t index) -> Point
      {
        double theta = scan.angle_min + (index * scan.angle_increment);
        Point p;
        const auto& r = scan.ranges[index];
        p.x = r * std::cos(theta);
        p.y = r * std::sin(theta);
        p.z = 0.0;
        return p;
      };

    const std::size_t mid_index = begin + (end - begin ) * 0.5;
    const auto begin_p = get_point(begin);
    const auto end_p = get_point(end);
    const auto mid_p = get_point(mid_index);

    pose.position.x = mid_p.x;
    pose.position.y = mid_p.y;
    pose.position.z = mid_p.z;

    size = std::sqrt(
      std::pow(begin_p.x - end_p.x, 2) +
      std::pow(begin_p.y - end_p.y, 2));
  }
};

} // anonymous namespace

//==============================================================================
auto LaserscanDetector::on_configure(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Configuring...");

  _obs_pub = this->create_publisher<Obstacles>(
    "rmf_obstacles",
    rclcpp::SensorDataQoS()
  );

  _scan_sub = this->create_subscription<LaserScan>(
    _scan_topic_name,
    rclcpp::SensorDataQoS(),
    [this](LaserScan::ConstSharedPtr msg)
    {
      if (!_obs_pub->is_activated() || msg->ranges.empty())
      {
        return;
      }

      _latest_scan = msg;
      if (!_calibrated)
      {
        _calibration_scans.push_back(msg);
      }
      process();
    });

  RCLCPP_INFO(
    this->get_logger(), "Done configuring!");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto LaserscanDetector::on_cleanup(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Cleaning up...");
  _obs_pub.reset();
  _scan_sub.reset();
  _calibration_scans.clear();
  _latest_scan.reset();
  _calibrated_scan.ranges.clear();
  _calibrated = false;

  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto LaserscanDetector::on_shutdown(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Shutting down...");

  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto LaserscanDetector::on_activate(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Activating...");
  _obs_pub->on_activate();
  RCLCPP_INFO(
    this->get_logger(), "Activation successful. Waiting to receive %ld "
    "LaserScan messages on topic %s to begin calibration.",
    _calibration_sample_count, _scan_topic_name.c_str());

  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto LaserscanDetector::on_deactivate(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Deactivating...");
  _obs_pub->on_deactivate();

  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto LaserscanDetector::on_error(const State& /*previous_state*/)
-> CallbackReturn
{
  RCLCPP_ERROR(
    this->get_logger(), "Error!");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
LaserscanDetector::LaserscanDetector(const rclcpp::NodeOptions& options)
: LifecycleNode("laserscan_obstacle_detector", options),
  _calibrated(false)
{
  _range_threshold = this->declare_parameter("range_threshold", 1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter range_threshold to %.3f meters", _range_threshold);

  _min_obstacle_size = this->declare_parameter("min_obstacle_size", 0.75);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter min_obstacle_size to %.3f meters", _min_obstacle_size);

  _scan_topic_name = this->declare_parameter("scan_topic_name", "lidar/scan");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter scan_topic_name to %s", _scan_topic_name.c_str());

  _level_name = this->declare_parameter("level_name", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter level_name to %s", _level_name.c_str());

  _calibration_sample_count =
    this->declare_parameter("calibration_sample_count", 10);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter calibration_sample_count to %ld",
    _calibration_sample_count);

  RCLCPP_INFO(this->get_logger(), "Waiting to configure...");
}

//==============================================================================
bool LaserscanDetector::calibrate()
{
  if (_calibration_scans.empty() ||
    _calibration_scans.size() < _calibration_sample_count)
  {
    RCLCPP_ERROR(
      this->get_logger(),
      "calibrate() called with empty calibration_scans. Please report this bug"
    );
    return false;
  }

  // TODO(YV): Validate scans
  _calibrated_scan = *(_calibration_scans.back());
  // std::size_t count = 1;
  // auto it = _calibration_scans.begin(); ++it;
  // for (; it != _calibration_scans.end(); ++it)
  // {
  //   ++count;
  //   for (std::size_t i = 0; i < _calibrated_scan.ranges.size(); ++i)
  //   {
  //     _calibrated_scan.ranges[i] += (*it)->ranges[i];
  //   }
  // }
  // for (std::size_t i = 0; i < _calibrated_scan.ranges.size(); ++i)
  // {
  //   _calibrated_scan.ranges[i] /= count;
  // }

  _calibration_scans.clear();
  return true;
}

//==============================================================================
void LaserscanDetector::process()
{
  if (!_calibrated)
  {
    if (_calibration_scans.size() < _calibration_sample_count)
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Still waiting to receive %ld scans for calibration",
        _calibration_sample_count - _calibration_scans.size()
      );
      return;
    }
    _calibrated = calibrate();
    if (!_calibrated)
    {
      RCLCPP_ERROR(
        this->get_logger(),
        "Calibration with %ld scans failed. Resetting to try again...",
        _calibration_scans.size()
      );
      _calibration_scans.clear();
    }
    else
    {
      RCLCPP_INFO(
        this->get_logger(),
        "Calibration successful with %ld scans!",
        _calibration_sample_count
      );
    }
  }

  // Check for breaks
  if (_latest_scan->ranges.size() != _calibrated_scan.ranges.size())
  {
    RCLCPP_WARN(
      this->get_logger(),
      "Received laser scan with %ld ranges while the calibrated scan has %ld "
      "ranges. Ignoring...",
      _latest_scan->ranges.size(),
      _calibrated_scan.ranges.size()
    );
    return;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "## Processing latest_scan with angle_min: %.2f, angle_max: %.2f, angle_increment: %.6f, "
    "range_min: %.2f, range_max: %.2f, #ranges: %ld",
    _latest_scan->angle_min, _latest_scan->angle_max, _latest_scan->angle_increment,
    _latest_scan->range_min, _latest_scan->range_max,
    _latest_scan->ranges.size()
  );

  std::vector<ScanObstacle> scan_obstacles = {};
  for (std::size_t i = 0; i < _calibrated_scan.ranges.size(); ++i)
  {

    if (_latest_scan->ranges[i] < _latest_scan->range_min ||
      _latest_scan->ranges[i] > _latest_scan->range_max)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Range [%.3f] at index [%ld] is not within valid range",
        _latest_scan->ranges[i], i
      );
      continue;
    }
    const double diff =
      std::abs(_calibrated_scan.ranges[i] - _latest_scan->ranges[i]);

    if (diff < _range_threshold)
    {
      continue;
    }
    // We have an obstacle
    std::size_t begin = i;
    std::size_t end = begin;
    // Find the index where the obstacle ends
    for (std::size_t j = begin + 1; j < _calibrated_scan.ranges.size(); ++j)
    {
      const double next_diff =
        std::abs(_calibrated_scan.ranges[j] - _latest_scan->ranges[j]);
      if (next_diff > _range_threshold)
      {
        // Still part of the same obstacle
        end = j;
        continue;
      }
      // We found the end
      break;
    }

    if (begin != end)
    {
      ScanObstacle obstacle{begin, end, *_latest_scan};

      RCLCPP_INFO(
        this->get_logger(),
        "Obstacle detected with begin: %ld, end: %ld, mean_range: %.3f, size: %.2f",
        obstacle.begin, obstacle.end, obstacle.mean_range, obstacle.size
      );

      if (obstacle.size < _min_obstacle_size)
      {
        RCLCPP_INFO(
          this->get_logger(),
          "The detected obstacle will be discarded as it is below the "
          "min_obstacle_size of %.2f meters.",
          _min_obstacle_size
        );
      }
      else
      {
        scan_obstacles.push_back(std::move(obstacle));
      }
    }

    // Find the next obstacle if present
    i = end + 1;
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Detected %ld valid obstacles", scan_obstacles.size());

  if (scan_obstacles.empty())
    return;

  auto msg = std::make_unique<Obstacles>();
  msg->header.frame_id = _latest_scan->header.frame_id;
  msg->header.stamp = this->get_clock()->now();
  std::size_t id = 0;
  for (const auto& scan : scan_obstacles)
  {
    rmf_obstacle_msgs::msg::Obstacle obstacle;
    obstacle.header.frame_id = _latest_scan->header.frame_id;
    obstacle.header.stamp = this->get_clock()->now();
    obstacle.id = id;
    obstacle.source = "rmf_obstacle_detector_laserscan";
    obstacle.level_name = _level_name;
    // TODO(YV): Consider making this a param until we use sophisticated
    // algorithms for detecting obstacles from scan data.
    obstacle.classification = "obstacle";
    obstacle.bbox.center = scan.pose;
    RCLCPP_INFO(
      this->get_logger(),
      "Publishing obstacle at [%.2f, %.2f]",
      scan.pose.position.x, scan.pose.position.y
    );
    obstacle.bbox.size.x = scan.size;
    obstacle.bbox.size.y = scan.size;
    obstacle.bbox.size.z = scan.size;
    obstacle.lifetime.sec = 1;
    obstacle.action = obstacle.ACTION_ADD;
    msg->obstacles.push_back(std::move(obstacle));
    ++id;
  }
  _obs_pub->publish(std::move(msg));
}

RCLCPP_COMPONENTS_REGISTER_NODE(LaserscanDetector)
