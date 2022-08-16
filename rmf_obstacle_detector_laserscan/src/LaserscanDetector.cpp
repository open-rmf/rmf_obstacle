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

#include <rclcpp_components/register_node_macro.hpp>

//==============================================================================

//==============================================================================
auto LaserscanDetector::on_configure(const State& previous_state)
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
      if (!_obs_pub->is_activated())
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
auto LaserscanDetector::on_cleanup(const State& previous_state)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Cleaning up...");
  return CallbackReturn::SUCCESS;

}

//==============================================================================
auto LaserscanDetector::on_shutdown(const State& previous_state)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Shutting down...");
  return CallbackReturn::SUCCESS;

}

//==============================================================================
auto LaserscanDetector::on_activate(const State& previous_state)
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
auto LaserscanDetector::on_deactivate(const State& previous_state)
-> CallbackReturn
{
  RCLCPP_INFO(
    this->get_logger(), "Deactivating...");
  return CallbackReturn::SUCCESS;
}

//==============================================================================
auto LaserscanDetector::on_error(const State& previous_state)
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
  _range_threshold = this->declare_parameter("range_threshold", 0.05);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter range_threshold to %.3f", _range_threshold);

  _scan_topic_name = this->declare_parameter("scan_topic_name", "lidar/scan");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter scan_topic_name to %s", _scan_topic_name.c_str());

  _calibration_sample_count =
    this->declare_parameter("calibration_sample_count", 10);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter calibration_sample_count to %ld",
    _calibration_sample_count);

  const double process_rate = this->declare_parameter("process_rate", 1.0);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter process_rate to %f hz", process_rate
  );
  _process_period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double, std::ratio<1>>(1.0 / process_rate));


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

  std::size_t count = 0;
  _calibrated_scan = *_calibration_scans[0];
  auto it = _calibration_scans.begin(); ++it;
  for (; it != _calibration_scans.end(); ++it)
  {
    ++count;
    for (std::size_t i = 0; i < _calibrated_scan.ranges.size(); ++i)
    {
      _calibrated_scan.ranges[i] += (*it)->ranges[i];
    }
  }
  for (std::size_t i = 0; i < _calibrated_scan.ranges.size(); ++i)
  {
    _calibrated_scan.ranges[i] /= count;
  }


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
}

RCLCPP_COMPONENTS_REGISTER_NODE(LaserscanDetector)
