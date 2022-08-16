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

#ifndef SRC__LASERSCANDETECTOR_HPP
#define SRC__LASERSCANDETECTOR_HPP


#include <rclcpp/rclcpp.hpp>
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "laser_geometry/laser_geometry.hpp"


//==============================================================================
class LaserscanDetector : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::LifecycleNode::CallbackReturn;
  using State = rclcpp_lifecycle::State;
  using LaserScan = sensor_msgs::msg::LaserScan;
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;

  LaserscanDetector(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  CallbackReturn on_configure(const State& previous_state) override;
  CallbackReturn on_cleanup(const State& previous_state) override;
  CallbackReturn on_shutdown(const State& previous_state) override;
  CallbackReturn on_activate(const State& previous_state) override;
  CallbackReturn on_deactivate(const State& previous_state) override;
  CallbackReturn on_error(const State& previous_state) override;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<Obstacles>> _obs_pub;
  rclcpp::Subscription<LaserScan>::SharedPtr _scan_sub;
  rclcpp::TimerBase::SharedPtr _process_timer;

  double _range_threshold;
  double _min_obstacle_size;

  std::string _scan_topic_name;
  std::string _level_name;
  std::chrono::nanoseconds _process_period;

  std::size_t _calibration_sample_count;
  std::vector<LaserScan::ConstSharedPtr> _calibration_scans;
  LaserScan _calibrated_scan;

  LaserScan::ConstSharedPtr _latest_scan;

  bool _calibrated;

  std::unique_ptr<laser_geometry::LaserProjection> _projector;

  void process();
  bool calibrate();
};

#endif // SRC__LASERSCANDETECTOR_HPP