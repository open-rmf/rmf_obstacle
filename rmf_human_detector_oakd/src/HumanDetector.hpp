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

#ifndef SRC__HUMANDETECTOR_HPP
#define SRC__HUMANDETECTOR_HPP

#include <depthai/depthai.hpp>

#include <rclcpp/rclcpp.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>

#include <thread>
#include <atomic>

namespace rmf_human_detector_oakd {
//==============================================================================
class HumanDetector : public rclcpp::Node
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  HumanDetector(
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  ~HumanDetector();

private:
  struct Data
  {
    std::string detector_name;
    std::string frame_id;
    std::string level_name;
    std::string obstacle_classification;
    // TODO(YV): Consider making this a generic detector and accept the label
    // of interest as a ROS 2 param. For now we will ignore all detections
    // except "person".
    const std::vector<std::string> labels = {
      "background", "aeroplane", "bicycle", "bird", "boat", "bottle", "bus",
      "car", "cat", "chair", "cow", "diningtable", "dog", "horse",
      "motorbike", "person", "pottedplant", "sheep", "sofa", "train",
      "tvmonitor"
    };

    dai::Pipeline pipeline;
    std::atomic_bool run = true;
    std::thread detection_thread;
    bool debug; // Visualize cv frames
    rclcpp::Publisher<Obstacles>::SharedPtr pub;
  };

  std::shared_ptr<Data> _data;

};

} // namespace rmf_human_detector_oakd

#endif // SRC__HUMANDETECTOR_HPP