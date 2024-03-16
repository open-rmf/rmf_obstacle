// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#ifndef HUMANDETECTOR_HPP_
#define HUMANDETECTOR_HPP_

#include <string>
#include <memory>

#include "YoloDetector.hpp"

#include <rclcpp/rclcpp.hpp>

namespace rmf_human_detector {

class HumanDetectorData;

/// \brief This class cretes a ROS 2 Node that susbcribes to:
///
/// - /camera - Camera feed
/// - /camera/camera_info - Camera info topic
/// - /camera/image_detections - Camera detections
///
/// and publish to:
///
/// - /rmf_obstacles: Topic to publish the detected object in the image
///

class HumanDetector : public rclcpp::Node
{
public:
  HumanDetector(const rclcpp::NodeOptions& options);
  ~HumanDetector();

private:
  void make_detector();

  std::shared_ptr<HumanDetectorData> _data;
};
}

#endif  // HUMANDETECTOR_HPP_
