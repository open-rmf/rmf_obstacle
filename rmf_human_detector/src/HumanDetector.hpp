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

#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

class HumanDetector : public rclcpp::Node
{
public:
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;
  HumanDetector();
  ~HumanDetector();

private:
  void make_detector();

  struct Data
  {
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr _camera_pose_sub;
    rclcpp::Publisher<Obstacles>::SharedPtr _obstacles_pub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_detections_pub;
    rclcpp::Subscription<rmf_building_map_msgs::msg::BuildingMap>::SharedPtr
      _building_map_sub;
    std::shared_ptr<YoloDetector> _detector;
    std::string _camera_name;
    std::string _camera_parent_name;
    std::string _camera_image_topic;
    std::string _camera_info_topic;
    std::string _image_detections_topic;
  };
  std::shared_ptr<Data> _data;
  tf2_ros::Buffer buffer;
  tf2_ros::TransformListener tfl;
};

#endif  // HUMANDETECTOR_HPP_
