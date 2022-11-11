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

#include "HumanDetector.hpp"

#include <filesystem>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/wait_for_message.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

HumanDetector::HumanDetector()
: Node("human_detector"), _data(std::make_shared<Data>()),
  buffer(this->get_clock()), tfl(buffer, this, true)
{
  make_detector();

  _data->_obstacles_pub = this->create_publisher<Obstacles>(
    "/rmf_obstacles",
    rclcpp::SensorDataQoS()
  );

  _data->_image_detections_pub =
    this->create_publisher<sensor_msgs::msg::Image>(
    _data->_image_detections_topic,
    rclcpp::SensorDataQoS()
    );

  _data->_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    _data->_camera_image_topic,
    rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
      // perform detections
      auto [rmf_obstacles, image_detections] = _data->_detector->image_cb(msg);

      // populate fields like time stamp, etc
      for (auto& obstacle : rmf_obstacles.obstacles)
      {
        obstacle.header.stamp = this->get_clock()->now();
      }

      // publish rmf_obstacles & image with bounding boxes
      _data->_obstacles_pub->publish(std::move(rmf_obstacles));
      _data->_image_detections_pub->publish(std::move(image_detections));
      if (buffer.canTransform(
            _data->_camera_parent_name,
            _data->_camera_name,
            rclcpp::Time()))
      {
        auto tStamped = buffer.lookupTransform(
          _data->_camera_parent_name,
          _data->_camera_name,
          rclcpp::Time());
        geometry_msgs::msg::Transform t;
        t.rotation = tStamped.transform.rotation;
        t.translation = tStamped.transform.translation;
        _data->_detector->camera_pose_cb(t);
      }
    });

  auto qos_profile = rclcpp::QoS(10);
  qos_profile.transient_local();
  _data->_building_map_sub =
    this->create_subscription<rmf_building_map_msgs::msg::BuildingMap>(
    "/map",
    qos_profile,
    [=](const rmf_building_map_msgs::msg::BuildingMap::ConstSharedPtr& msg)
    {
      if (msg->levels.empty())
      {
        RCLCPP_ERROR(this->get_logger(), "Received empty building map");
        return;
      }

      for (const auto& level : msg->levels)
      {
        _data->_detector->add_level(level.name, level.elevation);
      }
    });
}

void HumanDetector::make_detector()
{
  _data->_camera_name = this->declare_parameter("camera_name", "camera1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter camera_name to %s", _data->_camera_name.c_str()
  );
  _data->_camera_image_topic = _data->_camera_name;
  _data->_camera_info_topic = _data->_camera_name + "/camera_info";
  _data->_image_detections_topic = _data->_camera_name + "/image_detections";

  _data->_camera_parent_name =
    this->declare_parameter("camera_parent_name", "sim_world");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter camera_parent_name to %s",
    _data->_camera_parent_name.c_str()
  );

  const bool visualize = this->declare_parameter("visualize", true);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter visualize to %s", visualize ? "true" : "false"
  );

  const std::string camera_level =
    this->declare_parameter("camera_level", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter camera_level to %s", camera_level.c_str()
  );

  const int obstacle_lifetime_sec =
    this->declare_parameter("obstacle_lifetime_sec", 1);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter obstacle_lifetime_sec to %d", obstacle_lifetime_sec
  );

  const std::string nn_filepath = this->declare_parameter("nn_filepath", "");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter nn_filepath to %s", nn_filepath.c_str()
  );

  std::filesystem::path model_file(nn_filepath);
  if (!std::filesystem::exists(model_file))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Model file %s does not exist, follow README instructions to get the file",
      model_file.c_str()
    );
    return;
  }

  const std::string labels_filepath =
    this->declare_parameter("labels_filepath", "");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter labels_filepath to %s", labels_filepath.c_str()
  );

  std::filesystem::path labels_file(labels_filepath);
  if (!std::filesystem::exists(labels_file))
  {
    RCLCPP_INFO(
      this->get_logger(),
      "Labels file %s does not exist, follow README instructions to get the file",
      labels_file.c_str()
    );
    return;
  }

  const bool use_gpu = this->declare_parameter("use_gpu", false);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter use_gpu to %s", use_gpu ? "true" : "false"
  );

  const float confidence_threshold =
    this->declare_parameter("confidence_threshold", 0.25);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter confidence_threshold to %f", confidence_threshold
  );

  const float score_threshold =
    this->declare_parameter("score_threshold", 0.45);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter score_threshold to %f", score_threshold
  );

  const float nms_threshold = this->declare_parameter("nms_threshold", 0.45);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter nms_threshold to %f", nms_threshold
  );

  // get one camera_info msg
  sensor_msgs::msg::CameraInfo camera_info;
  std::shared_ptr<rclcpp::Node> temp_node =
    std::make_shared<rclcpp::Node>("wait_for_msg_node");
  rclcpp::wait_for_message(camera_info, temp_node, _data->_camera_info_topic);

  // make detector
  YoloDetector::Config config = {
    _data->_camera_name,
    camera_info,
    visualize,
    camera_level,
    obstacle_lifetime_sec,
    nn_filepath,
    labels_filepath,
    use_gpu,
    confidence_threshold,
    score_threshold,
    nms_threshold};

  _data->_detector = std::make_shared<YoloDetector>(config);
}

HumanDetector::~HumanDetector()
{
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting HumanDetector node" << std::endl;
  rclcpp::spin(std::make_shared<HumanDetector>());
  rclcpp::shutdown();
  return 0;
}
