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
#include <rclcpp_components/register_node_macro.hpp>

#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace rmf_human_detector {

class HumanDetectorData
{
public:
  HumanDetectorData(HumanDetector* node)
  : buffer(node->get_clock()), tfl(buffer, node, true)
  {
  }

  /// \brief Image subscriber
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr _image_sub;

  /// \brief Camera info subscriber
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _camera_info_sub;

  /// \brief RMF obstacles publisher
  rclcpp::Publisher<rmf_obstacle_msgs::msg::Obstacles>::SharedPtr _obstacles_pub;

  /// \brief Detections publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr _image_detections_pub;

  /// \brief YOLO detector
  std::shared_ptr<YoloDetector> _detector;

  /// \brief Camera name
  std::string _camera_name;

  /// \brief Camera parent name
  std::string _camera_parent_name;

  /// \brief Camera topic name
  std::string _camera_image_topic;

  /// \brief Camera info topic name
  std::string _camera_info_topic;

  /// \brief Image detections topic
  std::string _image_detections_topic;

  /// \brief tf2 ros2 buffer
  tf2_ros::Buffer buffer;

  /// \brief Transform listener
  tf2_ros::TransformListener tfl;
};

HumanDetector::HumanDetector(const rclcpp::NodeOptions& options)
: Node("human_detector", options),
  _data(std::make_shared<HumanDetectorData>(this))
{
  make_detector();

  _data->_obstacles_pub =
    this->create_publisher<rmf_obstacle_msgs::msg::Obstacles>(
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

      // rmf_obstacles.header.stamp = this->get_clock()->now();
      // rmf_obstacles.header.frame_id = _data->_camera_name;

      // populate fields like time stamp, etc
      for (auto& obstacle : rmf_obstacles.obstacles)
      {
        obstacle.header.stamp = this->get_clock()->now();
      }

      // publish rmf_obstacles & image with bounding boxes
      _data->_obstacles_pub->publish(std::move(rmf_obstacles));
      _data->_image_detections_pub->publish(std::move(image_detections));
      if (_data->buffer.canTransform(
        _data->_camera_parent_name,
        _data->_camera_name,
        rclcpp::Time()))
      {
        auto tStamped = _data->buffer.lookupTransform(
          _data->_camera_parent_name,
          _data->_camera_name,
          rclcpp::Time());
        geometry_msgs::msg::Transform t;
        t.rotation = tStamped.transform.rotation;
        t.translation = tStamped.transform.translation;
        _data->_detector->camera_pose_cb(t);
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

  const std::string level_name = this->declare_parameter("level_name", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter level_name parameter to %s", level_name.c_str());
  const int level_elevation = this->declare_parameter("level_elevation", 1);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter level_elevation parameter to %d", level_elevation);

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

  // make detector
  YoloDetector::Config config = {
    _data->_camera_name,
    std::nullopt,
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
  _data->_detector->add_level(level_name, level_elevation);

  _data->_camera_info_sub =
    this->create_subscription<sensor_msgs::msg::CameraInfo>(
    _data->_camera_info_topic,
    rclcpp::SensorDataQoS(),
    [&](const sensor_msgs::msg::CameraInfo& msg)
    {
      _data->_detector->set_camera_info(msg);
    });
}

HumanDetector::~HumanDetector()
{
}
}

RCLCPP_COMPONENTS_REGISTER_NODE(rmf_human_detector::HumanDetector)
