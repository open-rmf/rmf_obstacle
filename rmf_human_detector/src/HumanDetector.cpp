#include <rclcpp/wait_for_message.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <rmf_human_detector/HumanDetector.hpp>

HumanDetector::HumanDetector()
: Node("human_detector"), _data(std::make_shared<Data>())
{
  make_detector();

  _data->_image_detections_pub =
    this->create_publisher<sensor_msgs::msg::Image>(
    _data->_image_detections_topic,
    rclcpp::QoS(10).reliable()
    );
  _data->_detector->set_image_detections_pub(_data->_image_detections_pub);

  _data->_obstacles_pub = this->create_publisher<Obstacles>(
    "/rmf_obstacles",
    rclcpp::SensorDataQoS()
  );

  _data->_image_sub = this->create_subscription<sensor_msgs::msg::Image>(
    _data->_camera_image_topic,
    rclcpp::SensorDataQoS(),
    [=](const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
      // perform detections
      auto rmf_obstacles_msg = _data->_detector->image_cb(msg);

      // populate fields like time stamp, etc
      for (auto& obstacle : rmf_obstacles_msg.obstacles)
      {
        obstacle.header.stamp = this->get_clock()->now();
      }

      // publish rmf_obstacles_msg
      _data->_obstacles_pub->publish(rmf_obstacles_msg);
    });

  _data->_camera_pose_sub = this->create_subscription<tf2_msgs::msg::TFMessage>(
    _data->_camera_pose_topic,
    rclcpp::SensorDataQoS(),
    [=](const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg)
    {
      for (auto& transformStamped : msg->transforms)
      {
        if (transformStamped.header.frame_id == _data->_camera_parent_name
        && transformStamped.child_frame_id ==  _data->_camera_name)
        {
          _data->_detector->camera_pose_cb(transformStamped.transform);
        }
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
  _data->_camera_image_topic = _data->_camera_name + "/image_rect";
  _data->_camera_pose_topic = _data->_camera_name + "/pose";
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

  const bool camera_static = this->declare_parameter("camera_static", true);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter camera_static to %s", camera_static ? "true" : "false"
  );

  const std::string nn_filepath = this->declare_parameter("nn_filepath", "");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter nn_filepath to %s", nn_filepath.c_str()
  );

  const std::string labels_filepath =
    this->declare_parameter("labels_filepath", "");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter labels_filepath to %s", nn_filepath.c_str()
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

  const float confidence_threshold =
    this->declare_parameter("confidence_threshold", 0.25);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting parameter confidence_threshold to %f", confidence_threshold
  );

  // get one camera_info msg
  sensor_msgs::msg::CameraInfo camera_info;
  std::shared_ptr<rclcpp::Node> temp_node =
    std::make_shared<rclcpp::Node>("wait_for_msg_node");
  rclcpp::wait_for_message(camera_info, temp_node, _data->_camera_info_topic);

  // calculate camera fov
  float f_x = camera_info.p[0];
  float fov_x = 2 * atan2(camera_info.width, (2*f_x) );

  // make detector
  std::shared_ptr<YoloDetector::Config> config =
    std::make_shared<YoloDetector::Config>(
    YoloDetector::Config{
      _data->_camera_name,
      visualize,
      camera_static,
      fov_x,
      nn_filepath,
      labels_filepath,
      score_threshold,
      nms_threshold,
      confidence_threshold,
    }
    );

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
