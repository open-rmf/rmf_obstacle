#ifndef YOLODETECTOR_HPP
#define YOLODETECTOR_HPP

#include <string>
#include <Eigen/Geometry>

// ROS includes
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/transform.hpp>

// OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

// Project includes
#include <rmf_obstacle_msgs/msg/obstacle.hpp>
#include <rmf_obstacle_msgs/msg/obstacles.hpp>

class Plane
{
public:
  Plane(Eigen::Vector3f normal, Eigen::Vector3f p)
  : _normal(normal.normalized()), _d(- normal.dot(p) / normal.norm()) {}

  Eigen::Vector3f getNormal() { return _normal; }

  double getD() { return _d; }

private:
  Eigen::Vector3f _normal;
  double _d;
};

class YoloDetector
{
public:
  using Obstacle = rmf_obstacle_msgs::msg::Obstacle;
  using Obstacles = rmf_obstacle_msgs::msg::Obstacles;

  // User Configurations
  struct Config
  {
    // Camera configurations
    std::string camera_name;
    sensor_msgs::msg::CameraInfo camera_info;
    const bool visualize = true;
    std::string camera_level;
    const int obstacle_lifetime_sec;
    // YoloDetector configurations
    const std::string nn_filepath;
    const std::string labels_filepath;
    const bool use_gpu = false;
    const float score_threshold;
    const float nms_threshold;
    const float confidence_threshold;
  };

  YoloDetector(std::shared_ptr<Config> config);
  ~YoloDetector();

  std::pair<Obstacles, sensor_msgs::msg::Image> image_cb(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg);

  void camera_pose_cb(const geometry_msgs::msg::Transform& msg);

private:

  // Yolov5s Constants
  // Camera image gets scaled to this size before inference
  const float INPUT_WIDTH = 640.0;
  const float INPUT_HEIGHT = 640.0;

  // Members
  std::vector<std::string> _class_list;
  cv::dnn::Net _net;
  std::shared_ptr<Config> _config;
  geometry_msgs::msg::Transform _camera_pose;

  // Methods
  cv::Mat format_yolov5(const cv::Mat& source);

  std::vector<cv::Mat> detect(cv::Mat& input_image);

  Obstacles post_process(const cv::Mat& original_image, cv::Mat& image,
    std::vector<cv::Mat>& detections);

  Obstacles to_rmf_obstacles(
    const std::vector<int>& final_class_ids,
    const std::vector<cv::Rect>& final_boxes);

  Plane get_ground_plane();

  sensor_msgs::msg::Image to_ros_image(const cv::Mat& image);

  void drawing(const cv::Mat& original_image, cv::Mat& image,
    const std::vector<int>& final_class_ids,
    const std::vector<float>& final_confidences,
    const std::vector<cv::Rect>& final_boxes);

  void draw_label(cv::Mat& input_image, std::string label, int left, int top);

};

#endif // YOLODETECTOR_HPP