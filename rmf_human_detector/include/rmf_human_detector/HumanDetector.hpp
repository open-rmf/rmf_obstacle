#ifndef HUMANDETECTOR_HPP
#define HUMANDETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <rmf_obstacle_msgs/msg/obstacles.hpp>
#include <rmf_building_map_msgs/msg/building_map.hpp>
#include <rmf_human_detector/YoloDetector.hpp>

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
    std::string _camera_pose_topic;
    std::string _camera_info_topic;
    std::string _image_detections_topic;
  };
  std::shared_ptr<Data> _data;
};

#endif // HUMANDETECTOR_HPP
