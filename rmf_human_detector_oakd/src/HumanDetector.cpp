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

#include "HumanDetector.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>

namespace rmf_human_detector_oakd {

//==============================================================================
HumanDetector::HumanDetector(
  const rclcpp::NodeOptions& options)
: Node("rmf_human_detector_oakd", options)
{
  _data = std::make_shared<Data>();

  _data->pub = this->create_publisher<Obstacles>(
    "/rmf_obstacles",
    10);

  // Declare parameters
  RCLCPP_INFO(
    this->get_logger(),
    "Configuring rmf_human_detector_oakd...");
  const std::string nnPath = this->declare_parameter("blob_path", "");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting NN blob_path parameter to %s", nnPath.c_str());
  bool syncNN = this->declare_parameter("sync_nn", true);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting sync_nn parameter to %s", syncNN ? "True" : "False");
  _data->detector_name = this->declare_parameter(
    "detector_name", "rmf_human_detector_oakd");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting detector_name parameter to %s", _data->detector_name.c_str());
  _data->frame_id = this->declare_parameter("frame_id", "oakd_camera_link");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting frame_id parameter to %s", _data->frame_id.c_str());
  _data->level_name = this->declare_parameter("level_name", "L1");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting level_name parameter to %s", _data->level_name.c_str());
  _data->obstacle_classification = this->declare_parameter(
    "obstacle_classification", "human");
  RCLCPP_INFO(
    this->get_logger(),
    "Setting obstacle_classification parameter to %s",
    _data->obstacle_classification.c_str());
  _data->debug = this->declare_parameter("debug", false);
  RCLCPP_INFO(
    this->get_logger(),
    "Setting debug parameter to %s", _data->debug ? "True" : "False");

  // Initialize the OakD pipeline
  _data->pipeline = dai::Pipeline();

  // Add nodes for rgb, depth, and nn
  auto camRgb = _data->pipeline.create<dai::node::ColorCamera>();
  auto spatialDetectionNetwork =
    _data->pipeline.create<dai::node::MobileNetSpatialDetectionNetwork>();
  auto monoLeft = _data->pipeline.create<dai::node::MonoCamera>();
  auto monoRight = _data->pipeline.create<dai::node::MonoCamera>();
  auto stereo = _data->pipeline.create<dai::node::StereoDepth>();

  // Create connections
  auto xoutRgb = _data->pipeline.create<dai::node::XLinkOut>();
  auto xoutNN = _data->pipeline.create<dai::node::XLinkOut>();
  auto xoutBoundingBoxDepthMapping =
    _data->pipeline.create<dai::node::XLinkOut>();
  auto xoutDepth = _data->pipeline.create<dai::node::XLinkOut>();

  // Create message streams
  xoutRgb->setStreamName("rgb");
  xoutNN->setStreamName("detections");
  xoutBoundingBoxDepthMapping->setStreamName("boundingBoxDepthMapping");
  xoutDepth->setStreamName("depth");

  // Set properties
  camRgb->setPreviewSize(300, 300);
  camRgb->setResolution(
    dai::ColorCameraProperties::SensorResolution::THE_1080_P);
  camRgb->setInterleaved(false);
  camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

  monoLeft->setResolution(
    dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoLeft->setBoardSocket(dai::CameraBoardSocket::LEFT);
  monoRight->setResolution(
    dai::MonoCameraProperties::SensorResolution::THE_720_P);
  monoRight->setBoardSocket(dai::CameraBoardSocket::RIGHT);

  // Setting node configs
  stereo->initialConfig.setConfidenceThreshold(255);
  stereo->setDefaultProfilePreset(
    dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
  // Align depth map to the perspective of RGB camera, on which inference is done
  stereo->setDepthAlign(dai::CameraBoardSocket::RGB);
  stereo->setOutputSize(
    monoLeft->getResolutionWidth(), monoLeft->getResolutionHeight());

  spatialDetectionNetwork->setBlobPath(nnPath);
  spatialDetectionNetwork->setConfidenceThreshold(0.5f);
  spatialDetectionNetwork->input.setBlocking(false);
  spatialDetectionNetwork->setBoundingBoxScaleFactor(0.7);
  spatialDetectionNetwork->setDepthLowerThreshold(100);
  spatialDetectionNetwork->setDepthUpperThreshold(5000);
  // spatialDetectionNetwork->setNumInferenceThreads(2);

  // Link nodes
  monoLeft->out.link(stereo->left);
  monoRight->out.link(stereo->right);

  camRgb->preview.link(spatialDetectionNetwork->input);
  if (syncNN)
    spatialDetectionNetwork->passthrough.link(xoutRgb->input);
  else
    camRgb->preview.link(xoutRgb->input);

  spatialDetectionNetwork->out.link(xoutNN->input); // detections
  spatialDetectionNetwork->boundingBoxMapping.link(
    xoutBoundingBoxDepthMapping->input);

  stereo->depth.link(spatialDetectionNetwork->inputDepth);
  spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

  auto thread_fn =
    [data = _data]()
    {
      dai::Device device(data->pipeline);
      // Output queues will be used to get the rgb frames and nn data from the outputs defined above
      auto previewQueue = device.getOutputQueue("rgb", 4, false); // encoding, maxSize, blocking
      auto detectionNNQueue = device.getOutputQueue("detections", 4, false);
      auto xoutBoundingBoxDepthMappingQueue = device.getOutputQueue(
        "boundingBoxDepthMapping", 4, false);
      auto depthQueue = device.getOutputQueue("depth", 4, false);

      const auto color = cv::Scalar(255, 255, 255);

      while (data->run)
      {
        auto inDet = detectionNNQueue->get<dai::SpatialImgDetections>();
        if (inDet == nullptr)
          continue;

        auto depth = depthQueue->get<dai::ImgFrame>();
        cv::Mat depthFrame = depth->getFrame();  // depthFrame values are in millimeters

        const auto& detections = inDet->detections;
        const auto& boundingBoxMapping =
          xoutBoundingBoxDepthMappingQueue->get<dai::SpatialLocationCalculatorConfig>();
        const auto& roiDatas = boundingBoxMapping->getConfigData();

        const auto num_detections = detections.size();
        const auto num_rois = roiDatas.size();
        if (num_detections != num_rois)
        {
          // std::cout << "Number of detected bounding boxes [" << num_detections
          //           << "] does not match number of detected ROIs ["
          //           << num_rois << "]" << std::endl;
          continue;
        }

        // TODO(YV) track objects and update on when there is significant change in position

        cv::Mat depthFrameColor;
        cv::Mat frame;
        if (data->debug)
        {
          const auto& inPreview = previewQueue->get<dai::ImgFrame>();
          frame = inPreview->getCvFrame();
          cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF,
            CV_8UC1);
          cv::equalizeHist(depthFrameColor, depthFrameColor);
          cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);
        }

        auto detection_it = detections.begin();
        auto roi_it = roiDatas.begin();

        auto obstacles = std::make_unique<Obstacles>();
        std::size_t obstacle_count = 0;
        for (; detection_it != detections.end(); ++detection_it, ++roi_it)
        {
          ++obstacle_count;
          auto roi = roi_it->roi;
          roi = roi.denormalize(depthFrame.cols, depthFrame.rows);
          if (detection_it->label >= data->labels.size())
          {
            // std::cout << "[Warn] Detected unclassified object." << std::endl;
            continue;
          }
          const std::string& label = data->labels[detection_it->label];
          if (label != "person")
          {
            // std::cout << "Ignoring non-human detection: " << label << std::endl;
            continue;
          }


          // TODO(YV): Estimate actual width and height from spatial data.
          const double human_width = 0.6;
          const double human_height = 1.8;
          const double spatial_x = detection_it->spatialCoordinates.x / 1000.0;
          const double spatial_y = detection_it->spatialCoordinates.y / 1000.0;
          const double spatial_z = detection_it->spatialCoordinates.z / 1000.0;

          if (data->debug)
          {
            const auto& topLeft = roi.topLeft();
            const auto& bottomRight = roi.bottomRight();
            const auto xmin = (int)(topLeft.x);
            const auto ymin = (int)(bottomRight.y);
            const auto xmax = (int)(bottomRight.x);
            const auto ymax = (int)(topLeft.y);
            cv::rectangle(
              depthFrameColor,
              cv::Rect(cv::Point(xmin, ymin),
              cv::Point(xmax, ymax)),
              color,
              cv::FONT_HERSHEY_SIMPLEX);

            const auto frame_width = frame.cols;
            const auto frame_height = frame.rows;
            const auto x1 = (int)(detection_it->xmin * frame_width);
            const auto x2 = (int)(detection_it->xmax * frame_width);
            const auto y1 = (int)(detection_it->ymin * frame_height);
            const auto y2 = (int)(detection_it->ymax * frame_height);
            cv::putText(frame, label, cv::Point(x1 + 10,
              y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5,
              255);
            cv::putText(frame, std::to_string(
                detection_it->confidence*100)+"%", cv::Point(x1 + 10,
              y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5,
              255);
            cv::putText(frame, std::to_string(spatial_x)+"m",
              cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::putText(frame, std::to_string(spatial_y)+"m",
              cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::putText(frame, std::to_string(spatial_z)+"m",
              cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            cv::rectangle(frame, cv::Point(x1, y1), cv::Point(x2,
              y2), color,
              cv::FONT_HERSHEY_SIMPLEX);
          }

          rmf_obstacle_msgs::msg::Obstacle obstacle;
          obstacle.header.frame_id = data->frame_id;
          // TODO(YV): Stamp
          obstacle.id = obstacle_count;
          obstacle.source = data->detector_name;
          obstacle.level_name = data->level_name;
          obstacle.classification = data->obstacle_classification;
          obstacle.bbox.center.position.x = spatial_x;
          obstacle.bbox.center.position.y = spatial_y;
          obstacle.bbox.center.position.z = spatial_z;
          obstacle.bbox.size.x = human_width;
          obstacle.bbox.size.y = human_width;
          obstacle.bbox.size.z = human_height;
          obstacle.action = rmf_obstacle_msgs::msg::Obstacle::ACTION_ADD;
          obstacles->obstacles.push_back(obstacle);
        }

        if (!obstacles->obstacles.empty())
        {
          data->pub->publish(std::move(obstacles));
        }

        if (data->debug)
        {
          cv::imshow("depth", depthFrameColor);
          cv::imshow("preview", frame);
          cv::waitKey(1);
        }
      }

    };

  _data->detection_thread = std::thread(thread_fn);

}

//==============================================================================
HumanDetector::~HumanDetector()
{
  if (_data->detection_thread.joinable())
  {
    _data->run = false;
    _data->detection_thread.join();
  }
}

} // rmf_human_detector_oakd


RCLCPP_COMPONENTS_REGISTER_NODE(rmf_human_detector_oakd::HumanDetector)
