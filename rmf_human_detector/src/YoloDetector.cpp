#include <memory>
#include <cstdio>
#include <fstream>

// ROS includes
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// Project includes
#include <rmf_human_detector/YoloDetector.hpp>

// Namespaces
using namespace cv;
using namespace std;
using namespace cv::dnn;

// Text parameters
const float FONT_SCALE = 0.4;
const int FONT_FACE = FONT_HERSHEY_SIMPLEX;
const int FONT_THICKNESS = 1;
const int RECT_THICKNESS = 2;

// Colors
const Scalar BLACK = Scalar(0, 0, 0);
const Scalar BLUE = Scalar(255, 178, 50);
const Scalar YELLOW = Scalar(0, 255, 255);
const Scalar RED = Scalar(0, 0, 255);

YoloDetector::YoloDetector(std::shared_ptr<Config> config)
: _config(config)
{
  _net = readNet(_config->nn_filepath);
  if (_config->use_gpu)
  {
    _net.setPreferableBackend(DNN_BACKEND_CUDA);
    _net.setPreferableTarget(DNN_TARGET_CUDA);
  }

  ifstream ifs(_config->labels_filepath);
  string line;
  while (getline(ifs, line))
  {
    _class_list.push_back(line);
  }

  if (_config->visualize)
  {
    cv::namedWindow(_config->camera_name, cv::WINDOW_AUTOSIZE);
  }
}

YoloDetector::~YoloDetector()
{
  if (_config->visualize)
  {
    cv::destroyWindow(_config->camera_name);
  }
}

std::pair<YoloDetector::Obstacles,
  sensor_msgs::msg::Image> YoloDetector::image_cb(
  const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  // bridge from ROS image type to OpenCV image type
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  Mat original_image;
  if (msg->encoding == sensor_msgs::image_encodings::BGR8)
  {
    original_image = cv_ptr->image;
  }
  else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
  {
    cvtColor(cv_ptr->image, original_image, COLOR_RGB2BGR);
  }

  // format image, forward propagate and post process
  Mat image = format_yolov5(original_image);
  vector<Mat> detections = detect(image);
  Obstacles rmf_obstacles = post_process(original_image, image, detections);

  // convert OpenCV image with detections back to ROS image type
  auto image_detections = to_ros_image(image);

  return std::make_pair(rmf_obstacles, image_detections);
}

void YoloDetector::camera_pose_cb(const geometry_msgs::msg::Transform& msg)
{
  _camera_pose = msg;
}

Mat YoloDetector::format_yolov5(const Mat& source)
{
  int col = source.cols;
  int row = source.rows;
  int _max = MAX(col, row);

  Mat result = Mat::zeros(_max, _max, CV_8UC3);
  source.copyTo(result(Rect(0, 0, col, row)));
  return result;
}

vector<Mat> YoloDetector::detect(Mat& input_image)
{
  // Convert to blob
  Mat blob;
  blobFromImage(input_image, blob, 1./255.,
    Size(INPUT_WIDTH, INPUT_HEIGHT),
    Scalar(), true, false);

  _net.setInput(blob);

  // Forward propagate
  vector<Mat> outputs;
  _net.forward(outputs, _net.getUnconnectedOutLayersNames());

  return outputs;
}

YoloDetector::Obstacles YoloDetector::post_process(
  const Mat& original_image,
  Mat& image,
  vector<Mat>& detections)
{
  // Initialize vectors to hold respective outputs while unwrapping detections
  vector<int> class_ids;
  vector<float> confidences;
  vector<Rect> boxes;
  // Resizing factor
  float x_factor = image.cols / INPUT_WIDTH;
  float y_factor = image.rows / INPUT_HEIGHT;

  // detections[0] is expected to be 1 x 25200 x 85
  // yolov5s outputs 25200 possible bounding boxes
  // every bounding box is defined by 85 entries
  // the 85 entries are: px, py, w, h, confidence, 80 class_scores
  const int cols = 5 + _class_list.size();
  const int rows = detections[0].total()/cols;
  float* data = (float*)detections[0].data;
  // Iterate through 25200 detections
  for (int i = 0; i < rows; ++i, data += cols)
  {
    float confidence = data[4];
    // Discard bad detections and continue
    if (confidence >= _config->confidence_threshold)
    {
      float* classes_scores = data + 5;
      // Create a 1x80 Mat and store class scores of 80 classes
      Mat scores(1, _class_list.size(), CV_32FC1, classes_scores);
      // Perform minMaxLoc and acquire the index of best class  score
      Point class_id;
      double max_class_score;
      minMaxLoc(scores, 0, &max_class_score, 0, &class_id);
      // Continue if the class score is above the threshold
      // class_id.x == 0 corresponds to objects labelled "person"
      if (max_class_score > _config->score_threshold && class_id.x == 0)
      {
        // Store class ID and confidence in the pre-defined respective vectors
        confidences.push_back(confidence);
        class_ids.push_back(class_id.x);
        // Center
        float px = data[0];
        float py = data[1];
        // Box dimension
        float w = data[2];
        float h = data[3];
        // Bounding box coordinates
        int left = int((px - 0.5 * w) * x_factor);
        int top = int((py - 0.5 * h) * y_factor);
        int width = int(w * x_factor);
        int height = int(h * y_factor);
        // Store good detections in the boxes vector
        boxes.push_back(Rect(left, top, width, height));
      }
    }
  }

  // Perform Non-Maximum Suppression
  vector<int> indices; // will contain indices of final bounding boxes
  NMSBoxes(boxes, confidences, _config->score_threshold, _config->nms_threshold,
    indices);

  // use indices vector to get the final vectors
  vector<int> final_class_ids;
  vector<float> final_confidences;
  vector<Rect> final_boxes;
  for (auto i : indices)
  {
    final_class_ids.push_back(class_ids[i]);
    final_confidences.push_back(confidences[i]);
    final_boxes.push_back(boxes[i]);
  }

  // draw to image
  drawing(
    original_image,
    image,
    final_class_ids,
    final_confidences,
    final_boxes
  );

  if (_config->visualize)
  {
    // OpenCV display
    imshow(_config->camera_name, image);
    waitKey(3);
  }

  // generate rmf_obstacles
  auto rmf_obstacles = to_rmf_obstacles(
    final_class_ids,
    final_boxes
  );

  return rmf_obstacles;
}

sensor_msgs::msg::Image YoloDetector::to_ros_image(const cv::Mat& image)
{
  cv_bridge::CvImage img_bridge;
  std_msgs::msg::Header header;
  img_bridge =
    cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
  sensor_msgs::msg::Image image_detections;
  img_bridge.toImageMsg(image_detections);
  return image_detections;
}

void YoloDetector::add_level(const std::string level_name,
  const double level_elevation)
{
  _level_to_elevation.insert({level_name, level_elevation});
}

Plane YoloDetector::get_ground_plane()
{
  // get inverse of camera_tf
  tf2::Transform camera_tf, camera_inv_tf;
  tf2::fromMsg(_camera_pose, camera_tf);
  camera_inv_tf = camera_tf.inverse();
  geometry_msgs::msg::Transform camera_inv_tf_msg;
  camera_inv_tf_msg = tf2::toMsg(camera_inv_tf);


  // transform plane normal vector (0,0,1) from world coordinates to camera coordinates
  geometry_msgs::msg::TransformStamped camera_inv_tf_stamped;
  camera_inv_tf_stamped.transform = camera_inv_tf_msg;
  geometry_msgs::msg::Vector3Stamped in, out;
  in.vector = geometry_msgs::build<geometry_msgs::msg::Vector3>()
    .x(0)
    .y(0)
    .z(1);

  // do transformation
  tf2::doTransform(in, out, camera_inv_tf_stamped);

  // get level_elevation
  double level_elevation = 0.0;
  auto it = _level_to_elevation.find(_config->camera_level);
  if (it != _level_to_elevation.end())
  {
    level_elevation = it->second;
  }
  // transform point in plane (0,0,level_elevation)
  // from world coordinates to camera coordinates
  geometry_msgs::msg::PointStamped in2, out2;
  in2.point = geometry_msgs::build<geometry_msgs::msg::Point>()
    .x(0)
    .y(0)
    .z(level_elevation);

  // do transformation
  tf2::doTransform(in2, out2, camera_inv_tf_stamped);

  Eigen::Vector3f plane_normal(out.vector.x, out.vector.y, out.vector.z);
  Eigen::Vector3f point_in_plane(out2.point.x, out2.point.y, out2.point.z);
  return Plane(plane_normal, point_in_plane);
}

YoloDetector::Obstacles YoloDetector::to_rmf_obstacles(
  const vector<int>& final_class_ids,
  const vector<Rect>& final_boxes)
{
  auto rmf_obstacles = Obstacles();
  rmf_obstacles.obstacles.reserve(final_boxes.size());

  image_geometry::PinholeCameraModel model;
  model.fromCameraInfo(_config->camera_info);

  // prepare obstacle_msg objects and add to rmf_obstacles
  for (size_t i = 0; i < final_boxes.size(); i++)
  {
    // construct 3d ray from camera to middle of the bottom edge of bounding box
    Rect box = final_boxes[i];
    int left = box.x;
    int top = box.y;
    int width = box.width;
    int height = box.height;
    Point point_on_ground = Point(left + (width/2), top + height);
    cv::Point3d ray = model.projectPixelTo3dRay(point_on_ground);

    // change from image coordinates to camera coordinates
    cv::Point3d temp = ray;
    ray.x = temp.z;
    ray.y = -temp.x;
    ray.z = -temp.y;

    // construct ground plane in camera coordinates
    Plane plane = get_ground_plane();

    // obstacle is at the intersection point of ray and plane
    Eigen::Vector3f n = plane.getNormal();
    Eigen::Vector3f d(ray.x, ray.y, ray.z);
    double alpha = - plane.getD() / n.dot(d);
    Eigen::Vector3f intersection(alpha * d);

    // convert intersection from camera coordinates to world coordinates
    geometry_msgs::msg::TransformStamped camera_tf_stamped;
    camera_tf_stamped.transform = _camera_pose;
    geometry_msgs::msg::PointStamped in, out;
    in.point = geometry_msgs::build<geometry_msgs::msg::Point>()
      .x(intersection.x())
      .y(intersection.y())
      .z(intersection.z());

    // do transformation
    tf2::doTransform(in, out, camera_tf_stamped);
    Point3d obstacle(out.point.x, out.point.y, out.point.z);

    // populate rmf_obstacle
    auto rmf_obstacle = Obstacle();
    rmf_obstacle.header.frame_id = _config->camera_name;
    rmf_obstacle.id = i;
    rmf_obstacle.source = _config->camera_name;
    rmf_obstacle.level_name = _config->camera_level;
    rmf_obstacle.classification = _class_list[final_class_ids[i]];
    rmf_obstacle.bbox.center.position.x = obstacle.x;
    rmf_obstacle.bbox.center.position.y = obstacle.y;
    rmf_obstacle.bbox.center.position.z = obstacle.z;
    rmf_obstacle.bbox.size.x = 1.0;
    rmf_obstacle.bbox.size.y = 1.0;
    rmf_obstacle.bbox.size.z = 2.0;
    rmf_obstacle.lifetime.sec = _config->obstacle_lifetime_sec;

    rmf_obstacles.obstacles.push_back(rmf_obstacle);
  }
  return rmf_obstacles;
}

void YoloDetector::drawing(
  const Mat& original_image,
  Mat& image,
  const vector<int>& final_class_ids,
  const vector<float>& final_confidences,
  const vector<Rect>& final_boxes)
{
  for (size_t i = 0; i < final_class_ids.size(); i++)
  {
    Rect box = final_boxes[i];
    int left = box.x;
    int top = box.y;
    int width = box.width;
    int height = box.height;
    // Top left corner
    Point tlc = Point(left, top);
    // Bottom right corner
    Point brc = Point(left + width, top + height);
    // Draw bounding box
    rectangle(image, tlc, brc, BLUE, RECT_THICKNESS);

    // Get the label for the class name and its confidence
    string label = format("%.2f", final_confidences[i]);
    label = _class_list[final_class_ids[i]] + ":" + label;
    // Draw class labels
    draw_label(image, label, left, top);
  }

  // Put efficiency information
  // The function getPerfProfile returns the overall time for inference(t) and the timings for each of the layers(in layersTimes)
  vector<double> layersTimes;
  double freq = getTickFrequency() / 1000;
  double t = _net.getPerfProfile(layersTimes) / freq;
  string label = format("Inference time : %.2f ms", t);
  putText(image, label, Point(20, 40), FONT_FACE, FONT_SCALE, RED);

  // Slicing to crop the image
  image = image(Range(0, original_image.rows), Range(0, original_image.cols));
}

void YoloDetector::draw_label(Mat& input_image, string label, int left, int top)
{
  // Display the label at the top of the bounding box
  int baseLine;
  Size label_size =
    getTextSize(label, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseLine);
  top = max(0, top - label_size.height);
  left = max(0, left - RECT_THICKNESS/2);
  // Top left corner
  Point tlc = Point(left, top);
  // Bottom right corner
  Point brc = Point(left + label_size.width, top + label_size.height);
  // Draw blue text background
  rectangle(input_image, tlc, brc, BLUE, FILLED);
  // Put the label on the blue text background
  putText(
    input_image,
    label,
    Point(left, top + label_size.height),
    FONT_FACE,
    FONT_SCALE,
    YELLOW,
    FONT_THICKNESS
  );
}