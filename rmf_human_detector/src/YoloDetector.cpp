#include <memory>
#include <cstdio>
#include <fstream>
#include <cmath>
#include <filesystem>
#include <memory>

// ROS includes
#include <std_msgs/msg/header.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// OpenCV includes
#include <cv_bridge/cv_bridge.h>

// Project includes
#include <rmf_obstacle_msgs/msg/bounding_box3_d.hpp>
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
const Scalar BLACK = Scalar(0,0,0);
const Scalar BLUE = Scalar(255, 178, 50);
const Scalar YELLOW = Scalar(0, 255, 255);
const Scalar RED = Scalar(0,0,255);

YoloDetector::YoloDetector(std::shared_ptr<Config> config) : _config(config)
{
    calibrate();

    _net = readNet(_config->nn_filepath);

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

YoloDetector::Obstacles YoloDetector::image_cb(const sensor_msgs::msg::Image::ConstSharedPtr &msg)
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

    return rmf_obstacles;
}

void YoloDetector::camera_pose_cb(const geometry_msgs::msg::Transform &msg)
{
    _camera_pose = msg;

    if (_config->camera_static)
        return;

    // call calibrate() only if the camera is moving
    calibrate();
}

void YoloDetector::set_image_detections_pub(rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_detections_pub)
{
    _image_detections_pub = image_detections_pub;
}

void YoloDetector::calibrate()
{
    // calculate camera pitch
    tf2::Quaternion q(
        _camera_pose.rotation.x,
        _camera_pose.rotation.y,
        _camera_pose.rotation.z,
        _camera_pose.rotation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Suppose an unobstructed standing person model is placed exactly in the middle
    // of the camera image, such that the person's midpoint and image midpoint coincide.
    // _d_param = real world distance (metres) along the floor from camera to standing person
    if (std::abs(pitch) < 0.001)
    {
        // TO DO: handle when the camera pitch is zero
        _d_param = 5.28;
    }
    else
    {
        const float STANDING_PERSON_HEIGHT = 1.8;
        _d_param = (_camera_pose.translation.z - STANDING_PERSON_HEIGHT/2)/tan(pitch);
    }

    // CAMERA_TO_HUMAN = real world direct distance from camera to standing person's midpoint
    const float CAMERA_TO_HUMAN = _d_param/cos(pitch);

    // _w_param = width of real world (metres) within the image, at the location of the standing person
    _w_param = CAMERA_TO_HUMAN*tan(_config->camera_afov/2);

}

Mat YoloDetector::format_yolov5(const Mat &source)
{
    int col = source.cols;
    int row = source.rows;
    int _max = MAX(col, row);

    Mat result = Mat::zeros(_max, _max, CV_8UC3);
    source.copyTo(result(Rect(0, 0, col, row)));
    return result;
}

vector<Mat> YoloDetector::detect(Mat &input_image)
{
    // Convert to blob
    Mat blob;
    blobFromImage(input_image, blob, 1./255., Size(INPUT_WIDTH, INPUT_HEIGHT), Scalar(), true, false);

    _net.setInput(blob);

    // Forward propagate
    vector<Mat> outputs;
    _net.forward(outputs, _net.getUnconnectedOutLayersNames());

    return outputs;
}

YoloDetector::Obstacles YoloDetector::post_process(const Mat &original_image, Mat &image, vector<Mat> &detections)
{
    // Initialize vectors to hold respective outputs while unwrapping detections
    vector<int> class_ids;
    vector<float> confidences;
    vector<Rect> boxes;
    vector<Point> centroids; // image coordinates, (int, int)
    // Resizing factor
    float x_factor = image.cols / INPUT_WIDTH;
    float y_factor = image.rows / INPUT_HEIGHT;

    // detections[0] is expected to be 1 x 25200 x 85
    // yolov5s outputs 25200 possible bounding boxes
    // every bounding box is defined by 85 entries
    // the 85 entries are: px, py, w, h, confidence, 80 class_scores
    const int cols = 5 + _class_list.size();
    const int rows = detections[0].total()/cols;
    float *data = (float *)detections[0].data;
    // Iterate through 25200 detections
    for (int i = 0; i < rows; ++i, data += cols)
    {
        float confidence = data[4];
        // Discard bad detections and continue
        if (confidence >= _config->confidence_threshold)
        {
            float * classes_scores = data + 5;
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
                centroids.push_back(Point(px * x_factor, py * y_factor));
            }
        }
    }

    // Perform Non-Maximum Suppression
    vector<int> indices; // will contain indices of final bounding boxes
    NMSBoxes(boxes, confidences, _config->score_threshold, _config->nms_threshold, indices);

    // use indices vector to get the final vectors
    vector<int> final_class_ids;
    vector<float> final_confidences;
    vector<Rect> final_boxes;
    vector<Point> final_centroids;
    for (auto i : indices)
    {
        final_class_ids.push_back(class_ids[i]);
        final_confidences.push_back(confidences[i]);
        final_boxes.push_back(boxes[i]);
        final_centroids.push_back(centroids[i]);
    }

    // draw to image
    drawing(original_image, image, final_class_ids, final_confidences, final_boxes, final_centroids);
    if (_config->visualize)
    {
        // OpenCV display
        imshow(_config->camera_name, image);
        waitKey(3);
    }

    // generate rmf_obstacles
    auto rmf_obstacles = to_rmf_obstacles(original_image, final_class_ids, final_centroids);

    return rmf_obstacles;
}

YoloDetector::Obstacles YoloDetector::to_rmf_obstacles(const Mat &original_image, const vector<int> &final_class_ids, const vector<Point> &final_centroids)
{
    auto rmf_obstacles = Obstacles();

    // prepare obstacle_msg objects and add to rmf_obstacles
    rmf_obstacles.obstacles.reserve(final_centroids.size());
    for (size_t i = 0; i < final_centroids.size(); i++) {
        Point3d obstacle = img_coord_to_cam_coord(final_centroids[i], original_image);

        auto rmf_obstacle = Obstacle();
        rmf_obstacle.header.frame_id = _config->camera_name;
        rmf_obstacle.id = i;
        rmf_obstacle.source = _config->camera_name;
        rmf_obstacle.level_name = "L1";
        rmf_obstacle.classification = _class_list[final_class_ids[i]];
        rmf_obstacle.bbox.center.position.x = obstacle.x;
        rmf_obstacle.bbox.center.position.y = obstacle.y;
        rmf_obstacle.bbox.center.position.z = obstacle.z;
        rmf_obstacle.bbox.size.x = 1.0;
        rmf_obstacle.bbox.size.y = 1.0;
        rmf_obstacle.bbox.size.z = 2.0;
        rmf_obstacle.lifetime.sec = 10;

        cam_coord_to_world_coord(rmf_obstacle);
        rmf_obstacles.obstacles.push_back(rmf_obstacle);
    }

    return rmf_obstacles;
}

Point3d YoloDetector::img_coord_to_cam_coord(const Point &centroid, const Mat &original_image)
{
    // input obstacle centroid is in image coordinates (px, py)
    // output obstacle in camera coordinates (cx, cy, cz) are w.r.t. the camera's frame
    // camera's frame definition:
        // x-axis is +ve in the direction the camera is facing, ignoring camera pitch
        // y-axis is +ve to the left of the camera
        // x & y axis are parallel to the ground
        // z-axis is +ve upwards and perpendicular to the ground

    // _d_param & _w_param were calculated for a standing person's midpoint at the center of the image
    // assume that every vertical offset of 1 pixel corresponds to a constant change in depth of the obstacle
    float depth_per_pixel = _d_param*2/original_image.rows;
    float py = centroid.y;

    // calculate cx:
    // factor tries to account for the fact that pixels at top of the image correspond to a larger change
    // in depth compared to pixels at the bottom of the image
    float factor =  py/(original_image.rows/2);
    if (factor > 1)
    {
        depth_per_pixel = depth_per_pixel*factor;
    }
    else
    {
        depth_per_pixel = depth_per_pixel*(((1 - factor)*100));
    }

    // use image y-coordinate (py) and _d_param to estimate camera x-coordinate (cx)
    float cx = _d_param + (depth_per_pixel * ((original_image.rows/2) - py));
    if (cx < 0) cx = 0.1;

    // calculate cy:
    // assume that every horizontal offset of 1 pixel corresponds to a constant change in horizontal displacement of the obstacle
    float width_per_pixel = _w_param*2/original_image.cols;
    float px = centroid.x;

    // the higher the obstacle's depth, cx, the larger the width_per_pixel
    width_per_pixel = width_per_pixel * cx / _d_param;

    // use image x-coordinate (px) and _w_param to estimate camera y-coordinate (cy)
    float cy = width_per_pixel * ((original_image.cols/2) - px);

    // calculate cz:
    float cz = 0.0;
    return Point3d(cx, cy, cz);
}

void YoloDetector::cam_coord_to_world_coord(Obstacle &obstacle)
{
    // Transform obstacle coordinates from camera frame to world frame:
    // Input obstacle coordinates are in the camera's frame, where the camera's frame has roll = pitch = 0.
    // The true camera's frame in _camera_pose may have roll & pitch but we need to ignore that
    // before doing transformation. Also ignore z translation as obstacles are on the ground plane.

    // get roll, pitch, yaw
    tf2::Quaternion temp(
        _camera_pose.rotation.x,
        _camera_pose.rotation.y,
        _camera_pose.rotation.z,
        _camera_pose.rotation.w);
    tf2::Matrix3x3 m(temp);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // keep only yaw
    tf2::Quaternion q_yaw_only;
    q_yaw_only.setRPY( 0, 0, yaw );

    // create camera frame without z, roll, pitch
    geometry_msgs::msg::TransformStamped camera_tf_stamped;
    camera_tf_stamped.transform.translation = _camera_pose.translation;
    camera_tf_stamped.transform.translation.z = 0;
    camera_tf_stamped.transform.rotation.x = q_yaw_only.x();
    camera_tf_stamped.transform.rotation.y = q_yaw_only.y();
    camera_tf_stamped.transform.rotation.z = q_yaw_only.z();
    camera_tf_stamped.transform.rotation.w = q_yaw_only.w();

    // do transformation
    const auto before_pose = geometry_msgs::build<geometry_msgs::msg::PoseStamped>()
        .header(obstacle.header)
        .pose(obstacle.bbox.center);
    geometry_msgs::msg::PoseStamped after_pose;
    tf2::doTransform(before_pose, after_pose, camera_tf_stamped);
    obstacle.bbox.center = after_pose.pose;
}

void YoloDetector::drawing(const Mat &original_image, Mat &image, const vector<int> &final_class_ids, const vector<float> &final_confidences, const vector<Rect> &final_boxes, const vector<Point> &final_centroids)
{
    for (size_t i = 0; i < final_class_ids.size(); i++)
    {
        Rect box = final_boxes[i];
        int left = box.x;
        int top = box.y;
        int width = box.width;
        int height = box.height;
        // Draw bounding box
        rectangle(image, Point(left, top), Point(left + width, top + height), BLUE, RECT_THICKNESS);
        // Draw centroid
        circle(image, final_centroids[i], 2, CV_RGB(255,0,0), -1);

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
    image = image(Range(0,original_image.rows),Range(0,original_image.cols));

    // Draw image center
    circle(image, Point(original_image.cols/2, original_image.rows/2), 2, CV_RGB(255,255,0), -1);

    // publish ROS Topic
    cv_bridge::CvImage img_bridge;
    std_msgs::msg::Header header;
    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
    sensor_msgs::msg::Image image_msg;
    img_bridge.toImageMsg(image_msg);
    _image_detections_pub->publish(image_msg);
}

void YoloDetector::draw_label(Mat &input_image, string label, int left, int top)
{
    // Display the label at the top of the bounding box
    int baseLine;
    Size label_size = getTextSize(label, FONT_FACE, FONT_SCALE, FONT_THICKNESS, &baseLine);
    top = max(0, top - label_size.height);
    left = max(0, left - RECT_THICKNESS/2);
    // Top left corner
    Point tlc = Point(left, top);
    // Bottom right corner
    Point brc = Point(left + label_size.width, top + label_size.height);
    // Draw blue text background
    rectangle(input_image, tlc, brc, BLUE, FILLED);
    // Put the label on the blue text background
    putText(input_image, label, Point(left, top + label_size.height), FONT_FACE, FONT_SCALE, YELLOW, FONT_THICKNESS);
}