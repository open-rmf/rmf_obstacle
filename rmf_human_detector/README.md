# rmf_human_detector

A ROS 2 node that subscribes to `sensor_msgs::Image` messages published by a monocular camera and publishes the detections over `/rmf_obstacles` as `rmf_obstacle_msgs::Obstacles` message. The node runs `YOLOv5` to detect the presence of humans as obstacles. The relative pose of the humans with respect to the camera frame is estimated based on heuristics that can be configured through ROS 2 params.

## Requirements
* [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)- Tested on `humble`
* [Open-RMF](https://github.com/open-rmf/rmf)
* YOLOv5s model in ONNX format

## Setup

To run `rmf_human_detector`, an object detection model is required and a labels file is required. An rmf demo also needs to be run to publish the `sensor_msgs::Image` messages from a monocular camera.

### Model
YOLOv5 is an object detection model developed by [Ultralytics](https://github.com/ultralytics/yolov5). The default model for this package is the YOLOv5s, the small version of the YOLOv5 model.


Get YOLOv5s in ONNX format:
```bash
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt
python3 export.py --weights yolov5s.pt --include onnx
```
This will generate `yolov5s.pt` and `yolov5s.onnx`. Copy the `yolov5s.onnx` file into the folder `rmf_human_detector/assets/`.
Configure human_detector_launch.py to point to the `yolov5s.onnx` file.

### Labels file

A labels file contains all the possible classes or labels for an object.
A default labels file `coco.names` is provided in the folder `rmf_human_detector/assets/`.

The labels can also be found on the Ultralytics [github](https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml).

### Open-RMF

Following the instructions in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

## Run

In the order shown:

Run the hotel world rmf demo from your `Open-RMF` workspace:
```bash
ros2 launch rmf_demos_ign hotel.launch.xml
```

Run the `rmf_human_detector` launch file from your `Open-RMF` workspace:
```bash
ros2 launch rmf_human_detector human_detector_launch.py
```