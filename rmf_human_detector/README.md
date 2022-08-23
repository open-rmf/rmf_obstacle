# rmf_human_detector

Use cameras in gazebo to perform human detection using yolov5s model

## Requirements
* [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)- Tested on `humble`
* [Open-RMF](https://github.com/open-rmf/rmf)

## Setup

To run the human detector, an object detection model is required and a labels file is required.
A default labels file is provided. Follow the steps below to get the yolov5s object detection model.

### To get YOLOv5s ONNX format:

YOLOv5: Object detection models developed by [Ultralytics](https://github.com/ultralytics/yolov5).

```bash
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt
python3 export.py --weights yolov5s.pt --include onnx
```
This will generate yolov5s.pt and yolov5s.onnx.
Configure human_detector_launch.py to point to yolov5s.onnx model.

### To get coco.names file:

coco.names is a file of all the possible classes/labels for an object.
The labels can be found in [Ultralytics](https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml).

## Run

Run this launch file from your `Open-RMF` workspace:
```bash
ros2 launch rmf_human_detector human_detector_launch.py
```