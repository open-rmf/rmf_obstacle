# rmf camera

Use cameras in gazebo to perform human detection using yolov5s model

> ros2 launch rmf_human_detector human_detector_launch.py

### To get YOLOv5s ONNX format:

YOLOv5: Object detection models developed by [Ultralytics](https://github.com/ultralytics/yolov5)
ONNX: Open Neural Network Exchange

```bash
git clone https://github.com/ultralytics/yolov5
cd yolov5
pip install -r requirements.txt
python3 export.py --weights yolov5s.pt --include onnx
```
this will generate yolov5s.pt and yolov5s.onnx

### To get coco.names file:

coco.names is a file of all the possible classes/labels for an object.
coco.names is provided by default, and can also be found at [Ultralytics](https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml)
