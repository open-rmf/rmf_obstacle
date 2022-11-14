# rmf_human_detector

A ROS 2 node that subscribes to `sensor_msgs::Image` messages published by a monocular camera and detects humans using `YOLOv5` a real time object detection model. The position of detected humans is estimated and published over `/rmf_obstacles` as a `rmf_obstacle_msgs::Obstacles` message.

## Requirements
* YOLOv5s model in ONNX format
* Labels file
* [ROS 2](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)- Tested on `humble`
* [Open-RMF](https://github.com/open-rmf/rmf)
* GPU & OpenCV compiled with GPU support (optional)

To run `rmf_human_detector`, an object detection model is required and a labels file is required. An rmf demo also needs to be run to publish the `sensor_msgs::Image` messages from a monocular camera. Running `rmf_human_detector` on GPU is optional.

### Model
YOLOv5 is an object detection model developed by [Ultralytics](https://github.com/ultralytics/yolov5). The recommended model for this package is the `YOLOv5s`, the small version of the `YOLOv5` model.


Get YOLOv5s in ONNX format:
```bash
$ git clone https://github.com/ultralytics/yolov5
$ cd yolov5
$ pip install -r requirements.txt
$ python3 export.py --weights yolov5s.pt --include onnx
```
This will generate `yolov5s.pt` and `yolov5s.onnx`. Copy the `yolov5s.onnx` file into the folder `rmf_human_detector/assets/` and
configure human_detector_launch.py to point to the `yolov5s.onnx` file.

### Labels file

A labels file contains all the possible classes or labels that the `YOLOv5s` model can predict.
`rmf_human_detector/assets/coco.names` is the default labels file provided.

The labels can also be found on the Ultralytics [github](https://github.com/ultralytics/yolov5/blob/master/data/coco.yaml).

### ROS

Follow the installation instructions in [ROS 2 Documentation: Humble](https://docs.ros.org/en/humble/Installation.html).

### Open-RMF

Follow the installation instructions in [rmf_demos](https://github.com/open-rmf/rmf_demos/).

### GPU Support (Optional)

OpenCV's DNN Module uses pre-trained neural networks to perform inference using CPU only. To use NVIDIA GPU, OpenCV must be compiled from source with CUDA and cuDNN installed.

Follow the [CUDA Installation Guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html).

Follow the [cuDNN Installation Guide](https://docs.nvidia.com/deeplearning/cudnn/install-guide/index.html).

Check your NVIDIA GPU architecture version [here](https://developer.nvidia.com/cuda-gpus). This is used later in the `cmake` command to set the `CUDA_ARCH_BIN` variable.

Compile OpenCV from source:
```bash
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get install build-essential cmake unzip pkg-config
$ sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
$ sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev
$ sudo apt-get install libv4l-dev libxvidcore-dev libx264-dev
$ sudo apt-get install libgtk-3-dev
$ sudo apt-get install libatlas-base-dev gfortran
$ sudo apt-get install python3-dev
$ cd ~
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/4.5.5.zip
$ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.5.5.zip
$ unzip opencv.zip
$ unzip opencv_contrib.zip
$ mv opencv-4.5.5 opencv
$ mv opencv_contrib-4.5.5 opencv_contrib
$ cd opencv
$ mkdir build
$ cd build
$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D WITH_CUDA=ON \
	-D WITH_CUDNN=ON \
	-D OPENCV_DNN_CUDA=ON \
	-D ENABLE_FAST_MATH=1 \
	-D CUDA_FAST_MATH=1 \
	-D CUDA_ARCH_BIN=7.5 \
	-D WITH_CUBLAS=1 \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D HAVE_opencv_python3=ON \
	-D PYTHON_EXECUTABLE=/usr/bin/python3 \
	-D BUILD_EXAMPLES=ON ..
$ make -j8
$ sudo make install
$ sudo ldconfig
```

## Run

First run the hotel world rmf demo from your `Open-RMF` workspace:
```bash
ros2 launch rmf_demos_gz hotel.launch.xml
```

Then run the `rmf_human_detector` launch file from your `Open-RMF` workspace:
```bash
ros2 launch rmf_human_detector human_detector_launch.py
```

### Configurations for rmf_human_detector - ROS 2 param

| ROS 2 param           | Default value | Meaning |
| --------------------- | ------------- | ------- |
| camera_name           | "camera1"     | Name of camera model |
| camera_parent_name    | "sim_world"   | Name of parent model |
| visualize             | True          | Flag to create an OpenCV window of the image with detections |
| camera_level          | "L1"          | The floor the camera is on |
| obstacle_lifetime_sec | 1             | Duration the obstacle is valid |
| nn_filepath           | ""            | Path to `YOLOv5s` model |
| labels_filepath       | ""            | Path to labels file  |
| use_gpu               | False         | Flag to try using GPU |
| confidence_threshold  | 0.25          | Minimum confidence score for the model's prediction to be considered |
| score_threshold       | 0.45          | Minimum label score for the model's prediction to be considered |
| nms_threshold         | 0.45          | Non Maximal Suppression maximum IoU (intersection over union) for the model's prediction to be considered |

## Implementation Details

### Camera

The camera `model.sdf` has been added to `rmf_demos_assets` within the rmf_demos repo. It includes a sensor plugin that publishes camera images and camera info as ignition topics, and a pose publisher plugin that publishes camera pose as an ignition topic.

> Ideally, a user will be able to configure all the attributes of the camera through the `rmf_sandbox` GUI and then `rmf_sandbox` generates the camera's `model.sdf` and adds it in the world file. When this is ready, the camera model in `rmf_demos_assets` becomes obselete.

> Currently, traffic editor does not support model pitch. Thus the camera cannot be spawned in `rmf_demos` with a pitch. However, manually setting the camera pitch from gazebo is possible, and the human position estimation still works.

A ROS-ignition `parameter_bridge` node is used to bridge the camera image, camera info and camera pose from ignition topics to ROS topics.

An `image_proc` node is used to convert the raw camera image to a rectified image.

### Human position estimation

1. Load `YOLOv5s` model
2. Use OpenCV's DNN module to perform inference on image
3. Post process model output to get human bounding boxes
4. Project a ray to the middle of the bottom edge of bounding boxes
5. Get ground plane based on level elevation
6. Perform ray and ground plane intersection to estimate human position
7. Publish human position as `rmf_obstacles`
8. Publish image with bounding boxes around humans

Assumptions:
* The camera and camera info follow a pinhole camera model.
* The ray plane intersection method assumes the ground plane is flat.
* The ray plane intersection method assumes that the intersection point where the human touches the ground, is within the bounding box and not obstructed.

### Launch file contents

* ROS-ignition parameter bridge node
* Image rectification node
* Static pose publisher node
* `rmf_human_detector` node
