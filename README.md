# rmf_obstacle_detectors
![](https://github.com/open-rmf/rmf_obstacle_detectors/workflows/build/badge.svg)
![](https://github.com/open-rmf/rmf_obstacle_detectors/workflows/style/badge.svg)

Packages that infer the presence of obstacles from sensor inputs.
  - [rmf_obstacle_detector_laserscan](#rmf_obstacle_detector_laserscan)
  - [rmf_human_detector_oakd](#rmf_human_detector_oakd)
  - [rmf_human_detector](#rmf_human_detector)

## rmf_obstacle_detector_laserscan
![](../media/rmf_obstacle_detector_laserscan.gif)


A ROS 2 node that subscribes to `LaserScan` messages and publishes obstacles to `/rmf_obstacles`.

The node is implemented as an [`rclcpp::lifecycle_node`](https://github.com/ros2/demos/tree/rolling/lifecycle) node where upon activation, it calibrates the surroundings based on the range values in the initial few `LaserScan` messages.
This essentially becomes the "obstacle-free" configuration.
Subsequently, any changes to the surroundings are detected as an obstacle.

To run
```
ros2 run rmf_obstacle_detector_laserscan laserscan_detector
```
To configure and Activate
```
#to configure
ros2 lifecycle set /laserscan_obstacle_detector configure
```
```
#to activate
ros2 lifecycle set /laserscan_obstacle_detector activate
```


>Note: The node can also be loaded into a ROS 2 component container as a plugin (`LaserscanDetector`)

The node accepts the following parameters
| Parameter Name | Description | Default Value |
| --- | --- | --- |
| `scan_topic_name` | The topic over which `LaserScan` messages are published. It is strongly recommended to [filter the scan](http://wiki.ros.org/laser_filters) to remove out-of-range rays before passing it to this node. | `/lidar/scan` |
| `range_threshold` | For each ray, if the difference in range measurement between the latest `LaserScan` and the calibrated one is greater than this value, a new obstacle is assumed. | `1.0` meter |
| `min_obstacle_size` | The minimum perceived size of an object to be considered an obstacle. | `0.75` meter |
| `level_name` | The level(floor) name on which the scanner exists. | `L1` |
| `calibration_sample_count` | The number of initial `LaserScan` messages to cailbrate the "obstacle-free" configuration. | `10` |

## rmf_human_detector_oakd
![](../media/rmf_human_detector_oakd.gif)

A ROS 2 node that detects humans via on-chip-inference on `OAK-D` cameras and publishes the detections over `/rmf_obstacles` as `rmf_obstacle_msgs::Obstacles` message.

The node can be run either as a component within a ROS 2 container or as a standalone node.

The component plugin is `rmf_human_detector_oakd::HumanDetector` and can be loaded into a `ComponentManager` via `ros2 component load <COMPONENT_MANAGER> rmf_human_detector_oakd::HumanDetector`.

To launch as a standalone node,
```bash
ros2 launch rmf_human_detector_oakd human_detection.launch.xml blob_path:=<PATH_TO_MOBILENET-SSD_BLOB>
```

The node has several configurable parameters documented in the [launch file](rmf_human_detector_oakd/launch/human_detector.launch.xml).
The most important is `blob_path` as it holds the absolute path to the NN model for inference. See `depthai` documentation for more information on how the various pre-trained models can be obtained.
It is recommended to use the [blobconverter](https://github.com/luxonis/blobconverter/) tool to obtain the `mobilenet-ssd_openvino_2021.4_6shave.blob` blob for inference.


To visualize the detection frames, set `debug:=True`. Note: Frames will only be visualized when a human is actively detected.

For more information on setup and troubleshooting see [here](rmf_human_detector_oakd/README.md)

## rmf_human_detector
![](../media/rmf_human_detector.gif)

A ROS 2 node that subscribes to `sensor_msgs::Image` messages published by a monocular camera and runs `Yolo-V4` to detect the presence of humans. The relative pose of the humans with respect to the camera frame is estimated based on heuristics that can be configured through ROS 2 params.

The use case for this node would be to detect crowds from existing CCTV cameras or vision sensors in the facility.

To run:

```bash
ros2 launch rmf_human_detector human_detector_launch.py
```

## rmf_obstacle_ros2
The `rmf_obstacle_ros2` package contains ROS 2 nodes that react to the presence of obstacles.

At present the `lane_blocker_node` is available which subscribes to `/rmf_obstacles`, and checks whether
any of the obstacles intersect with any of the lanes across fleet navigation graphs.
If a new intersection is determined, the lanes for the corresponding fleets are closed.
Previously closed lanes are also opened once the obstacles no longer intersect with the lanes.

To run:
```bash
ros2 run rmf_obstacle_ros2 lane_blocker_node
```
