# rmf_obstacle_detectors
Packages that infer the presence of obstacles from sensor inputs.

## rmf_obstacle_detector_laserscan
![](../media/rmf_obstacle_detector_laserscan.gif)


A node that subscribes to `LaserScan` messages and publishes obstacles to `/rmf_obstacles`.

The node is implemented as an`rclcpp::lifecycle_node` node where upon activation, it calibrates the surroundings based on the range values in the initial few `LaserScan` messages.
This essentially becomes the "obstacle-free" configuration.
Subsequently, any changes to the surroundings is detected as an obstacle.

To run
```
ros2 run rmf_obstacle_detector_laserscan laserscan_detector
```
>Note: The node can also be loaded into a ROS 2 component container as a plugin (`LaserscanDetector`)

The node accepts the following parameters
| Parameter Name | Description | Default Value |
| --- | --- | --- |
| `scan_topic_name` | The topic over which `LaserScan` messages are published. It is strongly recommended to [filter the scan](http://wiki.ros.org/laser_filters) to remove out-of-range rays before passing it to this node. | `/lidar/scan` |
| `range_threshold` | For each ray, if the difference in range measurement between the latest `LaserScan` and the calibrated one is grater than this value, a new obstacle is assumed. | `1.0` meter |
| `min_obstacle_size` | The minimum perceived size of an object for it to be considered an obstacle. | `0.75` meter |
| `level_name` | The level(floor) name on which the scanner exists. | `L1` |
| `calibration_sample_count` | The number of initial `LaserScan` messages to use for calibrating the "obstacle-free" configuration. | `10` |