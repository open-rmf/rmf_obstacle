# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    rmf_human_detector_dir = get_package_share_directory("rmf_human_detector")
    model_path = os.path.join(rmf_human_detector_dir, "assets", "yolov5s.onnx")
    labels_path = os.path.join(rmf_human_detector_dir, "assets", "coco.names")
    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/camera1/pose@tf2_msgs/msg/TFMessage@ignition.msgs.Pose_V',
            '/world/sim_world/model/camera1/link/visual_link/sensor/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/world/sim_world/model/camera1/link/visual_link/sensor/camera/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            ],
        output='screen',
        remappings=[
            ('/model/camera1/pose', '/camera1/pose'),
            ('/world/sim_world/model/camera1/link/visual_link/sensor/camera/image', '/camera1/image_raw'),
            ('/world/sim_world/model/camera1/link/visual_link/sensor/camera/camera_info', '/camera1/camera_info'),
        ]
    )
    return LaunchDescription([
        Node(
           package='rmf_human_detector',
           executable='human_detector_node',
           parameters=[
                {"camera_name": "camera1"},
                {"camera_level": "L1"},
                {"nn_filepath": model_path},
                {"labels_filepath": labels_path},
                {"use_gpu": True},
           ]
        ),
        Node(
            package='image_proc',
            namespace='/camera1',
            executable='image_proc',
            remappings=[
                ('image', 'image_raw'),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = [
                '--frame-id', 'map',
                '--child-frame-id', 'camera1',
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                ]
        ),
        # Node(
        #    package='rmf_obstacle_ros2',
        #    executable='lane_blocker_node',
        #    parameters=[{
        #         "lane_closure_threshold": 4,
        #         "speed_limit_threshold": 2,
        #         "obstacle_lane_threshold": 0.0,
        #         "continuous_checker": True,
        #    }]
        # ),
        bridge,
    ])
