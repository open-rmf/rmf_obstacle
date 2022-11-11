import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    # Bridge
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
           executable='human_detector',
           parameters=[
                {"camera_name": "camera1"},
                {"camera_level": "L1"},
                {"nn_filepath": "/home/osrc/rmf_ws/src/rmf/rmf_obstacle_detectors/rmf_human_detector/assets/yolov5s.onnx"},
                {"labels_filepath": "/home/osrc/rmf_ws/src/rmf/rmf_obstacle_detectors/rmf_human_detector/assets/coco.names"},
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
