import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.substitutions import EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
        launch_arguments={'ign_args': '-r install/rmf_human_detector/share/rmf_human_detector/worlds/test_world.sdf'}.items(),
    )

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
                {"camera_name": "camera1"}
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
        ign_gazebo,
        bridge,
    ])
