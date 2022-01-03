# Copyright 2019 Open Source Robotics Foundation, Inc.
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

"""Launch a talker and a listener in a component container."""

import launch
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from tracetools_launch.action import Trace


this_package = get_package_share_directory('throughput_test')


def generate_launch_description():
    """Generate launch description with multiple components."""

    tracing = LaunchConfiguration('tracing', default=False)

    tracing_session = Trace(
        session_name='my-tracing-session',
        condition=IfCondition(tracing),
    )

    rosbag_play = launch.actions.ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '--loop',
             PathJoinSubstitution([this_package, 'data', 'rosbag2_2021_10_28-13_46_13'])],
        output='screen')

    # '--remap', '/stereo_camera/left/image:=' +

    # Load the parameters specific to the realsense ComposableNode
    config_file_path = os.path.join(this_package, 'config', 'd435.yaml')
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['realsense']['ros__parameters']

    realsense_camera_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense',
        parameters=[config_params],
        remappings=[
            ("color/image_raw", "image_raw"),
            ("depth/image_rect_raw", "depth_raw"),
        ],
        extra_arguments=[{'use_intra_process_comms': True}])

    talker_node = ComposableNode(
        package='throughput_test',
        plugin='throughput_test::Talker',
        name='realsense_republisher',
        extra_arguments=[{'use_intra_process_comms': True}])

    IMAGES = []
    for i in range(5):
        for type in ['image', 'depth']:
            IMAGES.append(Node(
                package="image_transport",
                executable="republish",
                name=f"realsense_{type}_{i}",
                arguments=[
                    'raw',  # Input
                    'raw',  # Output
                ],
                remappings=[
                    ("in", f"{type}_raw"),
                    ("out", f"{type}_{i}"),
                ]
            ))

    data_logging_node = ComposableNode(
        package='throughput_test',
        plugin='throughput_test::Listener',
        name='data_logging',
        # remappings=[
        #     ("image_raw", "color/image_raw"),
        #     ("color_camera_info", "color/camera_info"),
        #     ("pointcloud", "depth/color/points"),
        #     ("depth_raw", "depth/image_rect_raw"),
        #     ("depth_camera_info", "depth/camera_info"),
        #     ("extrinsics", "extrinsics/depth_to_color"),
        # ],
        extra_arguments=[{'use_intra_process_comms': True}])

    machine_learning_policy_node = ComposableNode(
        package='throughput_test',
        plugin='throughput_test::Listener',
        name='machine_learning_policy',
        extra_arguments=[{'use_intra_process_comms': True}])

    safety_controller_node = ComposableNode(
        package='throughput_test',
        plugin='throughput_test::Listener',
        name='safety_controller',
        extra_arguments=[
            {'use_intra_process_comms': True}
        ])

    compression_node = ComposableNode(
        package='throughput_test',
        plugin='throughput_test::Listener',
        name='compression',
        extra_arguments=[
            {'use_intra_process_comms': True}
        ])

    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                realsense_camera_node,
                talker_node,
                data_logging_node,
                machine_learning_policy_node,
                safety_controller_node,
                compression_node,
            ],
            output='screen')

    ld = launch.LaunchDescription()
    ld.add_action(tracing_session)
    # ld.add_action(rosbag_play)
    ld.add_action(container)
    # for image in IMAGES:
    #     ld.add_action(image)
    return ld
