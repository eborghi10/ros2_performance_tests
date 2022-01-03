import launch
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


this_package = get_package_share_directory('throughput_test')


def generate_launch_description():

    # Load the parameters specific to the realsense ComposableNode
    config_file_path = os.path.join(this_package, 'config', 'd435.yaml')
    with open(config_file_path, 'r') as file:
        config_params = yaml.safe_load(file)['realsense']['ros__parameters']

    realsense_camera_node = ComposableNode(
        package='realsense2_camera',
        plugin='realsense2_camera::RealSenseNodeFactory',
        name='realsense',
        parameters=[config_params],
        extra_arguments=[{'use_intra_process_comms': True}])

    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                realsense_camera_node,
            ],
            output='screen')

    # Change the maximum cache size
    # https://github.com/ros2/rosbag2/blob/master/ros2bag/ros2bag/verb/record.py#L90
    megabytes = 1024*1024
    image_throughput = HEIGHT_PX*WIDTH_PX*int(FPS)
    cache_size = 500*megabytes
    # TODO: Use this Composable Node
    #       https://github.com/berndpfrommer/rosbag2_composable_recorder/blob/master/launch/recorder.launch.py#L39
    rosbag_record = launch.actions.ExecuteProcess(
            # Specify output location
            # '-o', PathJoinSubstitution([this_package, 'data', 'bag_example'])
            cmd=['ros2', 'bag', 'record', '-a', '--max-cache-size', str(cache_size)],
            output='screen')

    # , rosbag_record
    return launch.LaunchDescription([container])
