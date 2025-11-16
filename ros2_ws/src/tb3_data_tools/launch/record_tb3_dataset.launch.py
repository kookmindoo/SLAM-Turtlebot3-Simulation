"""Launch multi-topic recorder with TurtleBot3 defaults."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bag_path_arg = DeclareLaunchArgument('bag_path', default_value='/tmp/tb3_dataset')
    storage_arg = DeclareLaunchArgument('storage_id', default_value='sqlite3')
    config_arg = DeclareLaunchArgument('config_file', default_value='config/recorder_topics.yaml')

    recorder_node = Node(
        package='tb3_data_tools',
        executable='multi_topic_recorder',
        name='tb3_multi_topic_recorder',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'bag_path': LaunchConfiguration('bag_path'),
                'storage_id': LaunchConfiguration('storage_id'),
            },
        ],
    )

    return LaunchDescription([
        bag_path_arg,
        storage_arg,
        config_arg,
        recorder_node,
    ])
