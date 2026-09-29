from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    packet_dir = get_package_share_directory('zvlidar_sdk')
    yaml_path = packet_dir + '/config/config.yaml'
    rviz_config = packet_dir + '/config/rviz/rviz2.rviz'
    return LaunchDescription([
        Node(
            package='zvlidar_sdk',
            executable='zvlidar_node',
            name='zvlidar_node_driver',
            parameters=[{"yaml_path":yaml_path}],
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rviz_config]
        )
    ])
