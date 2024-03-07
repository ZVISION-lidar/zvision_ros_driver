
import launch
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    """Generate launch description with multiple components."""
    packet_dir = get_package_share_directory('zvlidar_sdk')
    yaml_path = packet_dir + '/config/config.yaml'
    rviz_config = packet_dir + '/config/rviz/rviz2.rviz'
    container = ComposableNodeContainer(
            name='zvlidar_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='zvlidar_sdk',
                    plugin='zvlidar_sdk::ZVLidarNodelet',
                    name='ZVLidarNodelet',
                    #remappings=[('/zvlidar', '/zvlidar_driver')],
                    parameters=[{"yaml_path":yaml_path}]),
                
            ],
            output='both',
    )

    rviz = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rviz_config]
        )

    return launch.LaunchDescription([container, rviz])




# def generate_launch_description():
#     packet_dir = get_package_share_directory('zvlidar_sdk')
#     yaml_path = packet_dir + '/config/config.yaml'
#     rviz_config = packet_dir + '/config/rviz/rviz2.rviz'
#     container = Node(
#         name='zvlidar_container',
#         package='rclcpp_components',
#         executable='component_container',
#         output='screen',
#     ),

#     load_composable_nodes = LoadComposableNodes(
#         target_container='zvlidar_container',
#         composable_node_descriptions=[
#             ComposableNode(
#                 package='zvlidar_sdk',
#                 plugin='zvlidar_sdk::ZVLidarNodelet',
#                 name='zvlidar_node_driver',
#                 remappings=[('/image', '/burgerimage')],
#                 parameters=[{"yaml_path":yaml_path}]
#             ),
#         ],
#     )

#     return LaunchDescription([container, load_composable_nodes])