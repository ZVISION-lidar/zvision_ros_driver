import os
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

# def generate_launch_description():
#     packet_dir = get_package_share_directory('zvlidar_sdk')
#     yaml_path = packet_dir + '/config/config.yaml'
#     rviz_config = packet_dir + '/config/rviz/rviz2.rviz'

#     if os.getenv('GDB_ENABLED', False):  # 检查环境变量是否设置为启用 GDB
#         gdb_args = ['gdb', '--args']  # 添加 GDB 参数
#     else:
#         gdb_args = []  # 如果未启用 GDB，则为空列表

#     return LaunchDescription([
#         Node(
#             package='zvlidar_sdk',
#             executable='zvlidar_node',
#             name='zvlidar_node_driver',
#             parameters=[{"yaml_path":yaml_path}],
#             output='screen',
#             prefix=gdb_args  # 将 GDB 参数添加到启动节点的前缀
#         ),
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', rviz_config]
#         )
#     ])
