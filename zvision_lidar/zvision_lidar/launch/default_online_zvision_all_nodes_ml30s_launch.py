# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Launch the online zvision driver and pointcloud nodes with default configuration."""

import os
import yaml

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    device_ip = "192.168.10.108"
    port = 2368
    driver_share_dir = ament_index_python.packages.get_package_share_directory('zvision_lidar_driver')
    convert_share_dir = ament_index_python.packages.get_package_share_directory('zvision_lidar_pointcloud')
    # launch zvision driver node
    driver_params_file = os.path.join(driver_share_dir, 'config', 'ML30S-zvision_lidar_driver_node-params.yaml')
    with open(driver_params_file, 'r') as f:
        driver_params = yaml.safe_load(f)['zvision_lidar_node']['ros__parameters']
    driver_params["device_ip"] = device_ip
    driver_params["udp_port"] = port
    zvision_lidar_driver_node = launch_ros.actions.Node(package='zvision_lidar_driver',
                                                   executable='zvision_lidar_node',
                                                   output='both',
                                                   parameters=[driver_params])

    # launch zvision convert node
    convert_params_file = os.path.join(convert_share_dir, 'config', 'ML30S-zvision_lidar_convert_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['zvision_lidar_cloud_node']['ros__parameters']
    convert_params["device_ip"] = device_ip
    zvision_convert_node = launch_ros.actions.Node(package='zvision_lidar_pointcloud',
                                                    executable='zvision_convert_node',
                                                    output='both',
                                                    parameters=[convert_params])
    # launch rviz2 node
    rviz_config = ament_index_python.packages.get_package_share_directory('zvision_lidar_pointcloud')+'/rviz_cfg/zvision_lidar.rviz'
    zvision_rviz_node = launch_ros.actions.Node(package='rviz2',
                                                namespace='rviz2',
                                                executable='rviz2',
                                                arguments=['-d',rviz_config])


    return launch.LaunchDescription([zvision_lidar_driver_node,
                                     zvision_convert_node,
                                     zvision_rviz_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=zvision_lidar_driver_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     ])
