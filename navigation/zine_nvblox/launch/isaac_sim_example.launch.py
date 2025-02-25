# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    bringup_dir = get_package_share_directory('zine_nvblox')

    # Launch Arguments
    run_rviz_arg = DeclareLaunchArgument(
        'run_rviz', default_value='True',
        description='Whether to start RVIZ')
    run_nav2_arg = DeclareLaunchArgument(
        'run_nav2', default_value='True',
        description='Whether to run nav2')
    # NOTE(remos): When running Vslam make sure to set the global frame 
    #              to e.g. 'odom_vslam' to not clash with the Isaac Sim odometry frame
    run_vslam_arg = DeclareLaunchArgument(
        'run_vslam', default_value='True',
        description='Whether to run vslam')
    global_frame = LaunchConfiguration('global_frame',
                                       default='odom')

    # Create a shared container to hold composable nodes 
    # for speed ups through intra process communication.
    shared_container_name = "shared_nvblox_container"
    shared_container = Node(
        name=shared_container_name,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen')
    
    map_server_launch = Node(package='map_server', executable='map_node', output='screen')
    
    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'nav2', 'nav2_isaac_sim.launch.py')),
        launch_arguments={'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_nav2')))

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'perception', 'vslam.launch.py')]),
        launch_arguments={'output_odom_frame_name': global_frame,
                          'setup_for_isaac_sim': 'True',
                          # Flatten VIO to 2D (assuming the robot only moves horizontally).
                          # This is needed to prevent vertical odometry drift.
                          'run_odometry_flattening': 'True',
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items(),
        condition=IfCondition(LaunchConfiguration('run_vslam')))

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(bringup_dir, 'launch', 'nvblox', 'nvblox.launch.py')]),
        launch_arguments={'global_frame': global_frame,
                          'setup_for_isaac_sim': 'True', 
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items())

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'rviz', 'rviz.launch.py')]),
        launch_arguments={
            'config_name': 'isaac_sim_example.rviz',
            'global_frame': global_frame}.items(),
        condition=IfCondition(LaunchConfiguration('run_rviz')))

    return LaunchDescription([
        run_rviz_arg,
        run_nav2_arg,
        run_vslam_arg,
        shared_container,
        nav2_launch,
        vslam_launch,
        nvblox_launch,
        rviz_launch,
        map_server_launch
    ])
