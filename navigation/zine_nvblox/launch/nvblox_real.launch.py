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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    bringup_dir = get_package_share_directory('zine_nvblox')

    # Launch Arguments

    flatten_odometry_to_2d_arg = DeclareLaunchArgument(
        'flatten_odometry_to_2d', default_value='False',
        description='Whether to flatten the odometry to 2D (camera only moving on XY-plane).')
    
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

    # Nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            bringup_dir, 'launch', 'nav2', 'nav2_isaac_sim.launch.py')),
        launch_arguments={'global_frame': global_frame}.items(),
        )
    
    # Realsense
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'sensors', 'realsense.launch.py')]),
        launch_arguments={
            'attach_to_shared_component_container': 'True',
            'component_container_name': shared_container_name}.items(),
       )

    # Vslam
    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'perception', 'vslam.launch.py')]),
        launch_arguments={'output_odom_frame_name': global_frame, 
                          'setup_for_realsense': 'True',
                          'run_odometry_flattening': LaunchConfiguration('flatten_odometry_to_2d'),
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items())

    # Nvblox
    nvblox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            bringup_dir, 'launch', 'nvblox', 'nvblox.launch.py')]),
        launch_arguments={'global_frame': global_frame,
                          'setup_for_realsense': 'True',
                          'attach_to_shared_component_container': 'True',
                          'component_container_name': shared_container_name}.items())

    #
 

    return LaunchDescription([
  
        flatten_odometry_to_2d_arg,
        shared_container,
        realsense_launch,
        vslam_launch,
        nvblox_launch,
        nav2_launch,
        ])
