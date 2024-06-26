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

from launch import LaunchDescription
from launch_ros.actions import LoadComposableNodes, Node, SetParameter, SetRemap
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # output_odom_frame_name_arg = DeclareLaunchArgument(
    #     'output_odom_frame_name', default_value='odom',
    #     description='The name of the VSLAM output frame')
    

    # Option to attach the nodes to a shared component container for speed ups through intra process communication.
    # Make sure to set the 'component_container_name' to the name of the component container you want to attach to.
    attach_to_shared_component_container_arg = LaunchConfiguration('attach_to_shared_component_container', default=False)
    component_container_name_arg = LaunchConfiguration('component_container_name', default='vslam_container')

    # If we do not attach to a shared component container we have to create our own container.
    vslam_container = Node(
        name=component_container_name_arg,
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        condition=UnlessCondition(attach_to_shared_component_container_arg)
    )

    load_composable_nodes = LoadComposableNodes(
        target_container=component_container_name_arg,
        composable_node_descriptions=[
            # Vslam node
            ComposableNode(
                name='visual_slam_node',
                package='isaac_ros_visual_slam',
                plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode'),

           ])

    # Conditionals for setup
    setup_for_realsense = IfCondition(
        LaunchConfiguration('setup_for_realsense', default='False'))
    setup_for_isaac_sim = IfCondition(
        LaunchConfiguration('setup_for_isaac_sim', default='False'))

    group_action = GroupAction([
        ##########################################
        ######### VISUAL SLAM NODE SETUP #########
        ##########################################

        # Set general parameters
        SetParameter(name='enable_debug_mode', value=False),
        SetParameter(name='debug_dump_path', value='/tmp/cuvslam'),
        SetParameter(name='enable_slam_visualization', value=True),
        SetParameter(name='enable_observations_view', value=True),
        SetParameter(name='enable_landmarks_view', value=True),
        SetParameter(name='map_frame', value='map'),
        SetParameter(name='enable_localization_n_mapping', value=False),
        # SetParameter(name='publish_odom_to_base_tf', value=True),
        # SetParameter(name='publish_map_to_odom_tf', value=False),
        # SetParameter(name='invert_odom_to_base_tf', value=True),
   
        # Parameters for Realsense
        SetParameter(name='enable_rectified_pose', value=True,
                     condition=setup_for_realsense),
        SetParameter(name='denoise_input_images', value=False,
                     condition=setup_for_realsense),
        SetParameter(name='rectified_images', value=True,
                     condition=setup_for_realsense),
        SetParameter(name='base_frame', value='camera_link',
                     condition=setup_for_realsense),
        SetParameter(name='enable_imu_fusion', value=True,
                     condition=setup_for_realsense),
        SetParameter(name='gyro_noise_density', value=0.000244,
                     condition=setup_for_realsense),
        SetParameter(name='gyro_random_walk', value=0.000019393,
                     condition=setup_for_realsense),
        SetParameter(name='accel_noise_density', value=0.001862,
                     condition=setup_for_realsense),
        SetParameter(name='accel_random_walk', value=0.003,
                     condition=setup_for_realsense),
        SetParameter(name='calibration_frequency', value=200.0,
                     condition=setup_for_realsense),
        SetParameter(name='imu_frame', value='camera_gyro_optical_frame',
                     condition=setup_for_realsense),
        SetParameter(name='rig_frame', value= 'base_link',
                     condition=setup_for_realsense),
        SetParameter(name='odom', value= 'odom',
                     condition=setup_for_realsense),
        SetParameter(name='camera_optical_frames', value=[
                    'camera_infra1_optical_frame',
                    'camera_infra2_optical_frame',
                    ],condition=setup_for_realsense),


        # Remappings for Realsense
        SetRemap(src=['/stereo_camera/left/camera_info'],
                 dst=['/camera/infra1/camera_info'],
                 condition=setup_for_realsense),
        SetRemap(src=['/stereo_camera/right/camera_info'],
                 dst=['/camera/infra2/camera_info'],
                 condition=setup_for_realsense),
        SetRemap(src=['/stereo_camera/left/image'],
                 dst=['/camera/realsense_splitter_node/output/infra_1'],
                 condition=setup_for_realsense),
        SetRemap(src=['/stereo_camera/right/image'],
                 dst=['/camera/realsense_splitter_node/output/infra_2'],
                 condition=setup_for_realsense),
        SetRemap(src=['/visual_slam/imu'],
                 dst=['/camera/imu'],
                 condition=setup_for_realsense),
        #################################################
        ######### ODOMETRY FLATTENER NODE SETUP #########
        #################################################

        ########################################
        ######### ADD COMPOSABLE NODES #########
        ########################################

        load_composable_nodes
    ])

    return LaunchDescription([
                              vslam_container,
                              group_action])
