# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2022-2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch the H.264 Decoder Node."""
    decoder_front_left = ComposableNode(
        name='decoder_front_left',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        remappings=[
                    ('image_compressed', 'front_stereo_camera/right/image_raw_compressed'),
                    ('image_uncompressed', 'front_stereo_camera/right/image_raw')
        ])
    
    decoder_front_right = ComposableNode(
        name='decoder_front_right',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        remappings=[
                    ('image_compressed', 'left_stereo_camera/right/image_raw_compressed'),
                    ('image_uncompressed', 'left_stereo_camera/right/image_raw')
        ])
    
    decoder_left_left = ComposableNode(
        name='decoder_left_left',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        remappings=[
                    ('image_compressed', 'lleft_stereo_camera/left/image_raw_compressed'),
                    ('image_uncompressed', 'left_stereo_camera/left/image_raw')
        ])
    
    decoder_left_right = ComposableNode(
        name='decoder_left_right',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        remappings=[
                    ('image_compressed', 'left_stereo_camera/right/image_compressed'),
                    ('image_uncompressed', 'left_stereo_camera/right/image_uncompressed')
        ])
    

    decoder_right_left = ComposableNode(
        name='decoder_right_left',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        remappings=[
                    ('image_compressed', 'right_stereo_camera/left/image_raw_compressed'),
                    ('image_uncompressed', 'right_stereo_camera/left/image_raw')
        ])
    
    decoder_right_right = ComposableNode(
        name='decoder_right_right',
        package='isaac_ros_h264_decoder',
        plugin='nvidia::isaac_ros::h264_decoder::DecoderNode',
        remappings=[
                    ('image_compressed', 'right_stereo_camera/right/image_raw_compressed'),
                    ('image_uncompressed', 'right_stereo_camera/right/image_raw')
        ])

    container = ComposableNodeContainer(
        name='decoder_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[decoder_front_left, decoder_front_right,
                                      decoder_left_left, decoder_left_right,
                                      decoder_right_left, decoder_right_right],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info']
    )

    return (launch.LaunchDescription([container]))
