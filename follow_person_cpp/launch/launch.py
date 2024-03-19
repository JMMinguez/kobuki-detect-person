# Copyright 2024 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('follow_person_cpp')
    param_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    tf_cmd = Node(
        package='follow_person_cpp',
        executable='TFPublisher_main',
        parameters=[{
            'use_sim_time': True
        }],
        remappings=[
            ('input_detection', '/detection3d'),
        ],
        output='screen'
    )

    life_cycle_cmd = Node(
        package='follow_person_cpp',
        executable='FollowLifeCycle_main',
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    detector_cmd = Node(package='camera',
                        executable='hsv_filter',
                        output='screen',
                        parameters=[param_file],
                        remappings=[
                          ('input_image', '/camera/image_raw'),
                          ('camera_info', '/camera/camera_info'),
                          ('output_detection_2d', 'detection_2d'),
                        ])

    convert_2d_3d = Node(package='camera',
                        executable='detection_2d_to_3d_depth',
                        output='screen',
                        parameters=[param_file],
                        remappings=[
                          ('input_depth', '/camera/depth/image_raw'),
                          ('input_detection_2d', 'detection_2d'),
                          ('camera_info', '/camera/camera_info'),
                          ('output_detection_3d', 'detection_3d'),
                        ])

    ld = LaunchDescription()
    ld.add_action(tf_cmd)
    ld.add_action(detector_cmd)
    ld.add_action(convert_2d_3d)
    ld.add_action(life_cycle_cmd)

    return ld