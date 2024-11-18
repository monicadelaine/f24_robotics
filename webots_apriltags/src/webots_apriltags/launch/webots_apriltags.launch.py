#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
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

"""Launch Webots TurtleBot3 Burger driver."""

#!/usr/bin/env python


import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')


    # Include the turtlebot3_bringup launch file (robot.launch.py)

    turtlebot3_bringup_launch = IncludeLaunchDescription(

        PythonLaunchDescriptionSource(

            os.path.join(get_package_share_directory('turtlebot3_bringup'), 'launch', 'robot.launch.py')

        ),

        launch_arguments={'use_sim_time': use_sim_time}.items()

    )


    # Run the v4l2_camera node for camera input

    v4l2_camera_node = Node(

        package='v4l2_camera',

        executable='v4l2_camera_node',

        name='v4l2_camera_node',

        output='screen',

        parameters=[{'use_sim_time': use_sim_time}],

        remappings=[('/image_raw', '/camera/image_raw')]  # Remap to your camera topic

    )


    # Run the apriltag_ros node with remapped topics and parameter file

    apriltag_ros_node = Node(

        package='apriltag_ros',

        executable='apriltag_node',

        name='apriltag_node',

        output='screen',

        parameters=[{'use_sim_time': use_sim_time}],

        arguments=[

            '--ros-args',

            '-r', 'image_rect:=/image_raw',

            '-r', 'camera_info:=/camera_info',

            '--params-file',

            os.path.join(get_package_share_directory('apriltag_ros'), 'cfg', 'tags_36h11.yaml')

        ]

    )


    # Run the Random2.py script using python3

    random2_script = ExecuteProcess(

        cmd=['python3', '~/f24_robotics/webots_apriltags/scripts/Random2.py'],

        name='random2_script',

        output='screen'

    )


    return LaunchDescription([

        DeclareLaunchArgument(

            'use_sim_time',

            default_value='false',

            description='Use simulation time (set to true if using simulation)'

        ),


        # Launch the turtlebot3_bringup robot launch file

        turtlebot3_bringup_launch,


        # Launch the v4l2_camera_node for camera input

        v4l2_camera_node,


        # Launch the apriltag_ros_node for Apriltag detection

        apriltag_ros_node,


        # Launch the Random2.py script

        random2_script,

    ])
