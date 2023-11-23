
# Copyright 2023, Ekumen Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""Run navigation2 pkg with Andino robot in Gazebo."""


import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # [ ] Navigation Bringup
    # [ ] Slam Toolbox
    # [ ] RVIZ (aparte)
    # [ ] Andino bringup?
    # Get the launch directory
    andino_bringup_dir = get_package_share_directory('andino_bringup')

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    andino_slam_dir = get_package_share_directory('andino_slam')
    andino_navigation_dir = get_package_share_directory('andino_navigation')

    nav2_params_file = LaunchConfiguration('nav2_params_file')
    slam_params_file = LaunchConfiguration('slam_params_file')


    declare_nav2_params_file_cmd = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=os.path.join(andino_navigation_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS 2 parameters file to use for nav2 nodes',
    )

    declare_slam_params_file_cmd = DeclareLaunchArgument(
        'slam_params_file',
        default_value=os.path.join(andino_slam_dir, 'config', 'slam_toolbox_online_async.yaml'),
        description='Full path to the ROS 2 parameters file to use for slam nodes',
    )
    
    include_andino_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(andino_bringup_dir, 'launch', 'andino_robot.launch.py')
        )
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')
        ),  
            launch_arguments = {'params_file': nav2_params_file,
                                'use_sim_time': "False"}.items()
    )

    slam_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(andino_slam_dir, 'launch', 'slam_toolbox_online_async.launch.py'),
        ),
        launch_arguments={'slam_params_file': slam_params_file}.items(),
    )

    nav2_bringup_timer = TimerAction(period=20.0, actions=[nav2_bringup_cmd])
    slam_bringup_timer = TimerAction(period=20.0, actions=[slam_bringup_cmd])

    return LaunchDescription([
        declare_nav2_params_file_cmd,
        declare_slam_params_file_cmd,
        include_andino_bringup,
        nav2_bringup_timer,
        slam_bringup_timer
    ])
    