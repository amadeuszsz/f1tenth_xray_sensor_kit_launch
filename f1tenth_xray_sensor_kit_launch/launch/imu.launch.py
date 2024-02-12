# Copyright 2023 Amadeusz Szymko
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('f1tenth_xray_sensor_kit_launch')
    imu_config = PathJoinSubstitution([pkg_prefix, 'config/imu/xsens.param.yaml'])
    imu_corrector_config = PathJoinSubstitution([pkg_prefix, 'config/imu/imu_corrector.param.yaml'])
    imu_corrector_input = 'vesc/imu'
    if IfCondition(LaunchConfiguration('launch_driver')).evaluate(context):
        imu_corrector_input = 'xsens/imu'

    xsens_node = Node(
        package = 'bluespace_ai_xsens_mti_driver',
        executable = 'xsens_mti_node',
        name = 'xsens',
        parameters = [imu_config],
        output = 'screen',
        remappings=[
            ('imu/data', 'xsens/imu')
        ],
        condition=IfCondition(LaunchConfiguration('launch_driver')),
        respawn=True,
        respawn_delay=2
    )

    imu_corrector_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare('imu_corrector'), 'launch', 'imu_corrector.launch.xml'
            ]),
        ),
        launch_arguments={
            'input_topic': imu_corrector_input,
            'output_topic': 'imu_corrector/imu',
            'param_file': imu_corrector_config
        }.items()
    )

    return [
        xsens_node,
        imu_corrector_launch
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('launch_driver', 'false')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
