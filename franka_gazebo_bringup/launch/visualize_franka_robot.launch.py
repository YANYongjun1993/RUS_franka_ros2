# Copyright (c) 2024 Franka Robotics GmbH
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

import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import LaunchContext, LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_robot_description(context: LaunchContext, robot_type):
    robot_type_str = context.perform_substitution(robot_type)

    franka_xacro_file = os.path.join(
        get_package_share_directory('franka_description'),
        'robots',
        robot_type_str,
        robot_type_str + '.urdf.xacro'
    )

    robot_description_config = xacro.process_file(
        franka_xacro_file,
        mappings={
            'robot_type': robot_type_str,
            'hand': 'false',
            'ros2_control': 'true',
            'gazebo': 'true',
            'ee_id': 'none'
        }
    )

    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
        ]
    )

    return [robot_state_publisher]


def generate_launch_description():
    # Configure ROS nodes for launch
    robot_type_name = 'robot_type'
    namespace_name = 'namespace'

    robot_type = LaunchConfiguration(robot_type_name)
    namespace = LaunchConfiguration(namespace_name)

    robot_type_launch_argument = DeclareLaunchArgument(
        robot_type_name,
        default_value='fr3',
        description='Available values: fr3, fp3 and fer')
    namespace_launch_argument = DeclareLaunchArgument(
        namespace_name,
        default_value='',
        description='Namespace for the robot. If not set, the robot will be launched in the root namespace.')

    # Get robot description
    robot_state_publisher = OpaqueFunction(
        function=get_robot_description,
        args=[robot_type])

    # Gazebo Sim
    os.environ['GZ_SIM_RESOURCE_PATH'] = os.path.dirname(
        get_package_share_directory('franka_description'))
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': 'empty.sdf -r', }.items(),
    )

    # Spawn
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        namespace=namespace,
        arguments=['-topic', '/robot_description'],
        output='screen',
    )

    # Visualize in RViz
    rviz_file = os.path.join(get_package_share_directory('franka_description'), 'rviz',
                             'visualize_franka.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        namespace=namespace,
        arguments=['--display-config', rviz_file, '-f', 'world'],
    )

    load_joint_state_broadcaster = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_state_broadcaster'],
        output='screen'
    )

    return LaunchDescription([
        robot_type_launch_argument,
        namespace_launch_argument,
        gazebo_empty_world,
        robot_state_publisher,
        rviz,
        spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn,
                on_exit=[load_joint_state_broadcaster],
            )
        ),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=namespace,
            parameters=[
                {'source_list': ['joint_states'],
                 'rate': 30}],
        ),
    ])
