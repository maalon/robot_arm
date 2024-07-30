import os
from time import sleep

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro

def generate_launch_description():
    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
    )

    # Define paths to package and URDF file
    package_path = os.path.join(get_package_share_directory('seven_dof_arm'))
    xacro_file = os.path.join(package_path, 'urdf', 'robot_arm_gripper_limit.urdf')

    # Parse and process the URDF file
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # Node for publishing the robot state
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Command to load joint state controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','--set-state','active','joint_state_broadcaster'],
        output='screen',
        shell=True,
    )

    # Command to load arm controller
    load_arm_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller','--set-state','active', 'arm_controller'],
        output='screen',
        shell=True,
    )

    # Node to spawn the robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description', '-entity', 'seven_dof_arm'],
        output='screen'
    )
    # Define launch description with proper event handlers and delays
    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_arm_controller],
            )
        ),
    ])
