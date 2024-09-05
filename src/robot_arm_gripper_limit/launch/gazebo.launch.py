import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths
    gazebo_ros_path = get_package_share_directory('gazebo_ros')
    urdf_path = os.path.join(
        get_package_share_directory('robot_arm_gripper_limit'),
        'urdf',
        'robot_arm_gripper_limit.urdf'
    )
    
    # Include Gazebo empty_world.launch.py
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_path, 'launch', 'gazebo.launch.py')
        )
    )
    
    # Define the nodes
    tf_footprint_base_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint', '40']
    )
    
    spawn_model_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_model',
        arguments=['-file', urdf_path, '-entity', 'robot_arm_gripper_limit'],
        output='screen'
    )
    
    fake_joint_calibration_node = Node(
        package='std_msgs',
        executable='ros2 topic pub',
        name='fake_joint_calibration',
        arguments=['/calibrated', 'std_msgs/Bool', 'data: true']
    )
    
    return LaunchDescription([
        gazebo_launch,
        tf_footprint_base_node,
        spawn_model_node,
        fake_joint_calibration_node
    ])
