import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import FindExecutable, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define the model argument
    model_arg = LaunchConfiguration('model')
    
    # Define paths
    urdf_path = os.path.join(
        get_package_share_directory('robot_arm_gripper_limit'),
        'urdf',
        'robot_arm_gripper_limit.urdf'
    )
    rviz_config_path = os.path.join(
        get_package_share_directory('robot_arm_gripper_limit'),
        'urdf.rviz'
    )
    
    # Define the nodes
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', urdf_path])}]
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'model',
            default_value=model_arg,
            description='Absolute path to robot urdf file'
        ),
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
