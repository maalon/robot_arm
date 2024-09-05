from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
# from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
#from launch.substitutions import LaunchConfiguration
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Define paths to package and URDF file
    share_dir = get_package_share_directory('seven_dof_arm')
    xacro_file = os.path.join(share_dir, 'urdf', 'robot_arm_gripper_limit.urdf')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # rviz_config_file = os.path.join(share_dir, 'rviz', 'display.rviz')
    # gui_arg = DeclareLaunchArgument(
    #     name='gui',
    #     default_value='True'
    # )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )
    joint_state_publisher_node = Node(
    package='joint_state_publisher',
    executable='joint_state_publisher',
    name='joint_state_publisher'
    )
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'robot_arm7f',
            '-topic', 'robot_description'
        ],
        output='screen'
    )    
     # Nodes for controller management
    controller_manager = Node(
        package='controller_manager',
        executable="ros2_control_node",
        name='controller_manager',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare('seven_dof_arm'), 'config', 'arm_controllers.yaml'])]
    )
    # show_gui = LaunchConfiguration('gui')
    
    # joint_state_publisher_gui_node = Node(
    #     condition=IfCondition(show_gui),
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     name='joint_state_publisher_gui'
    # )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file],
    #     output='screen'
    # )

    return LaunchDescription([
        #gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        controller_manager,
        #rviz_node,
        #joint_state_publisher_gui_node,
    ])