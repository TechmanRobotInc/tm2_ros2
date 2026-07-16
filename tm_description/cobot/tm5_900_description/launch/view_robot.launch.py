############################################################################################### 
# view_robot.launch.py
# Description: Launch TM AI Cobot in RViz2
############################################################################################### 

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

# Prerequisites: joint-state-publisher-gui
# Terminal Command Examples:
#   ros2 launch tm5_900_description view_robot.launch.py robot_model:=tm5-900
#   ros2 launch tm5_900_description view_robot.launch.py robot_model:=tm5x-900
#   ros2 launch tm20_description view_robot.launch.py robot_model:=tm20
#   ros2 launch tm20_description view_robot.launch.py robot_model:=tm20x

def launch_setup(context, *args, **kwargs):
    # Retrieve the CLI robot_model parameter
    robot_model_arg = LaunchConfiguration('robot_model').perform(context).strip()
    robot_model_file = f"{robot_model_arg}.urdf.xacro"

    # Derive the series folder e.g., tm5x-900 -> tm5_900_description)
    if 'tm5-900' in robot_model_arg:
        series_folder = 'tm5_900_description'
    elif 'tm5x-900' in robot_model_arg:
        series_folder = 'tm5_900_description'
    elif 'tm5-700' in robot_model_arg:
        series_folder = 'tm5_700_description'
    elif 'tm5x-700' in robot_model_arg:
        series_folder = 'tm5_700_description'
    elif 'tm12' in robot_model_arg:
        series_folder = 'tm12_description'
    elif 'tm14' in robot_model_arg:
        series_folder = 'tm14_description'
    elif 'tm16' in robot_model_arg:
        series_folder = 'tm16_description'
    elif 'tm20' in robot_model_arg:
        series_folder = 'tm20_description'
    else:
        series_folder = 'tm5_900_description'

    try:
        base_dir = get_package_share_directory('tm_description')
        target_package_root = os.path.join(base_dir, 'cobot', series_folder)
    except Exception:
        target_package_root = get_package_share_directory(series_folder)

    # Specify the paths/directories to TM/ROS package definition
    robot_description_file = os.path.join(target_package_root, 'xacro', robot_model_file)
    rviz_config_file = os.path.join(target_package_root, 'rviz', 'view_robot.rviz')

    print('########################################################################################################################')
    print(f"the user key-in Model: {robot_model_arg}")
    print(f"Model Series_folder  : {target_package_root}")
    print(f"the target Xacro File: {robot_description_file}")
    print('########################################################################################################################')

    # -------------------------------------------------------------------------
    # Validation and parse Xacro configuration into URDF XML content
    if not os.path.exists(robot_description_file):
        raise FileNotFoundError(f"\n[ERROR] Target Xacro file not found at: {robot_description_file}\n")

    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}
    # -------------------------------------------------------------------------

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            robot_description
        ]
    )

    # Publish joint states
    joint_state_slider = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output=['screen']
    )

    # Visualize in RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description
        ]
    )

    # List of nodes to be launched
    return [
        static_tf,
        robot_state_publisher_node,
        joint_state_slider,
        rviz_node,
    ]


def generate_launch_description():
    declare_robot_model = DeclareLaunchArgument(
        'robot_model',
        default_value='tm5-900',
        description='Specification of Techman AI Cobot model (e.g., tm5-700, tm5x-900, tm16, tm20x)'
    )

    return LaunchDescription([
        declare_robot_model,
        OpaqueFunction(function=launch_setup)
    ])
