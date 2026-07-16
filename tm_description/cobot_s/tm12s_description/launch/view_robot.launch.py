############################################################################################### 
# view_robot.launch.py
# Description: Launch TM AI Cobot S in RViz2
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
#   ros2 launch tm12s_description view_robot.launch.py robot_model:=tm12sft
#   ros2 launch tm12s_description view_robot.launch.py robot_model:=tm12s
#   ros2 launch tm30s_description view_robot.launch.py robot_model:=tm30sxft
#   ros2 launch tm30s_description view_robot.launch.py robot_model:=tm30sx

def launch_setup(context, *args, **kwargs):
    # Retrieve the CLI robot_model parameter
    robot_model_arg = LaunchConfiguration('robot_model').perform(context).strip()
    robot_model_file = f"{robot_model_arg}.urdf.xacro"

    # Derive the series folder e.g., tm12sft -> tm12s_description)
    if 'tm12s' in robot_model_arg:
        series_folder = 'tm12s_description'
    elif 'tm3s' in robot_model_arg:
        series_folder = 'tm3s_description'
    elif 'tm5s' in robot_model_arg:
        series_folder = 'tm5s_description'
    elif 'tm6s' in robot_model_arg:
        series_folder = 'tm6s_description'
    elif 'tm7s' in robot_model_arg:
        series_folder = 'tm7s_description'
    elif 'tm14s' in robot_model_arg:
        series_folder = 'tm14s_description'
    elif 'tm16s' in robot_model_arg:
        series_folder = 'tm16s_description'
    elif 'tm20s' in robot_model_arg:
        series_folder = 'tm20s_description'
    elif 'tm25s' in robot_model_arg:
        series_folder = 'tm25s_description'
    elif 'tm30sc' in robot_model_arg:
        series_folder = 'tm30sc_description'
    elif 'tm30s' in robot_model_arg:
        series_folder = 'tm30s_description'
    else:
        series_folder = 'tm12s_description'

    try:
        base_dir = get_package_share_directory('tm_description')
        target_package_root = os.path.join(base_dir, 'cobot_s', series_folder)
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
        default_value='tm12s',
        description='Specification of Techman AI Cobot model (e.g., tm12s, tm12sft, tm14sft, tm25s)'
    )

    return LaunchDescription([
        declare_robot_model,
        OpaqueFunction(function=launch_setup)
    ])
