############################################################################################### 
#  view_cobot.launch.py
#  Description: Launch TM Cobot in RVIZ
############################################################################################### 

import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Usage: Spawn a Techman robot model in the Rviz2.
# Example: Take TM12S Techman robot model as the default, so set 'tm12s.urdf.xacro' in robot_description_config
# Terminal: [key-in] shell cmd $ ros2 launch tm_gazebo view_cobot.launch.py


def generate_launch_description():

    # Robot Description and Controller Configuration
    robot_model_file = 'tm12s.urdf.xacro'
    robot_model_path = 'xacro'
    robot_controller_file = 'tm_controllers.yaml'

    # Specify the paths/directories to TM/ROS package definition
    project_description_pkg = 'tm_description'
    description_dir = get_package_share_directory(project_description_pkg)
    project_gazebo_pkg = 'tm_gazebo'
    gazebo_dir = get_package_share_directory(project_gazebo_pkg)
    project_ros_gz_sim_pkg = 'ros_gz_sim'
    ros_gz_sim_dir = get_package_share_directory(project_ros_gz_sim_pkg)
    gazebo_models_path = gazebo_dir + '/models'
    gazebo_worlds_path = gazebo_dir + '/worlds'
    world_path = os.path.join(gazebo_dir, 'worlds', 'test_world.world')
    models_path = os.path.join(gazebo_dir, 'worlds')
    world_file = "empty.sdf"

    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(project_gazebo_pkg), "config", robot_controller_file]
    )

    rviz_path_file = '/rviz/view_model.rviz'
    rviz_config_file = gazebo_dir + rviz_path_file


    # -------------------------------------------------------------------------	
    # Load the robot_description
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [
                    FindPackageShare(project_gazebo_pkg),
                    robot_model_path,
                    robot_model_file,
                ]
            ),
            ' ',
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    # -------------------------------------------------------------------------		

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
    return LaunchDescription(
        [
            joint_state_slider,
            robot_state_publisher_node,
            rviz_node,
        ]
    )
