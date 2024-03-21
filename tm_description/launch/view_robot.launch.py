import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

# Prerequisites: joint-state-publisher-gui
# Terminal: [key-in] shell cmd $ sudo apt install ros-humble-joint-state-publisher
#                              $ sudo apt install ros-humble-joint-state-publisher-gui
# Usage: Spawn a Techman robot model in the Rviz2.
# Example: Take TM12S Techman robot model as the default, so set 'tm12s.urdf.xacro' in robot_description_config
# Terminal: [key-in] shell cmd $ ros2 launch tm_description view_robot.launch.py


def generate_launch_description():
    robot_description_config = xacro.process_file(
        os.path.join(
            get_package_share_directory('tm_description'),
            'xacro',
            'tm12s.urdf.xacro',
        )
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    # RViz
    rviz_config_file = get_package_share_directory('tm_description') + '/rviz/view_robot.rviz'
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        parameters=[robot_description]
        )

    # Static TF
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='log',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'base']
    )

    # Publish TF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    return LaunchDescription([static_tf, robot_state_publisher, joint_state_publisher_node, rviz_node])
