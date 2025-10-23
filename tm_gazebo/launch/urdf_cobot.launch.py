############################################################################################### 
#  urdf_cobot.launch.py
#  Description: Launch TM Cobot in RVIZ
############################################################################################### 

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# Usage: Spawn a Techman robot model in the Rviz2.
# Example: Take TM12S Techman robot model as the default, so set 'tm12s.urdf' in robot_description_config
# Terminal: [key-in] shell cmd $ ros2 launch tm_gazebo urdf_cobot.launch.py


def generate_launch_description():

    # Robot Description Configuration
    robot_model_file = 'tm12s.urdf'

    # Specify the paths/directories to TM/ROS package definition
    project_gazebo_pkg = 'tm_gazebo'
    gazebo_dir = get_package_share_directory(project_gazebo_pkg)
    models_from_urdf = gazebo_dir + '/xacro/' + robot_model_file
    rviz_path_file = '/rviz/view_model.rviz'
    rviz_config_file = gazebo_dir + rviz_path_file

    # -------------------------------------------------------------------------
    # Load the robot_description
    robot_description_config = models_from_urdf
    with open(robot_description_config, 'r') as infp:
        robot_desc = infp.read()
    # -------------------------------------------------------------------------

    # Publish TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[
            {'use_sim_time': True},
            {'robot_description': robot_desc},
        ]
    )

    # Publish joint states
    joint_state_slider = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        arguments=[robot_description_config],
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
            {'robot_description': robot_desc},
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
