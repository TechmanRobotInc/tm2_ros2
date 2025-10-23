############################################################################################### 
#  gz_empty.launch.py
# Terminal: [key-in] shell cmd $ ign gazebo empty.sdf
# or # [key-in] workspace $ ros2 launch tm_gazebo gz_empty.launch.py
############################################################################################### 
#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, AppendEnvironmentVariable
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    # Configure gazebo environment variables

    # Check: Basic sdf file contain a world on the gazebo GUI
    gz_world_file = 'empty.sdf'

    # Specify the paths/directories to TM/ROS package definition
    project_gazebo_pkg = 'tm_gazebo'
    gazebo_dir = get_package_share_directory(project_gazebo_pkg)
    gazebo_models_path = gazebo_dir + '/models'
    gazebo_worlds_path = gazebo_dir + '/worlds'
    worlds_path_sdf = gazebo_dir + '/worlds/' + gz_world_file

    # Setting "GZ_SIM_RESOURCE_PATH" , "IGN_GAZEBO_RESOURCE_PATH": To support pre-garden. Deprecated.
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        print("ign path exist...")
    else:
        print("ign path not exist...")

    os.environ['GZ_SIM_RESOURCE_PATH'] = gazebo_models_path + ":" + gazebo_worlds_path


    ign_execute = ExecuteProcess(cmd=['ign', 'gazebo', worlds_path_sdf], output='screen')

    # List to be launched
    return LaunchDescription([
        ign_execute
    ])

    del os.environ['GZ_SIM_RESOURCE_PATH']

