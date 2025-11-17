# __Related Projects and Tutorials Usage__
## &sect; ROS2 driver usage
> 
> After the user has set up the ROS2 environment (example : [Debian packages for ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)) and built the TM ROS Apps based on the specific workspace, please enter your workspace `<workspace>` by launching the terminal, and remember to make the workspace visible to ROS. 
>
>
> ```bash
> source /opt/ros/humble/setup.bash
> cd <workspace>
> source ./install/setup.bash
> ```
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running. 
> 
> Then, run the driver to test whether the complete communication interface is working properly with the TM Robot by typing 
>
>```bash
> ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>
>```
> Example :``ros2 run tm_driver tm_driver robot_ip:=192.168.10.2``, if the <robot_ip_address> is 192.168.10.2
>
> Now, the user can use a new terminal to run each ROS node or command, but don't forget to source the correct setup shell files when starting a new terminal.
> Note: When you finish executing your developed scripts or motion commands through the TM ROS driver connection, press __CTRL + C__ in all terminal windows to shut everything down.

## &sect; Usage with MoveIt2-humble (Binary)
>
> See [MoveIt2 tutorial](https://moveit.ros.org/install-moveit2/binary/) to install the MoveIt2 packages.<br/>
> ```bash
> sudo apt install ros-humble-moveit
> ```
> Then, use the following command to install these ROS2 Humble dependency packages
> ```bash
> sudo apt-get install ros-humble-controller-manager
> sudo apt install ros-humble-joint-trajectory-controller
> sudo apt install ros-humble-joint-state-broadcaster
> ```
>
> If you plan to use MoveIt, it is recommended to install and use Cyclone DDS.
> ```bash
> sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> ```
>
> The `<tm2_ws>` means TM ROS Apps workspace, for example `tm2_ws` .<br/>
>
>
> Then, to build the TM ROS Apps based on the <tm2_ws> workspace, please enter the specific workspace `tm2_ws` by launching the terminal, and remember to make the workspace visible to ROS.<br/>
>
>
> ```bash
> source /opt/ros/humble/setup.bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> cd ~/tm2_ws
> colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
> source ./install/setup.bash
> ```
>
> :bulb: If you have built the TM ROS Apps before or downloaded new packages to expand new applications, it is recommended that you delete the build, install, and log folders by the command `rm -rf build install log`, and __recompile the workspace__. For example,<br/>
>
>
> ```bash
> source /opt/ros/humble/setup.bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> cd ~/tm2_ws
> rm -rf build install log
> colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
> source ./install/setup.bash
> ```
>
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.<br/>
>
> * To bring up the MoveIt2 - MoveGroup demo in simulation mode with the virtual TM Robot, by typing<br/>
>
>
> ```bash
> ros2 launch <tm_robot_type>_moveit_config <tm_robot_type>_run_move_group.launch.py
> ```
>
> The prefix `<tm_robot_type>` means the TM Robot type, available for tm5s, tm7s, tm12s, tm14s, tm25s, tm30s, and (without the integrated camera) tm5sx, tm7sx, tm12sx, tm14sx, tm25sx, and tm30sx models.
>>
>> :bulb: If you have started some executable programs with ROS commands in some terminal windows, it is recommended that you close them and then execute the following commands.<br/>
>> Taking the TM12S robot as an example, use the commands introduced above, by typing<br/>
``source ./install/setup.bash``<br/>
``ros2 launch tm12s_moveit_config tm12s_run_move_group.launch.py``<br/>
>>
>
> * The user can also manipulate the real TM Robot to run, by typing<br/>
>
> ```bash
> ros2 launch <tm_robot_type>_moveit_config <tm_robot_type>_run_move_group.launch.py robot_ip:=<robot_ip_address>
> ```
> The parameter `<robot_ip_address>` means the IP address of the TM Robot.<br/>
>
> :warning:[CAUTION] This demo will let the real TM Robot move, please be careful. If the user are a beginner or unfamiliar with the arm movement path, it is recommended that the user place your hand on the big red emergency _Stick Stop Button_ at any time, and press the button appropriately in the event of any accident that may occur.<br/>
>>
>> :bulb: If you have started some executable programs with ROS commands in some terminal windows, it is recommended that you close them and then execute the following commands.<br/>
>> Taking the TM12S robot as an example (if the IP address is 192.168.10.2), use the commands introduced above, by typing<br/>
``source ./install/setup.bash``<br/>
``ros2 launch tm12s_moveit_config tm12s_run_move_group.launch.py robot_ip:=192.168.10.2``
>
> Note: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>
> :bookmark_tabs: Note1: There are several built-in TM Robot nominal robot model settings, available for TM5S, TM7S, TM12S, TM14S, TM25S, TM30S, and (without the integrated camera) TM5SX, TM7SX, TM12SX, TM14SX, TM25SX, and TM30SX models.<br/>
> :bookmark_tabs: Note2: TM Robot set the default to read the Xacro file, such as _TM5S_ model, to read the file _tm5s.urdf.xacro_ into robot_description or such as _TM12S_ model, to read the file _tm12s.urdf.xacro_ into robot_description. If the user wants to use the specific model parameters instead of the nominal model to control the robot, please go back to the section __6. Generate your TM Robot-Specific Kinematics Parameters Files__ to modify the Xacro file.<br/>
> :bookmark_tabs: Note3: __Running two TM ROS drivers at the same IP address is not allowed.__ Since the tm driver node has been written into the moveit launch file, there is no need to execute _ros2 run tm_driver tm_driver robots_ip:=<robot_ip_address>_.<br/>

## &sect; Usage with Gazebo Simulation 
>
> See [Gazebo tutorial](https://gazebosim.org/docs/fortress/install_ubuntu/) to install the Gazebo Fortress (formerly Ignition) libraries.<br/>
> Then, use the following command to install these ROS2 Gazebo dependency packages<br/>
`` sudo apt-get install ros-humble-gazebo-ros-pkgs``<br/>
`` sudo apt-get install ros-humble-ros-gz-sim``<br/>
`` sudo apt-get install ros-humble-ros-gz``<br/>
`` sudo apt-get install ros-humble-ign-ros2-control``<br/>
>
> A workaround for a single package is to define the environment variable IGN_CONFIG_PATH to point to the location of the Gazebo library installation, where the YAML file for the package is found, such as<br/>
>> export IGN_CONFIG_PATH=/user/local/share/ignition<br/>
>> export IGN_GAZEBO_RESOURCE_PATH=`<full path to your gazebo models directory>`<br/>
>>
> The tm_gazebo package contains the Xacro model files to simulate the TM Robot in Gazebo.
>
> :bulb: If you download new packages to expand new applications, it is recommended that you delete the build, install, and log folders in your workspace by the command `rm -rf build install log`, and __recompile the workspace__.<br/>
> There are several built-in launch files that can be used to start the TM Robot simulated robot using the nominal Xacro robot model settings in Gazebo.
> The common command's form to bring up the TM simulated robot in Gazebo is as follows: 
>
> ```bash
> ros2 launch tm_gazebo <tm_robot_type>_gazebo.launch.py
> ```
>
> The prefix `<tm_robot_type>` means the TM Robot type, available for the tm5s, tm7s, tm12s, tm14s, tm25s, tm30s, and (without the integrated camera) tm5sx, tm7sx, tm12sx, tm14sx, tm25sx, and tm30sx models.<br/>
> 
>> Taking the TM12S robot as an example, use the ros2 action send_goal command line tool to send some FollowJointTrajectory goals to move to several positions.<br/>
>> :bulb: If you have started some executable programs with ROS commands in some terminal windows, it is recommended that you close them and then execute the following commands.<br/>
>> 1. To open the terminal 1: Running with Gazebo.<br/>
`` source /opt/ros/humble/setup.bash``<br/>
`` cd <workspace>``<br/>
`` source ./install/setup.bash``<br/>
`` ros2 launch tm_gazebo tm12s_gazebo.launch.py``<br/>
>> 2. In a new terminal 2: Type to change the current directory into the scripts directory path and execute the send_goal.sh .<br/>
`` cd tm_gazebo/scripts``<br/>
`` ./send_goal.sh``
> (or `` ./demo_gz_goalaction.sh``)<br/>
>
> Note: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>

> __Using MoveIt 2 with Gazebo Simulator__
>
>  You can also use MoveIt 2 to control the simulated robot, which is configured to run alongside Gazebo.
> 
> ```bash
> ros2 launch <tm_robot_type>_moveit_config <tm_robot_type>_run_move_group_gz.launch.py sim:=True
> ```
>> Taking the TM12S simulated robot as an example, ("sim:=True" is set by default and can be omitted for virtual robot simulation), use the command described above:
>> :bulb: If you have started some executable programs with ROS commands in some terminal windows, it is recommended that you close them and then execute the following commands.<br/>
>> Example: To open the terminal: Running Gazebo with moveit 2<br/>
``source ./install/setup.bash``<br/>
``ros2 launch tm12s_moveit_config tm12s_run_move_group_gz.launch.py``<br/>
>
> Note: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>
>
> * The user can also manipulate the real TM Robot to run, by typing<br/>
>
> ```bash
> ros2 launch <tm_robot_type>_moveit_config <tm_robot_type>_run_move_group_gz.launch.py robot_ip:=<robot_ip_address> sim:=False
> ```
> The parameter `<robot_ip_address>` means the IP address of the TM Robot.<br/>
>
> :warning:[CAUTION] This demo will let the real TM Robot move, please be careful. If the user are a beginner or unfamiliar with the arm movement path, it is recommended that the user place your hand on the big red emergency _Stick Stop Button_ at any time, and press the button appropriately in the event of any accident that may occur.<br/>
>
>> Taking the TM12S real robot as an example, use the commands introduced above, by typing<br/>
>> :bulb: If you have started some executable programs with ROS commands in some terminal windows, it is recommended that you close them and then execute the following commands.<br/>
>> Example: To open the terminal: Running Gazebo with moveit 2, and if the IP address of the TM Robot is 192.168.10.2<br/>
``source ./install/setup.bash``<br/>
``ros2 launch tm12s_moveit_config tm12s_run_move_group_gz.launch.py robot_ip:=192.168.10.2 sim:=False``<br/>
>
> Note: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>
> :bookmark_tabs: Note1: Remember to close all these executables when you no longer use them for Gazebo simulations.<br/>
> :bookmark_tabs: Note2: If the GUI Gazebo is not properly shut down after terminating the launch, you can try to kill the corresponding process with the following command.<br/>
>
>>:bulb: **Tip**: Use __grep__ to view the specified 'ign gazebo' process information.<br/>
>> ``ps aux | grep ign``<br/>
>>:bulb: **Tip**: To kill the GUI Gazebo executables.<br/>
>> ``pkill -f -9 'ign gazebo'``<br/>
>
<div> </div>

