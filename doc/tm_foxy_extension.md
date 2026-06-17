# __Related Projects and Tutorials Usage__
## &sect; ROS2 driver usage
> 
> After the user has set up the ROS2 environment (example : [Debian packages for ROS 2 Foxy](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)) and built the TM ROS Apps based on the specific workspace, please enter your workspace `<workspace>` by launching the terminal, and remember to make the workspace visible to ROS. 
>
>
> ```bash
> source /opt/ros/foxy/setup.bash
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
> Now, the user can use a new terminal to run each ROS node or command, but don't forget to source the correct setup shell files as starting a new terminal.
> Note: When you finish executing your developed scripts or motion commands through the TM ROS driver connection, press __CTRL + C__ in all terminal windows to shut everything down.

## &sect; Usage with MoveIt2-foxy (2.2.3)
>
> See [MoveIt2 tutorial](https://moveit.ros.org/install-moveit2/source/) to install the MoveIt2 packages.<br/>
>
> If you plan to use MoveIt, it is recommended to install and use Cyclone DDS.
> ```bash
> sudo apt install ros-$ROS_DISTRO-rmw-cyclonedds-cpp
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> ```
>
> The `<MoveIt_WS>` means the MoveIt2 workspace, for example `ws_moveit2` .<br/>
> The `<tm2_ws>` means TM ROS Apps workspace, for example `tm2_ws` .<br/>
>
> To build MoveIt2 
> ```bash
> cd ws_moveit2
> colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
> ```
>
> Assuming that the user is ready to build MoveIt2, and the user wants to apply the MoveIt by TM Robot, please do'nt forget to source the MoveIt environment, or you can add  ``source <MoveIt_WS>/install/setup.bash`` to your `.bashrc`.<br/>
> Then, to build the TM ROS Apps based on the <tm2_ws> workspace, please enter the specific workspace `tm2_ws` by launching the terminal, and remember to make the workspace visible to ROS.<br/>
>
>
> ```bash
> source /opt/ros/foxy/setup.bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> cd ~/tm2_ws
> source ~/ws_moveit2/install/setup.bash
> colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
> source ./install/setup.bash
> ```
>
> :bulb: If you have built the TM ROS Apps before or downloaded new packages to expand new applications, it is recommended that you delete the build, install, and log folders by the command `rm -rf build install log`, and __recompile the workspace__. For example,<br/>
>
>
> ```bash
> source /opt/ros/foxy/setup.bash
> export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
> cd ~/tm2_ws
> rm -rf build install log
> source ~/ws_moveit2/install/setup.bash
> colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
> source ./install/setup.bash
> ```
>
> The demo launches the RViz GUI and demonstrates planning and execution of a simple collision-free motion plan with TM Robot.<br/> 
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.<br/>
>
> * To bring up the MoveIt2 demo environment in simulation mode with the virtual TM Robot, by typing<br/>
>
>
> ```bash
> ros2 launch tm_moveit_cpp_demo <tm_robot_type>_run_moveit_cpp.launch.py
> ```
>
>> The prefix `<tm_robot_type>` means the TM Robot type, available for tm5s, tm7s, tm12s, tm14s, and tm25s models.
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing
> ```bash
> ros2 launch tm_moveit_cpp_demo tm12s_run_moveit_cpp.launch.py
> ```
>
> **Note**: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>
> The package tm_move_group provides a launch configuration for running a MoveGroup setup. The user can start planning and executing motions using the RViz MotionPlanning display.<br/>
> :bulb: Do you prepare the __TM Robot__ ready ? Make sure that TM Robot's operating software (__TMflow__) network settings are ready and the __Listen node__ is running.<br/>
>
> * To bring up the MoveIt2 - MoveGroup demo in simulation mode with the virtual TM Robot, by typing<br/>
>
>
> ```bash
> ros2 launch tm_move_group <tm_robot_type>_run_move_group.launch.py
> ```
>
>> The prefix `<tm_robot_type>` means the TM Robot type, available for tm5s, tm7s, tm12s, tm14s, and tm25s models.
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing
> ```bash
> ros2 launch tm_move_group tm12s_run_move_group.launch.py
> ```
>
> * The user can also manipulate the real TM Robot to run, by typing<br/>
>
> ```bash
> ros2 launch tm_move_group <tm_robot_type>_run_move_group.launch.py robot_ip:=<robot_ip_address>
> ```
> :warning:[CAUTION] This demo will let the real TM Robot move, please be careful. If the user are a beginner or unfamiliar with the arm movement path, it is recommended that the user place your hand on the big red emergency _Stick Stop Button_ at any time, and press the button appropriately in the event of any accident that may occur.<br/>
>
> Taking the TM12S robot as an example, use the commands introduced above, by typing<br/>
>
> ```bash
> ros2 launch tm_move_group tm12s_run_move_group.launch.py robot_ip:=<robot_ip_address>
> ```
>
>> The parameter `<robot_ip_address>` means the IP address of the TM Robot.<br/>
>
> Note: When you have finished, press CTRL + C in all terminal windows to shut everything down.<br/>
> :bookmark_tabs: Note1: There are several built-in TM Robot nominal robot model settings, available for TM5S, TM7S, TM12S, TM14S, and TM25S models.<br/>
> :bookmark_tabs: Note2: TM Robot set the default to read the Xacro file, such as _TM5S_ model, to read the file _tm5s.urdf.xacro_ into robot_description or such as _TM12S_ model, to read the file _tm12s.urdf.xacro_ into robot_description. If the user wants to use the specific model parameters instead of the nominal model to control the robot, please go back to the section __6. Generate your TM Robot-Specific Kinematics Parameters Files__ to modify the Xacro file.<br/>
> :bookmark_tabs: Note3: __Running two TM ROS drivers at the same IP address is not allowed.__ Since the tm driver node has been written into the moveit launch file, there is no need to execute _ros2 run tm_driver tm_driver robot_ip:=<robot_ip_address>_.<br/>
<div> </div>

