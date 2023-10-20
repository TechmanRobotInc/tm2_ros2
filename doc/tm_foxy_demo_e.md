# __TM Program Script Demonstration__
This chapter describes the demo package and the code used as a C++ programming example, showing how to program robot scripts (TM Robot Expressions) through the TM ROS driver connection.

## &sect; Demo package description

> * demo_send_script:<br/>
In this demo code, it shows how to send a __Listen node__ script to control the TM Robot.<br/>
The user can use a service named "send_script" to send the script.<br/>
"id" &rarr; The transaction number expressed in any <u>alphanumeric</u> <sup>1</sup> characters.<br/> 
"script" &rarr; the script that the user wants to send.<br/>
"ok" &rarr; the correctness of the script.<br/>
> <sup>1</sup> If a non-alphanumeric byte is encountered, a CPERR 04 error is reported. When used as a communication packet response, it is a transaction number and identifies which group of commands to respond.<br/>
>
> * demo_ask_item:<br/>
In this demo code, the user can use this service to send TMSVR <sup>2</sup> cmd.<br/> 
> <sup>2</sup> For more detailed information, please refer to _defined protocol_: TM Expression Editor Manual(2.12 rev1.00) (Chapter14.6 TMSVR)<br/>
>
> * demo_ask_sta:<br/>
In this demo code, the user can use this service to send TMSTA <sup>3</sup> cmd.<br/>
> <sup>3</sup> For more detailed information, please refer to _defined protocol_ (Chapter11.5 TMSTA)<br/>
> * demo_connect_tm:<br/>
In this demo code, the user can set the connection type. <br/>
>
> * demo_set_event:<br/>
In this demo code, six event types can be selected.<br/> 
func &rarr;  TAG, WAIT_TAG, STOP, PAUSE, RESUME and EXIT<br/>
arg0 &rarr;  if func is TAG or WAIT_TAG, arg0 is the tag number<br/>
arg1 &rarr;  if func is TAG or WAIT_TAG, arg1 is timeout in ms<br/>
>
> * demo_set_io:<br/>
In this demo code, the user should set the module, type, pin and state. <sup>4</sup> <br/>
module &rarr;  MODULE_CONTROLBOX or MODULE_ENDEFFECTOR<br/>
type &rarr;  TYPE_DIGITAL_IN, TYPE_DIGITAL_OUT, TYPE_INSTANT_DO, TYPE_ANALOG_IN, TYPE_ANALOG_OUT, TYPE_INSTANT_AO<br/>
pin &rarr;  pin number<br/>
state &rarr;  STATE_OFF or STATE_ON value, or other value (if type expressed in a specific control module)<br/>
> <sup>4</sup> For more detailed information, please refer to _defined protocol_ (Chapter10.5 IO)<br/>
>
> * demo_set_positions:<br/>
In this demo code, the user should pay attention to the parameter definition of the data format setting <sup>5</sup> and the parameter unit to be operated.  <br/>
motion_type &rarr;  PTP_J , PTP_T , LINE_T <br/>
positions &rarr;  motion target position: If expressed in Cartesian coordinate (unit: m), if expressed in joint angles (unit: rad)<br/>
velocity &rarr;  motion velocity: if expressed in Cartesian coordinate (unit: m/s) <sup>6</sup>, if expressed in joint velocity (unit: rad/s, and the maximum value is limited to  &pi; )  <sup>6</sup>  <br/>
acc_time &rarr; time to reach maximum speed (unit: ms)<br/> 
blend_percentage &rarr; blending value: expressed as a percentage (unit: %, and the minimum value of 0 means no blending) <br/>
fine_goal &rarr; precise position mode: If activated, the amount of error in the final position will converge more, but it will take a few more milliseconds.<br/>
> <sup>5</sup> For more detailed information, please refer to _defined protocol_ (Chapter12 PTP, Line) <br/>
> <sup>6</sup> The unit of the parameters are different, the user can find the conversion in the program of TM ROS driver.<br/>
>
> * demo_write_item: <br/>
In this demo code, the user can use this service to send TMSVR <sup>7</sup> cmd. <br/>
> <sup>7</sup> For more detailed information, please refer to _defined protocol_ (Chapter14.3 svr_write())<br/>
>
> * demo_leave_listen_node:<br/>
In this demo code, the user can use send_script service sending a script to leave the __Listen node__.<br/>
> :bulb: If the user has sent the demo_leave_listen_node script to leave the __Listen node__, and you want to run the TM Robot again, please remember that the _Listen task_ project should be resumed to run. You can press the Stop Button on the Robot Stick and then press the Play/Pause Button to resume operation. <br/>

## &sect; Prerequisites
> 1. To use this package, make sure your ROS PC is installed correctly.
> 2. The user has successfully configured the network settings of the TM Robot and the user's PC in the same subnet. In other words, users have been able to ping the remote system on the same subnet successfully.
>>:bulb: Tip:  For example, set the user computer IP address and remote TM Robot to 192.168.10.30 and 192.168.10.2 (Netmask: 255.255.255.0). Users can ping the remote IP address 192.168.10.2, by typing "ping 192.168.10.2".<br>
>
<br/>

  <img src="./figures/ping_target_host.png" width="1000" height="220">
<br/>
> 3. The user already knows how to use _TMflow 2_ programming, especially to configure TM ROS _Ethernet Slave_ "Data Table Setting" and the Listen nodes programming through a flow project. In other words, the user has created and completed the Listen task with the TM ROS setting of a TMflow software process project.
>>:bulb: Tip: The user can refer to the chapter introduced in the main text: _3. TM ROS driver usage and installation_ for quick and easy setup or refer to the _Software Manual TMflow_ for details.
>
> 4. Remember to press the Play/Pause (&#9658;) button on the Robot Stick to start running this Listen task project under auto Mode.
>>:bulb: Tip: If under Manual Mode, it requires the trigger of the _Enabling Switch_ function. Therefore, the user needs to press and hold the __Enabling Switch__ button slightly and continuously to press the Play/Pause (&#9658;) button to run the operation. The user can select to suspend _Enabling Switch_ on the UI of _TMflow_ &rArr;  Configuration &rArr; Safety, and the triggering effect of _Enabling Switch_ will be disabled. For details of the _Enabling Switch_ function, refer to the relevant contents in the _Safety Manual_ or the _Software Manual TMflow_.
>
<br/>

  <img src="./figures/suspend_enabling switch_on.png" width="1000" height="320">
<br/>

## &sect; Usage with demo code & driver on the external Linux PC
> 1. Type to create a root workspace directory by starting a terminal: For example,  ``tm2_ws`` or ``catkin_ws``, then type to change the current directory into the workspace directory path.<br/>
``mkdir ~/tm2_ws``<br/>
``cd ~/tm2_ws``<br/>
> 2. Clone the TM driver of the git repository into the current directory by typing<br/>
``git clone https://github.com/TechmanRobotInc/tm2_ros2.git``<br/>
> 3. After the download done, rename the download folder ``tm2_ros2``(or ``tm2_ros2-master``) to ``src`` by typing<br/>
``mv tm2_ros2 src``<br/>  (or right-click on the download folder, select "Rename...")<br/>
> 4. At the workspace directory to build the download packages and source 'setup.bash' in this workspace to make the workspace visible to ROS.<br/>
**Note**: Do you set ``source /opt/ros/foxy/setup.bash`` ready? Make sure to obtain the correct setup file according to your workspace hierarchy, and then type the following below to compile.<br/>
``colcon build``<br/>
``source ./install/setup.bash``<br/>
> 5. In a new terminal: Source setup.bash in the workspace path, connect to TM Robot with ROS_DOMAIN_ID and type the specific demo node function that the user wants to study for applications. For example: the user select to run demo_set_io, the user can type<br/>
``source ./install/setup.bash``<br/>
``export RMW_IMPLEMENTATION=rmw_fastrtps_cpp``<br/>
``export ROS_DOMAIN_ID=<ROS_DOMAIN_ID>``<br/>
The <ROS_DOMAIN_ID> value of this setting must be the same as the value of TM Flow.<br/>
``ros2 run demo demo_set_io``<br/>
> :warning:[CAUTION] Some demos will let the TM Robot move, please be careful.<br/>
>
For example:  (select to run demo_set_io)
>> Assume the domain ID is set to 30 and the embedded TM ROS driver is successfully enabled on _Tmflow 2_. If the user has successfully built the specific code (tm2_ros2) before, now the user only needs to open a new terminal, and set the path to the TM driver workspace ``cd ~/tm2_ws``, then type the following command to run demo_set_io.<br/>
>> ```bash
>> $ cd <workspace>
>> {workspace}$ source /opt/ros/foxy/setup.bash
>> {workspace}$ export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
>> {workspace}$ export ROS_DOMAIN_ID=30
>> {workspace}$ source ./install/setup.bash
>> {workspace}$ ros2 run demo demo_set_io
>> ```
<div> </div>

