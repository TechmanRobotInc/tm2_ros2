# __TECHMAN ROBOT__

## __1. Overview__

Techman Robot is a state-of-the-art production tool that is highly compatible and flexible to collaboration between human and machine. The Robot Operating System (ROS) provides abundant libraries and tools which can be utilized to reduce the cost of trivial development software tools and build robot applications without struggling. Our TM ROS driver provides nodes for communication with Techman Robot controllers, data including robot states, images from the eye-in-hand camera, and URDF models for various robot arms via __TMflow__ <sup>1</sup>. In addition to TM ROS Driver, TM Robot also provides related resources, such as sample programs, GUI tools for debugging, and resource description files required for simulation on MoveIt or Gazebo.
<div> </div>
This manual applies to TMflow Version 2.14 or above and adapts to HW5.0 mainly.

## __2. TM ROS Driver Feature__

The TM ROS driver connects to _TMflow Ethernet Slave_ to control _TMflow_ project. The robot state is transmitted through this connection.  A working driver also connects to a __Listen Node__ <sup>2</sup> (running at a _TMflow project_) at the same time. To control the robot locomotion, IO, etc., the TM ROS driver sends the robot script (__TMscript__ <sup>3</sup>) through this connection. More information about __TM Robot Expression__ <sup>4</sup> and _Ethernet Slave_, see the defined protocol <sup>4</sup> _Expression Editor Manual_.<br/>

> [!NOTE]  
> To use the driver, make sure your ROS PC is installed correctly.

&#10146; <sup>1</sup>  __TMflow__ is a graphical human-machine interface (HMI).<br/>
&#10146; <sup>2</sup>  __Listen Node__: A socket server can be established and connected by an external device to communicate according to the defined protocol in the _Listen Node_. All the functions available in _Expression Editor_ can also be executed in Listen Node.<br/>
&#10146; <sup>3</sup>  __TMscript__ is the programming language of Techman Robot applicable to Flow projects and Script projects.<br/>
&#10146; <sup>4</sup>  __Techman Robot Expression__ (defined protocol) is the programming language of Techman Robot applicable to Flow programming projects and Script programming projects.<br/>
>
Some relevant references [Docs](https://www.tm-robot.com/en/docs/introducing-tmflow-2-14/):
> :bookmark_tabs: The _Expression Editor_ version changes may have slightly different settings. (Several old versions for reference: ([Expression Editor_1.88_Rev1.00](https://www.tm-robot.com/zh-hant/wpfd_file/expression-editor_1-88_rev1-00_en/)) ([1.84_Rev1.00](https://www.tm-robot.com/zh-hant/wpfd_file/expression-editor-and-listen-node_1-84_rev1-00_en-2/))<br/>
> :bookmark_tabs: The user can download the new "_Expression Editor Manual_" from [TM Download Center](https://www.tm-robot.com/en/download-center/) or [Contact us](https://www.tm-robot.com/en/contact-us/).<br/>

TM ROS Driver consists of three main parts: Topic Publisher, Service Server, and Action Server:

> __Topic Publisher__
>
> - publish feedback state on _/feedback_states_
The FeedbackState includes robot position, error code, and IO status, etc.
(see _tm_msgs/msg/FeedbackState.msg_)
> - publish joint states on _/joint_states_
> - publish tool pose on _/tool_pose_
>
> __Service Server__
>
> - _/tm_driver/send_script_ (see _tm_msgs/srv/SendScript.srv_) :
send robot script (_TM Robot Expression_) to _Listen Node_
> - _/tm_driver/set_event_ (see _tm_msgs/srv/SetEvent.srv_) :
Send "Stop", "Pause", or "Resume" commands to _Listen Node_
> - _/tm_driver/set_io_ (see _tm_msgs/srv/SetIO.srv_) :
send digital or analog output value to _Listen Node_
> - _/tm_driver/set_positions (see _tm_msgs/srv/SetPositions.srv_) :
Send motion command to _Listen Node_, the motion type includes PTP_J, PTP_T, LINE_T, the position value is a joint angle(__J__) or Cartesian coordinate(__T__), see [The TM "Expression Editor" manual]
>
> __Action Server__
>
> - An action interface on _/follow_joint_trajectory_ for seamless integration with MoveIt
>

The _Topic Publisher_ connects to _TMflow_ through the Ethernet slave, collects robot-related data, and publishes it as a topic (such as robot states, joint states, end tool pose, etc.), and the customer's ROS node can subscribe to these topics to obtain data. The role of the _Service Server_ interface is to control the movement of the robot and provide various movement instructions  _tm_msgs_. When the _TMflow project_ runs to the _Listen Node_, the customer's ROS node can issue instructions to the _Listen node_ through the _Service Server_ to drive the robot. The role of the _Action Server_ interface is to translate the trajectory calculated by MoveIt into the movement command of the robot and drive the robot to complete the trajectory.
<div> </div>

## __3. TM ROS Driver Usage and Installation__

The TM ROS driver is designed to interface the TM Robot's operating software (_TMflow_) with the Robot Operating System (ROS) so that program developers and researchers can build and reuse their own programs to control the TM robot externally.<br/>
[![TM ROS Driver](https://markdown-videos.vercel.app/youtube/LuKE2wVNn5Y)](https://youtu.be/LuKE2wVNn5Y)[![TM AI Cobot](https://markdown-videos.vercel.app/youtube/EG3v1KbxLoM.gif)](https://youtu.be/EG3v1KbxLoM.gif)<br/>

If the user wants to know how to use the TM ROS driver, please visit the TM ROS APP website or directly click the TM ROS APP version listed in the table below.

<table>
<head>
</head>
    <tr>
        <th colspan="5">TMflow 2 + TM AI Cobot S-Series </th>
    </tr>
    <tr>
        <th>ROS Distro</th>
        <th>GitHub repo: TM 2 App Release</th>
        <th>TM ROS Driver</th>
        <th>TMvision Support</th>
        <th>GitHub Branch</th>
    </tr>
    <tr>
        <td><a href="http://wiki.ros.org/noetic">ROS Noetic Ninjemys</a></td>
        <td><a href="https://github.com/TechmanRobotInc/tm2_ros1">TM2 ROS1 Noetic</a></td>
        <th>&#9711;</th>
        <th>&#9711;</th>
        <th>noetic</th>
    </tr>
    <tr>
        <td><a href="https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/">ROS 2 Foxy Fitzroy</a></td>
        <td><a href="https://github.com/TechmanRobotInc/tm2_ros2/tree/foxy">TM2 ROS2 Foxy</a></td>
        <th>&#9711;</th>
        <th>&#9711;</th>
        <th>foxy</th>
    </tr>
    <tr>
        <td><a href="https://docs.ros.org/en/humble/index.html">ROS 2 Humble Hawksbill</a></td>
        <td><a href="https://github.com/TechmanRobotInc/tm2_ros2/tree/humble">TM2 ROS2 Humble</a></td>
        <th>&#9711;</th>
        <th>&#9711;</th>
        <th>humble</th>
    </tr>
</table>

&#10148; Example: If your ROS PC is installed with ROS 2 Humble Hawksbill, see [TM2 ROS2 Humble](https://github.com/TechmanRobotInc/tm2_ros2/tree/humble).<br/>

- [Usage Guideline](./doc/tm_humble.md)

> :bookmark_tabs: The drivers require a relevant system with ROS installed. This repository provides the external _TM2 ROS2 Humble Driver_ and related  software packages.<br/>
> :bookmark_tabs: Using _TMflow_, especially the Listen Nodes and Vision Nodes (external detection), please refer to _Software Manual TMflow ([SW2.14](https://www.tm-robot.com/zh-hant/wpfd_file/software_tmflow_sw2-14_rev1-01_en/))_  and _Software Manual TMvision ([SW2.14](https://www.tm-robot.com/zh-hant/wpfd_file/software_tm-3dvision_sw2-14_rev1-00_en/))_ for more details.<br/>
> :bookmark_tabs: Using _TMscript_ (expressions, the Listen Node commands, etc.), please refer to the Manual: [Programming Language TMscript](https://www.tm-robot.com/zh-hant/wpfd_file/programming-language-tmscript_2-20_rev1-0_en/) for more details.<br/>

<div> </div>

## __4. TM Program Script Demonstration__
This chapter describes the demo package and the code used as a C++ programming example, showing how to program robot scripts (TM Robot Expressions) through the TM ROS driver connection.
- [Usage Guideline](./doc/tm_humble_demo.md)

:technologist:: See the demo code [`demo_send_script`](./demo/src/demo_send_script.cpp) as an example.
<div> </div>

## __5. TM External GUI debugging and Demonstration__
This chapter describes a simplified GUI for displaying tm_driver connection status, sct, sta, svr messages, and robot status. The user can optionally install the _tm_inspect_ package to aid in viewing messages between the driver and the robot through the GUI display.
- [Usage Guideline](./doc/tm_humble_gui.md)
<div> </div>

## __6. Generate your TM Robot-Specific Kinematics Parameters Files__
Real kinematic values vary from one TM robot to another as each robot is calibrated at the factory.<br/>
This chapter describes how the user can use a script program to extract specific kinematic values from your TM robot. The Python script function automatically generates a new URDF robot file that has XML macros in it (i.e., a new Xacro robot file) using a specific set of commands.
- [Usage Guideline](./doc/tm_humble_description.md)

> [!TIP]   
> 1. If the user just wants to use the TM Robot nominal model to control the robot, the user can skip the rest of this chapter.<br/>
> 2. The tm_description package contains description files and meshes, available for TM5S, TM6S, TM7S, TM12S, TM14S, TM20S, TM25S, TM30S, and (without the integrated camera) TM5SX, TM6SX, TM7SX, TM12SX, TM14SX, TM20SX, TM25SX, and TM30SX models.<br/>
<div> </div>

## __7. Related ROS Projects and Tutorials Usage__
&#10148; For example, you can try to run __MoveIt__ on the TM robot<br/>
The user can use MoveIt to control the TM robot in the motion planning to plan paths or run the TM Robot simulation in your scene description for operations such as _collision checking_ or _obstacle avoidance_.
See [MoveIt2 tutorial](https://moveit.ros.org/install-moveit2/binary/) to install the MoveIt2 packages.<br/>
- [Usage Guideline](./doc/tm_humble_extension.md)

> [!TIP]
> 1. Some software packages with ROS2 Humble MoveIt2 configurations for TM Cobots are available for TM5S, TM6S, TM7S, TM12S, TM14S, TM20S, TM25S, TM30S, and (without the integrated camera) TM5SX, TM6SX, TM7SX, TM12SX, TM14SX, TM20SX, TM25SX, and TM30SX models.<br/>
> 2. Some software packages with ROS2 Humble Gazebo Fortress configurations for TM Cobots are available for TM5S, TM6S, TM7S, TM12S, TM14S, TM20S, TM25S, TM30S, and (without the integrated camera) TM5SX, TM6SX, TM7SX, TM12SX, TM14SX, TM20SX, TM25SX, and TM30SX models.<br/>
<div> </div>

## __8. Contact us / Technical support__   [![Email](https://img.shields.io/badge/-Email-c14438?style=flat&logo=Gmail&logoColor=white)](mailto:tmsales@tm-robot.com)
More Support & Service, please contact us. [@TECHMAN ROBOT](https://www.tm-robot.com/en/contact-us/)``[https://www.tm-robot.com/en/contact-us/] ``<br/>
<div> </div>
