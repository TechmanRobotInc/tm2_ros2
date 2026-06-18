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
Some relevant references [Docs](https://www.tm-robot.com/en/support/technical-document/):
> :bookmark_tabs: The _Expression Editor_ version changes may have slightly different settings.<br/>
> :bookmark_tabs: The user can download the new "_Expression Editor Manual_" from [TM Download Center](https://www.tm-robot.com/en/support/download-center/) or [Contact us](https://www.tm-robot.com/en/support/contact-us/).<br/>

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
    <tr>
        <td><a href="https://docs.ros.org/en/jazzy/index.html">ROS 2 Jazzy Jalisco</a></td>
        <td><a href="https://github.com/TechmanRobotInc/tm2_ros2/tree/jazzy">TM2 ROS2 Jazzy</a></td>
        <th>&#9711;</th>
        <th>&#9711;</th>
        <th>jazzy</th>
    </tr>
</table>

&#10148; Example: If your ROS PC is installed with ROS 2 Jazzy Jalisco, see [TM2 ROS2 Jazzy](https://github.com/TechmanRobotInc/tm2_ros2/tree/jazzy).<br/>

- [Usage Guideline](./doc/tm_jazzy.md)

> :bookmark_tabs: The drivers require a relevant system with ROS installed. This repository provides the external _TM2 ROS2 Jazzy Driver_ and related software packages. It is recommended to use __Ubuntu 24.04__ with __ROS 2 Jazzy Jalisco__.<br/>
> :bookmark_tabs: Using _TMflow_, especially the Listen Nodes and Vision Nodes (external detection), please refer to _Software Manual TMflow ([SW2.14](https://www.tm-robot.com/zh-hant/support/download-center/))_  and _Software Manual TMvision ([SW2.14](https://www.tm-robot.com/zh-hant/support/download-center/))_ for more details.<br/>
> :bookmark_tabs: Using _TMscript_ (expressions, the Listen Node commands, etc.), please refer to the Manual: [Programming Language TMscript](https://www.tm-robot.com/zh-hant/support/download-center/) for more details.<br/>

> [!TIP]
> - Remember to configure the _Ethernet Slave Data setting_ of _TMflow_:<br/>
>>:rocket: See [TM ROS Jazzy Driver vs TMflow Software Usage: Import Data Table Setting](https://github.com/TechmanRobotInc/tm2_ros2/tree/jazzy/configs/README.md).<br/>
> - Usage to get the Image by the TM _EIH_ Camera API:
>> :rocket: See [TM ROS and TM _Eye-in-Hand_ (_EIH_) Camera API](https://github.com/TechmanRobotInc/tm_eih_cam_client?tab=readme-ov-file), allowing developers to directly access TM EIH Camera resources via the API.<br/>
<div> </div>

## __4. TM Program Script Demonstration__
This chapter describes the demo package and the code used as a C++ programming example, showing how to program robot scripts (TM Robot Expressions) through the TM ROS driver connection.
- [Usage Guideline](./doc/tm_jazzy_demo.md)

:technologist:: See the demo code [`demo_send_script`](./demo/src/demo_send_script.cpp) as an example.
<div> </div>

## __5. TM External GUI debugging and Demonstration__
This chapter describes a simplified GUI for displaying tm_driver connection status, sct, sta, svr messages, and robot status. The user can optionally install the _tm_inspect_ package to aid in viewing messages between the driver and the robot through the GUI display.
- [Usage Guideline](./doc/tm_jazzy_gui.md)
<div> </div>

## __6. Contact Us / Technical Support__   [![Email](https://img.shields.io/badge/-Email-c14438?style=flat&logo=Gmail&logoColor=white)](mailto:tmsales@tm-robot.com)
Access to some software, manuals, and technical documents requires logging into the official [TM Download Center](https://www.tm-robot.com/en/support/download-center/).<br/>
For further support and service, please contact us: [TM Contact Us](https://www.tm-robot.com/en/support/contact-us/) | 📞 [+886-3-3288350](tel:+88633288350)<br/>
<div> </div>
