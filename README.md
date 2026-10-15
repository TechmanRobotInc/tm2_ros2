# TECHMAN ROBOT

## 1. Overview

Techman Robot is a state-of-the-art production tool that is highly compatible and flexible for human-machine collaboration. The Robot Operating System (ROS) provides extensive libraries and tools that reduce software development overhead and streamline robot application development.

The **TM ROS Driver** enables communication nodes with Techman Robot controllers, providing:
- Real-time robot states
- Images from the integrated eye-in-hand camera
- URDF kinematic models for various robot arms via **TMflow**

In addition to the core driver, related resources such as sample programs, GUI debugging tools, and simulation description files required for **MoveIt** or **Gazebo** are also provided.

> [!NOTE]
> This manual applies to **TMflow Version 2.14 or above** and adapts to HW5.0 mainly.

## 2. TM ROS Driver Features

The TM ROS driver connects to _TMflow Ethernet Slave_ to control _TMflow_ project. The robot state is transmitted through this connection.  A working driver also connects to a __Listen Node__ <sup>2</sup> (running at a _TMflow project_) at the same time. To control the robot locomotion, IO, etc., the TM ROS driver sends the robot script (__TMscript__ <sup>3</sup>) through this connection. More information about __TM Robot Expression__ <sup>4</sup> and _Ethernet Slave_, see the defined protocol <sup>4</sup> _Expression Editor Manual_.<br/>

> [!NOTE]  
> :technologist: Before running the driver, ensure your ROS workstation is installed and configured correctly.

### Terminology & Definitions
&#10146; <sup>1</sup>  __TMflow__ is a graphical human-machine interface (HMI).<br/>
&#10146; <sup>2</sup>  __Listen Node__: A socket server can be established and connected by an external device to communicate according to the defined protocol in the _Listen Node_. All the functions available in _Expression Editor_ can also be executed in Listen Node.<br/>
&#10146; <sup>3</sup>  __TMscript__ is the programming language of Techman Robot applicable to Flow projects and Script projects.<br/>
&#10146; <sup>4</sup>  __Techman Robot Expression__ (defined protocol) is the programming language of Techman Robot applicable to Flow programming projects and Script programming projects.<br/>
>
### Reference Documentation
For precise implementation specifications, access the official assets hosted on the [TM Technical Document Center](https://www.tm-robot.com/en/support/technical-document/):
> :bookmark_tabs: The _Expression Editor_ version changes may have slightly different settings.<br/>
> :bookmark_tabs: The user can download the new "_Expression Editor Manual_" from [TM Download Center](https://www.tm-robot.com/en/support/download-center/) or [Contact us](https://www.tm-robot.com/en/support/contact-us/).<br/>

### Driver Architecture Overview
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

The _Topic Publisher_ connects to _TMflow_ through the Ethernet slave, collects robot-related data, and publishes it as a topic (such as robot states, joint states, end-effector pose, etc.), and the customer's ROS node can subscribe to these topics to obtain data. The role of the _Service Server_ interface is to control the movement of the robot and provide various movement instructions  _tm_msgs_. When the _TMflow project_ runs to the _Listen Node_, the customer's ROS node can issue instructions to the _Listen Node _ through the _Service Server_ to drive the robot. The role of the _Action Server_ interface is to translate the trajectory calculated by MoveIt into the movement command of the robot and drive the robot to complete the trajectory.
<div> </div>

## 3. TM ROS Driver Usage and Installation

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

&#10148; Example: :technologist: If your workstation is installed with ROS 2 Jazzy Jalisco, see [TM2 ROS2 Jazzy](https://github.com/TechmanRobotInc/tm2_ros2/tree/jazzy).<br/>

- **Documentation:** [Usage Guideline](./doc/tm_jazzy.md)

> :bookmark_tabs: The drivers require a relevant system with ROS installed. This repository provides the external _TM2 ROS2 Jazzy Driver_ and related software packages. It is recommended to use __Ubuntu 24.04__ with __ROS 2 Jazzy Jalisco__.<br/>
> :bookmark_tabs: Using _TMflow_, especially the Listen Nodes and Vision Nodes (external detection), please refer to _Software Manual TMflow ([SW2.14](https://www.tm-robot.com/zh-hant/support/download-center/))_  and _Software Manual TMvision ([SW2.14](https://www.tm-robot.com/zh-hant/support/download-center/))_ for more details.<br/>
> :bookmark_tabs: Using _TMscript_ (expressions, the Listen Node commands, etc.), please refer to the Manual: [Programming Language TMscript](https://www.tm-robot.com/zh-hant/support/download-center/) for more details.<br/>

> [!TIP]
> - Remember to configure the _Ethernet Slave Data setting_ of _TMflow_:<br/>
>>:rocket: See [TM ROS Jazzy Driver vs TMflow Software Usage: Import Data Table Setting](https://github.com/TechmanRobotInc/tm2_ros2/tree/jazzy/configs/TM_Export/README.md).<br/>
> - Usage to get the Image by the TM _EIH_ Camera API:
>> :rocket: See [TM ROS and TM _Eye-in-Hand_ (_EIH_) Camera API](https://github.com/TechmanRobotInc/tm_eih_cam_client?tab=readme-ov-file), allowing developers to directly access TM EIH Camera resources via the API.<br/>
<div> </div>

## 4. TM Program Script Demonstration

This chapter describes the demonstration package and provides a C++ programming example illustrating how to program robot scripts (TM Robot Expressions) through the TM ROS driver connection.

- **Documentation:** [Usage Guideline](./doc/tm_jazzy_demo.md)
- **Demo Reference:** See the demo code [`demo_send_script`](./demo/src/demo_send_script.cpp) as a programming template.
<div> </div>

## 5. TM External GUI Debugging and Demonstration

This chapter details a simplified Graphical User Interface (GUI) used to monitor `tm_driver` connection status and display `sct`, `sta`, and `svr` communication packets alongside real-time robot feedback. Users can optionally deploy the `tm_inspect` software package to view inter-node messaging directly via the interface.

- **Documentation:** [Usage Guideline](./doc/tm_jazzy_gui.md)
<div> </div>

## 6. TM_Cobots_ROS2_Description

This package provides the official ROS 2 Robot Description Files (URDF/Xacro) and 3D meshes for TM Cobots manipulators, which are essential for robot simulation, visualization (**RViz2**), physics simulation (**Gazebo**), and advanced motion planning (**MoveIt 2**).

- **Main Repository:** [TM_Cobots_ROS2_Description](https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Description/)
- **Description Format:** Xacro (XML Macros)
- **Documentation:** [Usage Guideline](https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Description/blob/main/tm_description/TM_Cobots_ROS2_Description.md)

> [!TIP]
> The default `tm_description` package included in this `tm2_ros2` repository only contains files for the following standard nominal profiles:<br/>
> &emsp;**TM12S Cobots Series:** &ensp;TM12S profile only<br/>
> &emsp;**TM5-900 Cobots Series:** &ensp;TM5-900 profile only<br/>
> 
> If you are working with other cobot models, please download the full model support from the main official [repository](https://github.com/TechmanRobotInc/TM_Cobots_ROS2_Description/).
<div> </div>

## 7. Generate Your TM Robot-Specific Kinematics Parameters Files

Real kinematic values vary from one TM robot to another, as each robot is calibrated at the factory.<br/>
This chapter describes how to use a script to extract specific kinematic values from your TM robot. The Python script function automatically generates a new URDF robot file that has XML macros in it (i.e., a new Xacro robot file) using a specific set of commands.

- **Description Format:** Xacro (XML Macros)
- **Documentation:** [Usage Guideline](./doc/tm_jazzy_description.md)

> [!TIP]
> If the user just wants to use the TM Robot nominal model to control the robot, the user can skip the rest of this chapter.<br/>

<div> </div>

## 8. Contact Us / Technical Support   [![Email](https://img.shields.io/badge/-Email-c14438?style=flat&logo=Gmail&logoColor=white)](mailto:tmsales@tm-robot.com)

Access to some software, manuals, and technical documents requires logging into the official [TM Download Center](https://www.tm-robot.com/en/support/download-center/).<br/>
For further support and service, please contact us: [TM Contact Us](https://www.tm-robot.com/en/support/contact-us/) | 📞 [+886-3-3288350](tel:+88633288350)<br/>
<div> </div>

