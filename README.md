# __TECHMAN ROBOT__

## __1. Overview__

Techman Robot is a state-of-the-art production tool that is highly compatible and flexible to collaboration between human and machine. The Robot Operating System (ROS) provides abundant libraries and tools which can be utilized to reduce the cost of trivial development software tool and build robot applications without struggling. Our TM ROS driver provides nodes for communication with Techman Robot controllers, data including robot states, images from the eye-in-hand camera, and URDF models for various robot arms via __TMflow__ <sup>1</sup>. In addition to TM ROS Driver, TM Robot also provides related resources, such as sample programs, GUI tools for debugging, and resource description files required for simulation on MoveIt or Gazebo.
<div> </div>
This manual applies to TMflow Version 2.14 or above and adapts to HW5.0 mainly.

## __2. TM ROS Driver Feature__

The TM ROS driver connects to _TMflow Ethernet Slave_ to control _TMflow_ project. The robot state is transmitted through this connection.  A working driver also connects to a __Listen Node__ <sup>2</sup> (running at a _TMflow project_) at the same time. To control the robot locomotion, IO, etc., the TM ROS driver sends the robot script (__TMscript__ <sup>4</sup>) through this connection. More information about __TM Robot Expression__ <sup>3</sup> and _Ethernet Slave_, see the defined protocol <sup>3</sup> _Expression Editor Manual_.<br/>

&#10146; <sup>1</sup>  __TMflow__ is a graphical human-machine interface (HMI).
&#10146; <sup>2</sup>  __Listen Node__: A socket server can be established and be connected by an external device to communicate according to the defined protocol In the _Listen Node_. All the functions available in _Expression Editor_ can also be executed in Listen Node.
&#10146; <sup>3</sup>  __TMscript__ is the programming language of Techman Robot applicable to Flow projects and Script projects. 
&#10146; <sup>4</sup>  __Techman Robot Expression__ (defined protocol) is the programming language of Techman Robot applicable to Flow programming projects and Script programming projects.
> :bookmark_tabs: Note1: The user can download the new "_Expression Editor Manual_" or "_Embedded TM ROS Driver User Manual ([2.14_Rev1.0](https://www.tm-robot.com/en/download-center/#3100-4746-wpfd-embedded-tm-ros-driver-manual))_ " from [TM Download Center](https://www.tm-robot.com/zh-hant/download-center/) or [Contact us](https://www.tm-robot.com/zh-hant/contact-us/).<br/>
> :bookmark_tabs: Note2: The _Expression Editor_ version changes may have slightly different settings. (Several old versions for reference: ([Expression Editor_1.88_Rev1.00](https://www.tm-robot.com/zh-hant/wpfd_file/expression-editor_1-88_rev1-00_en/)) ([1.84_Rev1.00](https://www.tm-robot.com/zh-hant/wpfd_file/expression-editor-and-listen-node_1-84_rev1-00_en-2/))<br/>

TM ROS Driver consists of three main parts: Topic Publisher, Service Server, and Action Server: 

> __Topic Publisher__
>
> - publish feedback state on _/feedback_states_  
The FeedbackState includes robot position, error code, and io status, etc.
(see _tm_msgs/msg/FeedbackState.msg_)  
> - publish joint states on _/joint_states_  
> - publish tool pose on _/tool_pose_
>
> __Service Server__
>
> - _/tm_driver/send_script_ (see _tm_msgs/srv/SendScript.srv_) :  
send robot script (_TM Robot Expression_) to _Listen Node_  
> - _/tm_driver/set_event_ (see _tm_msgs/srv/SetEvent.srv_) :  
send "Stop", "Pause" or "Resume" commands to _Listen Node_  
> - _/tm_driver/set_io_ (see _tm_msgs/srv/SetIO.srv_) :  
send digital or analog output value to _Listen Node_  
> - _/tm_driver/set_positions (see _tm_msgs/srv/SetPositions.srv_) :  
send motion command to _Listen Node_, the motion type include PTP_J, PTP_T, LINE_T, the position value is a joint angle(__J__) or Cartesian coordinate(__T__), see [The TM "Expression Editor" manual]
>
> __Action Server__
>
> - An action interface on _/follow_joint_trajectory_ for seamless integration with MoveIt
>

The _Topic Publisher_ connects to _TMflow_ through the Ethernet slave, collects robot-related data, and publishes it as a topic (such as robot states, joint states, end tool pose, etc.), and the customer's ros node can subscribe to these topics to obtain data. The role of the _Service Server_ interface is to control the movement of the robot and provide various movement instructions  _tm_msgs_. When the _TMflow project_ runs to the _Listen Node_, the customer's ros node can issue instructions to the _Listen node_ through the _Service Server_ to drive the robot. The role of the _Action Server_ interface is to translate the trajectory calculated by MoveIt into the movement command of the robot and drive the robot to complete the trajectory.
<div> </div>

## __3. TM ROS Driver Usage and Installation__

The TM ROS driver is designed to interface the TM Robot's operating software (_TMflow_) with the Robot Operating System (ROS) so that program developers and researchers can build and reuse their own programs to control the TM robot externally.<br/>
[![TM ROS Driver](https://markdown-videos.vercel.app/youtube/LuKE2wVNn5Y)](https://youtu.be/LuKE2wVNn5Y)[![TM AI Cobot](https://markdown-videos.vercel.app/youtube/EG3v1KbxLoM.gif)](https://youtu.be/EG3v1KbxLoM.gif)<br/>
In __TMflow 2__ software, in addition to the old method of installing TM ROS drivers from GitHub on a Linux-based computer to start externally, we also support TM ROS driver embedded and based on ROS2 Foxy in Robot Controller via __TMflow 2__. It makes it easier for users to start ROS and run the driver to connect TM Robot on the TMflow 2 Setting UI interface and try to improve performance.<br/>

If the user wants to know how to use the TM ROS driver, please visit the TM ROS APP website or directly click the TM ROS APP version listed in the table below.

<table>
<head>
</head>
    <tr>
        <th colspan="5">TMflow 2 + TM AI Cobot S-Series </th>
    </tr>
    <tr>
        <th>ROS Distro</th>
        <th>TM ROS APP version</th>
        <th>TM ROS Driver</th>
        <th>TMvision Support</th>
        <th>Embedded TM ROS Driver</th>
    </tr>
    <tr>
        <td><a href="http://wiki.ros.org/noetic">ROS Noetic Ninjemys</a></td>
        <td><a href="https://github.com/TechmanRobotInc/tm2_ros1">TM2 ROS1 Noetic</a></td>
        <th>&#9711;</th>
        <th>&#9711;</th>
        <th>&#10005;&nbsp</th>
    </tr>
    <tr>
        <td><a href="https://index.ros.org/doc/ros2/Releases/Release-Foxy-Fitzroy/">ROS 2 Foxy Fitzroy</a></td>
        <td><a href="https://github.com/TechmanRobotInc/tm2_ros2">TM2 ROS2 Foxy</a></td>
        <th>&#9711;</th>
        <th>&#9711;</th>
        <th>DDS FastRTPS</th>
    </tr>
    <tr>
        <td><a href="https://docs.ros.org/en/humble/index.html">ROS 2 Humble Hawksbill</a></td>
        <td><a href="https://github.com/TechmanRobotInc/tm2_ros2/tree/humble">TM2 ROS2 Humble</a></td>
        <th>&#9711;</th>
        <th>&#9711;</th>
        <th>DDS FastRTPS</th>
    </tr>
</table>

&#10148; Example: If your ROS PC is installed with ROS 2 Humble Hawksbill, see [TM2 ROS2 Humble](https://github.com/TechmanRobotInc/tm2_ros2/tree/humble) and then select how you want to use the TM ROS driver.<br/>

- External TM ROS Driver [Usage Guideline](./doc/tm_humble.md)
- Embedded TM ROS Driver [Usage Guideline](./doc/tm_humble_e.md)

**Note**: To use the driver, make sure your ROS PC is installed correctly.
> :bookmark_tabs: Note1: Using the embedded TM ROS driver can only communicate with ROS 2 applications that use eProsima FastDDS as their RMW layer. If the user needs different RMW layers for other applications, such as RMW_IMPLMENTATION=rmw_cyclonedds_cpp, please choose the installation method using the external TM ROS driver.<br/>
> :bookmark_tabs: Note2: Using _TMflow_, especially the Listen Nodes and Vision Nodes (external detection). Please refer to _Software Manual TMflow ([SW2.14_Rev1.00](https://www.tm-robot.com/zh-hant/wpfd_file/software-manual-tmflow_sw2-14_rev1-00_en/))_  and _Software Manual TMvision ([SW2.14_Rev1.00](https://www.tm-robot.com/zh-hant/wpfd_file/software-manual-tmvision_sw2-14_rev1-00_en/))_ for more details.<br/>
> :bookmark_tabs: Note3: Using _TMscript_ (expressions, the Listen Node commands, etc.). Please refer to the Manual: [Programming Language TMscript](https://www.tm-robot.com/zh-hant/wpfd_file/programming-language-tmscript_rev1-00_en/) for more details.<br/>
  
<div> </div>

## __4. TM Program Script Demonstration__
This chapter describes the demo package and the code used as a C++ programming example, showing how to program robot scripts (TM Robot Expressions) through the TM ROS driver connection.
- External TM ROS Driver [Usage Guideline](./doc/tm_humble_demo.md)
- Embedded TM ROS Driver [Usage Guideline](./doc/tm_humble_demo_e.md)

**Note**: See the demo code [`demo_send_script`](./demo/src/demo_send_script.cpp) as an example.
<div> </div>

## __5. TM External GUI debugging and Demonstration__
This chapter describes a simplified GUI for displaying tm_driver connection status, sct, sta, svr messages, and robot status. The user can optionally install the _tm_inspect_ package to aid in viewing messages between the driver and the robot through the GUI display.
- External TM ROS Driver [Usage Guideline](./doc/tm_humble_gui.md)
- Embedded TM ROS Driver [Usage Guideline](./doc/tm_humble_gui_e.md)
<div> </div>

## __6. Contact us / Technical support__
More Support & Service, please contact us. [@TECHMAN ROBOT](https://www.tm-robot.com/zh-hant/contact-us/)``[https://www.tm-robot.com/zh-hant/contact-us/] ``<br/>
<div> </div>
