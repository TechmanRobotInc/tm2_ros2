# __Generate your TM Robot-Specific Kinematics Parameters Files__
Real kinematic values vary from TM robot to another one as each robot is calibrated at the factory.<br/>
The user can use the tm_mod_urdf package to extract specific kinematic values from your TM robot, which are taken into account by a Python script function using a specific set of commands to automatically generate a new Xacro robot model description file.
> If the user just wants to use the TM Robot nominal model to control the robot, the user can skip the rest of this chapter.<br/>

## &sect; Corrected kinematics value description
 > The precise kinematic parameters of a robot are useful for improving the end-point accuracy of the robot.<br/>
 > Due to manufacturing tolerances during manufacturing and the installation error in the robot assembly process, the positioning accuracy and precision of the mechanism will be affected. The error between the reality and the nominal robot model is significantly reduced by the corrected robot description. The kinematic parameter compensated deviations of the robot can improve the absolute positioning accuracy of the robot.<br/>
 > If the user needs to improve simulation accuracy or end effector tracking performance, it is recommended that the user import the corrected calibrated kinematic parameters from the real TM Robot to replace the nominal set of D-H parameters. Techman Robot provides the Xacro file that configures the TM Robot model with a set of nominal DH parameters, and one that uses the programming scripts to obtain calibrated kinematic parameters from a parameter server connected to your TM robot and perform a set of overrides to output a new corrected Xacro file.<br/>
 > <br/>
 > The common Python script is used as follows:
 >```bash
 > python3 <script_name> <urdf_from> <urdf_gen>
 >```
 > * <script_name> : Provide modify_xacro.py or modify_urdf.py two Python scripts program as options.
 > * <urdf_from>: The first argument represents the original URDF model form of the TM Robot, and the file part naming <sup>1</sup> is <urdf_from>.<br/>
 > <sup>1</sup> There will be several built-in TM Robot nominal robot model settings, available for TM5S, TM7S, TM12S, TM14S, and TM25S models.<br/>
 > For example, select your real robot type as the input nominal model form. If your TM robot is TM12S, then the user can type tm12s as the <urdf_from>.<br/>
 > * <urdf_gen>: The second argument means the newly generated URDF model form of the TM Robot, and the file <sup>2</sup> name is <urdf_gen>.<br/>
 > <sup>2</sup> For example, if the user names it test and select modify_xacro.py as script program, a test.urdf.xacro robot description file will be generated.<br/>
 >
 > The Python script for more specific arguments is used as follows:
 >```bash
 > python3 <script_name> <urdf_from> <urdf_gen> <specific_para>
 >```
 > * <specific_para>: The third argument is provided for use in some special cases. Please refer to the scripting program <sup>3</sup> for details of this item.<br/>
 > <sup>3</sup> For a simple third argument example, type the argument "-M" as follows:<br/>
 > Example : ``python3 modify_xacro.py tm12s test -M``<br/>
 >  &rarr; A robot description file "`macro.test.urdf.xacro`" will be generated, and the string 'macro.' is prepended to the <urdf_gen> name.<br/>


## &sect; Create with specific kinematic parameters of the local TM Robot
> :bulb: Do you run the driver to maintain the connection with TM Robot, make sure that TM Robot's operating software (TMflow) network settings are ready and the Listen node is running.<br/>
> <br/>
> * #### __Take generating a new Xacro file as an example__
> The following steps describe how to import specific kinematic values using a real TM12S Robot following the procedure below, and select the corresponding type tm12s as an example of <urdf_from>.<br/>
>
> 1. In a terminal: Source setup.bash in the workspace path and connect to TM Robot by typing<br/>
>
> ```bash
> source /opt/ros/humble/setup.bash
> cd <workspace>
> export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
> export ROS_DOMAIN_ID=<ROS_DOMAIN_ID>
> source ./install/setup.bash
> ```
> **Note**: Domain ID is the key to ROS communication, and please make sure the ROS node works under the ROS environment setup with the same Domain ID as the robot.<br/>
> 
> 2. In this terminal: Change the current directory to the directory path of the Python script to get the specific kinematic parameters of your TM Robot, and then enter the specified command format to generate a new name by the <urdf_gen> argument, for example, named user_defined.<br/>
> 
> ```bash
> cd src/tm_mod_urdf/tm_mod_urdf
> python3 modify_xacro.py tm12s user_defined
> ```
> When this procedure is completed, the user can find that the newly generated named robot description file has been saved, e.g."``user_defined.urdf.xacro``".<br/>
> :bookmark_tabs: Note: In the previous chapter, we renamed the download folder tm2_ros2 (or tm2_ros2-master) to src. If the user misses this step, they will encounter such an error "``[Error] [modify_xacro]: workspace directory not find ``" on the screen when executing the above command.<br/>
> 
> 3. Next, the user must modify the filename part of the default pre-built nominal robot model in tm12s.urdf.xacro to a newly generated robot model description naming file.<br/>
> ```bash
> cd src\tm_description\xacro\
> sudo vim tm12s.urdf.xacro
> ```
>>  or use ``gedit`` text editor instead of ``vim`` to edit the file contents, by typing<br/>
> ```bash
> sudo gedit tm12s.urdf.xacro
> ```
>
> :bookmark_tabs: Note: If your real Robot is a TM25S, in the above example, you should type tm25s as an example for <urdf_from> and modify the tm25s.urdf.xacro file.<br/>
>
> Please refer to the following to modify the content format of the filename line:<br/>
> ```bash
> # Before modification : (Take the pre-built TM12S nominal robot model as an example) 
>   <xacro:include filename="$(find tm_description)/xacro/macro.tm12s-nominal.urdf.xacro" />
> # After modification : (Replace with your actual newly generated Xacro file)
>   <xacro:include filename="$(find tm_description)/xacro/user_defined.urdf.xacro" />
> ```
> Finally, the user can launch the modified robot file "``tm12s.urdf.xacro``" to run your TM Robot or simulate the robot more accurately.<br/>
>> :bulb: **Tip**: Remember to recompile since the code has been changed.<br/>
>> Please go back to your specific workspace. Then you can choose `colcon build --cmake-clean-cache` to rebuild, or you can clean the build, install and log directories with `rm -r build install log` before executing `colcon build`.<br/>
>
>
## &sect; Import information available on the screen
>    *  How can the user confirm that the data conversion process has been completed?<br/>
> Ans: The user can find the string "``File saved with new kinematic values.``" displayed on the screen.<br/>
>    *  How can the user find the location of the newly generated named robot description file?<br/>
> Ans: The user can first find the displayed string "``[new save file path:] ``" on the screen, and the following string is the file save location.<br/>


