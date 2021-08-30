scott@restfulrobotics.com

Aug 29 2021

References
1) https://answers.ros.org/question/216975/loading-variables-with-xacroinclude/
2) https://answers.ros.org/question/372407/xacro-macro-noetic-not-working/
3) https://answers.ros.org/question/10401/how-to-convert-xacro-file-to-urdf-file/
4) https://github.com/cyberbotics/urdf2webots
5) https://github.com/xArm-Developer/xarm_ros2
6) https://github.com/cyberbotics/urdf2webots
7) https://github.com/cyberbotics/urdf2webots/blob/master/docs/tutorial.md




Note:  This requires a ROS 1 installation to access the command line tools xacro

To generate the URDF file for the xarm7 for import into webots

1) Download / install xarm_ros2 package
2) Navigate to xarm_ros2/xarm_description/urdf/xarm7
3) Make a backup copy of xarm7.udrf.xacro => copy_xarm7.udrf.xacro
4) Make the following changes to the copy:

a) Comment out the <xacro:macro> sections; one is at the beginning of the file with the joint limits
and there is a matching one towards the end

b) Replace the above commented out xacro:macro section with (update values per original file) below

  <xacro:property name="joint1_lower_limit" value="${-2.0*pi}"/>
  <xacro:property name="joint1_upper_limit" value="${2.0*pi}"/>

  <xacro:property name="joint2_lower_limit" value="${-2.059}"/>
  <xacro:property name="joint2_upper_limit" value="${2.0944}"/>

  <xacro:property name="joint3_lower_limit" value="${-2.0*pi}"/>
  <xacro:property name="joint3_upper_limit" value="${2.0*pi}"/>

  <xacro:property name="joint4_lower_limit" value="${-0.19198}"/>
  <xacro:property name="joint4_upper_limit" value="${3.927}"/>

  <xacro:property name="joint5_lower_limit" value="${-2.0*pi}"/>
  <xacro:property name="joint5_upper_limit" value="${2.0*pi}"/>

  <xacro:property name="joint6_lower_limit" value="${-1.69297}"/>
  <xacro:property name="joint6_upper_limit" value="${pi}"/>

  <xacro:property name="joint7_lower_limit" value="${-2.0*pi}"/>
  <xacro:property name="joint7_upper_limit" value="${2.0*pi}"/>

c) Add an additional xacro property for the "prefix" variable

<xacro:property name="prefix" value="xarm_"/> 

d) For each line that specifies the paths to the meshes

<mesh filename="file:///$(find xarm_description)/meshes/xarm7/visual/link_base.STL"/>

replace with suitable paths that remove the macro variable

<mesh filename="/home/snortman/ros2_ws/src/xarm_ros2/xarm_description/meshes/xarm7/visual/link_base.STL"/>

e) save the file

5) Install ros2 xacro if needed
$ sudo apt-get install ros-foxy-xacro

6) Navigate to the correct folder, for example

$ cd /home/snortman/ros2_ws/src/xarm_ros2/xarm_description/urdf/xarm7

7) Generate the URDF with the command

$ ros2 run xacro xacro -o xarm7.urdf copy_xarm7.urdf.xacro

8) Open the generated URDF file an visually inspect

9) Convert the generate URDF into a webots PROTO

python3 -m urdf2webots.importer --input=../urdf/xarm7.urdf \
--output=xarm7.proto \
--normal \
--static-base \
--tool-slot="xarm_link_eef" \
--name-to-def \
--link-to-def \
--joint-to-def \
--rotation="1 0 -1.5708" \
--init-pos="0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0"

10) Confirm output files exist

11) Open / edit file:

a) Change 'supervisor' to TRUE
b) Change 'selfCollision' to TRUE
c) Change controller to "<extern>"


12) Copy / move to webots / protos folder






