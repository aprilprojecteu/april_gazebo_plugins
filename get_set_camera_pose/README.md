# get_set_camera_pose

A gazebo system plugin to get and set the gazebo GUI camera pose to a custom value programatically.

(Hint: you would normally change the "GUI camera" view by simply dragging/rotating it with the mouse).

# Installation

Target system: Ubuntu 20.04 and ROS1 noetic.

No binaries are available, you need to clone, build and source this ROS pkg as you would do
with any other.

Is succesfull you should be able to locate the plugin under:

```bash
ls <your_catkin_ws>/devel/lib/libget_set_camera_pose.so
```

# Load plugin option 1 (easiest)

Run gazebo as you would normally do but set gui to false to prevent gzclient from running, e.g.:

```bash
roslaunch gazebo_ros empty_world.launch gui:=false
```

In a separate terminal, do:

```bash
rosrun gazebo_ros gzclient __name:=gazebo_client -g libget_set_camera_pose.so
```

open issue: gzclient is not dying when you launch it in a separate terminal, your options are:

- close gazebo window with mouse and then click on "force quit"
- with command: ```killall gzclient``` (easiest)
- make a launch file with gzclient like this one:

```xml
<?xml version="1.0"?>
<launch>

  <!-- start gazebo client -->
  <node name="gazebo_client" pkg="gazebo_ros" type="gzclient" respawn="false" output="screen"
        args="-g libget_set_camera_pose.so" />

</launch>
```

upon killing the launch file, after a while... the process will end.

# Load plugin option 2 (currently not working: see Load plugin option 1 or 3)

In a terminal type:

```bash
# start gazebo server in the background
gzserver &
# start gazebo client with option to load system plugin
gzclient -g libget_set_camera_pose.so
# plugin is now loaded, for plugin usage consult the section below "Use plugin"
# exit by pressing ctrl + c
# bring process back from background
fg
# optionally terminate gazebo server from another terminal
killall gzserver
```

NOTE: Unfortunately we have tried to load it via sdf unsuccesfully, but if you find a way please let me know!

```xml
<!-- DOES NOT WORK! -->

<!-- plugin to control the gui camera programatically -->
<plugin name="get_set_camera_pose" filename="libget_set_camera_pose.so"/>
```

# Load plugin option 3

Pass the name of your library as command line arguments to gzclient from launch file:

```xml
<!-- start gazebo client -->
<group if="$(arg gui)">
  <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="$(arg output)"
        args="-g libget_set_camera_pose.so" required="$(arg gui_required)"/>
</group>
```

Full xml launch file example:

```xml
<?xml version="1.0"?>
<launch>

  <!-- these are the arguments you can pass this launch file, for example paused:=true -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="extra_gazebo_args" default=""/>
  <arg name="gui" default="true"/>
  <arg name="recording" default="false"/>
  <!-- Note that 'headless' is currently non-functional.  See gazebo_ros_pkgs issue #491 (-r arg does not disable
       rendering, but instead enables recording). The arg definition has been left here to prevent breaking downstream
       launch files, but it does nothing. -->
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>
  <arg name="physics" default="ode"/>
  <arg name="verbose" default="false"/>
  <arg name="output" default="screen"/>
  <arg name="world_name" default="worlds/empty.world"/> <!-- Note: the world_name is with respect to GAZEBO_RESOURCE_PATH environmental variable -->
  <arg name="respawn_gazebo" default="false"/>
  <arg name="use_clock_frequency" default="false"/>
  <arg name="pub_clock_frequency" default="100"/>
  <arg name="enable_ros_network" default="true" />
  <arg name="server_required" default="false"/>
  <arg name="gui_required" default="false"/>

  <!-- set use_sim_time flag -->
  <param name="/use_sim_time" value="$(arg use_sim_time)"/>

  <!-- set command arguments -->
  <arg unless="$(arg paused)" name="command_arg1" value=""/>
  <arg     if="$(arg paused)" name="command_arg1" value="-u"/>
  <arg unless="$(arg recording)" name="command_arg2" value=""/>
  <arg     if="$(arg recording)" name="command_arg2" value="-r"/>
  <arg unless="$(arg verbose)" name="command_arg3" value=""/>
  <arg     if="$(arg verbose)" name="command_arg3" value="--verbose"/>
  <arg unless="$(arg debug)" name="script_type" value="gzserver"/>
  <arg     if="$(arg debug)" name="script_type" value="debug"/>

  <!-- start gazebo server-->
  <group if="$(arg use_clock_frequency)">
    <param name="gazebo/pub_clock_frequency" value="$(arg pub_clock_frequency)" />
  </group>
  <group>
    <param name="gazebo/enable_ros_network" value="$(arg enable_ros_network)" />
  </group>
  <node name="gazebo" pkg="gazebo_ros" type="$(arg script_type)" respawn="$(arg respawn_gazebo)" output="$(arg output)"
	args="$(arg command_arg1) $(arg command_arg2) $(arg command_arg3) -e $(arg physics) $(arg extra_gazebo_args) $(arg world_name)"
  required="$(arg server_required)" />

  <!-- start gazebo client -->
  <group if="$(arg gui)">
    <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="$(arg output)" args="-g libget_set_camera_pose.so"
    required="$(arg gui_required)"/>
  </group>

</launch>
```

Note that is almost identical to ```roslaunch gazebo_ros empty_world.launch``` but changing the line at the end from:

```xml
<node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="$(arg output)" args="$(arg command_arg3)"
      required="$(arg gui_required)"/>
```

```xml
<node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false" output="$(arg output)" args="-g libget_set_camera_pose.so"
      required="$(arg gui_required)"/>
```

# Call plugin

The plugin offers 2 services under the namespace gazebo_client called ```print_current_camera_pose```
and ```update_camera_pose```. As their names hint, the first one can be used to programatically get the
current camera pose (is printed on the console where gazebo was launched) and the second service allows
the user to set the gazebo GUI camera to a custom pose programatically. For example:

Select a custom view from the gazebo GUI by dragging or rotating the scene with the mouse. Then run:

```bash
# print to console the current gazebp GUI camera pose
rosservice call /gazebo_client/print_current_camera_pose "{}"
```

Then move the scene again to another view. Run the following command to restore the previous view:
(Modify with your pose values)

```bash
rosservice call /gazebo_client/update_camera_pose "pose:
  position:
    x: 7.72044
    y: 9.38877
    z: 4.7051
  orientation:
    x: 0.168527
    y: 0.352922
    z: -0.396589
    w: 0.830519"
```
