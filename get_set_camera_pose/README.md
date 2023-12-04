# get_set_camera_pose

A gazebo system plugin to get and set the gazebo GUI camera pose to a custom value programatically.

(Hint: you would normally change the "GUI camera" view by simply dragging/rotating it with the mouse).

# Installation

Target system: Ubuntu 20.04 and ROS1 noetic.

No binaries are available, you need to clone, build and source this ROS pkg as you would do with any other.

Is succesfull you should be able to locate the plugin under:

```bash
ls <your_catkin_ws>/devel/lib/libget_set_camera_pose.so
```

# Load plugin

Launch gazebo as you would normally do, but set gui to false to prevent gzclient from running, e.g.:

```bash
roslaunch gazebo_ros empty_world.launch gui:=false
```

In a separate terminal, do:

```bash
rosrun gazebo_ros gzclient __name:=gazebo_gui -g libget_set_camera_pose.so
```

A launch file is provided for your convenience:

```xml
<?xml version="1.0"?>
<launch>

  <arg name="gz_args" default="-g libget_set_camera_pose.so"
       doc="additional option: --verbose"/>

  <!-- start gazebo client -->
  <node name="gazebo_gui" pkg="gazebo_ros" type="gzclient" respawn="false"
        output="screen" args="$(arg gz_args)" >
    <remap from="/gazebo_gui/get_physics_properties" to="/gazebo/get_physics_properties" />
    <remap from="/gazebo_gui/set_physics_properties" to="/gazebo/set_physics_properties" />
  </node>

</launch>
```

Or simply launch:

```bash
roslaunch get_set_camera_pose gzclient.launch
```

# Call plugin services

The plugin offers 2 services under the namespace gazebo_gui called ```print_current_camera_pose```
and ```update_camera_pose```. As their names hint, the first one can be used to programatically get the
current camera pose (is printed on the console where gazebo was launched) and the second service allows
the user to set the gazebo GUI camera to a custom pose programatically. For example:

Select a custom view from the gazebo GUI by dragging or rotating the scene with the mouse. Then run:

```bash
# print to console the current gazebp GUI camera pose
rosservice call /gazebo_gui/print_current_camera_pose "{}"
```

Then move the scene again to another view. Run the following command to restore the previous view:
(Modify with your pose values)

```bash
rosservice call /gazebo_gui/update_camera_pose "pose:
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
