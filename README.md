# april gazebo plugins

A collection of gazebo plugins used in april project.

Simulation in gazebo 11 is provided.

# Installation

        sudo -H pip3 install xacro4sdf

# Conveyor belt plugin

A standalone version of the conveyor belt from osrf gazebo ariac competition.

To test:

        roslaunch conveyor_belt_sim single_conveyor.launch
        rosservice call /gazebo/unpause_physics "{}"
        rosservice call /machinery/conveyor_belt_a/control "power: 75"

# gazebo_ros_vel plugin

A plugin to exert a 6D twist velocity on a body.

An example on how to use this plugin can be found under urdf folder, it is a simple robot cube

that you can move in gazebo by publishing on cmd_vel topic.

# gazebo get_set_camera_pose plugin

A gazebo system plugin to get and set the gazebo GUI camera pose to a custom value programatically.

(Hint: you would normally change the "GUI camera" view by simply dragging/rotating it with the mouse).

See documentation inside get_set_camera_pose folder for more details.
