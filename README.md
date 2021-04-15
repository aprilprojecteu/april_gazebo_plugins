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

# gazebo_ros_vel

A plugin to exert a 6D twist velocity on a body.

An example on how to use this plugin can be found under urdf folder, it is a simple robot cube

that you can move in gazebo by publishing on cmd_vel topic.
