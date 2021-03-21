# april gazebo machinery

A collection of machines used in april project.

Simulation in gazebo 11 is provided.

# Installation

        sudo -H pip3 install xacro4sdf

# Conveyor belt

        roslaunch osrf_gear single_conveyor.launch
        rosservice call /gazebo/unpause_physics "{}"
        rosservice call /machinery/conveyor_belt_a/control "power: 75"

# Light indicator

        TODO
