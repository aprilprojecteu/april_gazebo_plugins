# april gazebo machinery

A collection of machines used in april project.

Simulation in gazebo 11 is provided.

# Conveyor belt

roslaunch osrf_gear only_conveyor.launch
rosservice call /gazebo/unpause_physics "{}"
rosservice call /ariac/conveyor/control "power: 75"
