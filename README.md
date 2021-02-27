# Conveyor belt

roslaunch osrf_gear only_conveyor.launch
rosservice call /gazebo/unpause_physics "{}"
rosservice call /ariac/conveyor/control "power: 75"
