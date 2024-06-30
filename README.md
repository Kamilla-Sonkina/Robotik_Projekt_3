# Robotik_Projekt_3

## Install:

download the regler_package, the object_tracker_package, the object_interfaces, the ro45_portalrobot_interfaces, the ro45_ros2_pickrobot_serial, the watchdog and the target_interfaces in your source folder in your ros2 workspace. 


## Build:

build it using the colcon build command in your workspace
`colcon build`

## Run
make shure the robot is connected and run the ro45_ros2_pickrobot_serial is running. 
You can run it with the following command:

`ros2 launch ro45_ros2_pickrobot_serial launch_in_container.py`
For further instructions please read the documentation in https://github.com/matl-hsk/ro45_ros2_pickrobot_serial 

for the regler_node run the command:

`ros2 run regler_package regler_node`

the watchdog checking the controlling node can be run with

`ros2 run watchdog watchdog`

if you want to visualize the robot digitaly

`ros2 run rviz rviz_publish` + `rviz2`


if you want to debug regler_node and see extended information of the code run the command:

`ros2 run regler_package regler_node --ros-args --log-level debug`

for the object tracker node connect the camera and run the command

`ros2 run object_tracker_package object_tracker_node`

