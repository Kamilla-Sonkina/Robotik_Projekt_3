# Robotik_Projekt_3

## Install:

download the regler_package, the object_tracker_package, the object_interfaces, the ro45_portalrobot_interfaces, the ro45_ros2_pickrobot_serial and the target_interfaces in your source folder in your ros2 workspace. 


## Build:

build it using the colcon build command in your workspace
`'colcon build'`

## Run
make shure the robot is connected and run the ro45_ros2_pickrobot_serial is running. 
You can run it with the following command:

`ros2 launch ro45_ros2_pickrobot_serial launch_in_container.py`
For further instructions please read the documentation in https://github.com/matl-hsk/ro45_ros2_pickrobot_serial 

for the regler_node run the command:

`ros2 run regler_package regler_node`

for the object tracker node connect the camera and run the command
`
ros2 run object_tracker_package object_tracker_node`
