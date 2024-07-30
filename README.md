# Robotik_Projekt_3

# Initialization and Setup for Objeckdetection

1. **Initialize the Coordinate System:**
   - The coordinate system must be initialized at the start by updating the pixel coordinates of the origin point (marked on the conveyor belt) in the code.
   - Once the system starts, the camera must remain stationary to prevent any shift in the coordinate system.

2. **Camera Position:**
   - Ensure that the camera is fixed and does not move after the system starts, as any movement will cause the coordinate system to shift.

3. **Aruco Marker:**
   - An Aruco marker is used for perspective transformation. Ensure that the marker is fully visible in the image for accurate transformation.

4. **YOLO Model Integration:**
   - Integrate the YOLO model into the system for object detection. Ensure the model is properly loaded and configured within the code.

By following these steps, you will set up the system correctly for accurate object tracking and coordinate mapping.



## Install:

Download the regler_package, the object_tracker_package, the object_interfaces, target_interfaces, the ro45_portalrobot_interfaces, the ro45_ros2_pickrobot_serial, the watchdog and rviz in your source folder in your ros2 workspace. 


## Build:

Build it using the colcon build command in your workspace
`colcon build`

## Run
Make shure the robot is connected and the ro45_ros2_pickrobot_serial is running. 
You can run it with the following command:

`ros2 launch ro45_ros2_pickrobot_serial launch_in_container.py`
For further instructions please read the documentation in https://github.com/matl-hsk/ro45_ros2_pickrobot_serial 
Please wait between these commands to make shure everything is running.
For the regler_node run the command:

`ros2 run regler_package regler_node` if you want to see further details of this node attach `--ros-args --log-level debug`

The watchdog checking the controlling node can be run with

`ros2 run watchdog watchdog`

If you want to visualize the robot digitaly run 

`ros2 run rviz rviz_publish` and `rviz2`

For the object tracker node connect the camera and run the command

`ros2 run object_tracker_package object_tracker_node`

If you want to restart the regler you also have to restart the rviz publisher. Please be carefull with turning of the robot, because the arduinos save the currents velocity and keep the motors running.

