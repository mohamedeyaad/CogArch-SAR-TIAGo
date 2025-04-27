Robot Communication Flow Description
=====================================

This document describes the main nodes and their communication structure for the robot's cognitive and navigation system, based on the provided ROS graph.

.. image:: ../images/rosgraph.png
	:width: 1500px


Sensors
-------

- **/audio**

  Provides audio input for the victim detection system.

- **/rgb_raw**

  Provides raw RGB images from the RGB-D camera.

- **/depth_raw**

  Provides raw depth images from the RGB-D camera.

- **/odometry**

  Publishes odometry information about the mobile base's movement.

- **/sonar**

  Publishes sonar distance measurements.

- **/scan_lidar**

  Publishes LiDAR scans used for mapping and obstacle avoidance.

- **/force_sensor** and **/wrist_right_ft**

  Sensors included in the system but not currently connected in the displayed graph.

Nodes and Their Roles
----------------------

- **/xtion**

  A wrapper node that manages the RGB and Depth streams from the camera.

  Publishes:
  
  - /xtion/rgb/image_raw
  - /xtion/depth/image_raw

- **/mobile_base_controller**

  Handles low-level movement of the robot base.

  Publishes:

  - /mobile_base_controller/odom (odometry information)

- **/slam**

  Performs 2D Simultaneous Localization and Mapping (SLAM) based on LiDAR and sonar data.

  Subscribes to:

  - /scan
  - /sonar_base
  - /mobile_base_controller/odom

- **/slam_3d**

  Provides the current 3D pose of the robot.

  Publishes:

  - /slam_3d/current_pose

- **/path_planner**

  Plans navigation paths using the robot’s current position.

  Subscribes to:

  - /slam_3d/current_pose

  Publishes:

  - /planned_path

- **/navigation_controller**

  Executes the planned paths or manual goals sent by the operator.

  Subscribes to:

  - /move_base_simple/goal
  - /planned_path

  Publishes:

  - /mission_report

- **/move_base_simple**

  Interface to send simple manual goals to the robot.

  Subscribes to:

  - /move_base_simple/goal

- **/victim_detection**

  Detects victims using RGB images, depth images, and audio.

  Subscribes to:

  - /xtion/rgb/image_raw
  - /xtion/de

