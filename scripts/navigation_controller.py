#!/usr/bin/env python3

"""
.. module:: navigation_controller
    :platform: ROS
    :synopsis: ROS node for robot navigation control and mission management

.. moduleauthor:: Your Name <your.email@example.com>

This node handles path following, obstacle avoidance, and mission status reporting.
"""

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class NavigationController:
    """
    Main navigation control class implementing path following and obstacle avoidance.
    
    Attributes:
        current_pose (PoseStamped): Stores current robot position from SLAM
        path (Path): Current navigation path to follow
        obstacle_detected (bool): Flag for obstacle presence
        mission_completed (bool): Flag for mission completion status
        cmd_pub (rospy.Publisher): Velocity command publisher
        mission_report_pub (rospy.Publisher): Mission status publisher
    """
    
    def __init__(self):
        """
        Initializes node, subscribers, publishers, and state variables.
        Sets up ROS communication infrastructure.
        """
        rospy.init_node('navigation_controller')

        # Subscribers
        rospy.Subscriber('/slam_3d/current_pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/planned_path', Path, self.path_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)

        # Publishers
        self.cmd_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        self.mission_report_pub = rospy.Publisher('/mission_report', String, queue_size=10)

        # State
        self.current_pose = None
        self.path = None
        self.obstacle_detected = False
        self.mission_completed = False

    def pose_callback(self, msg):
        """
        Updates current robot pose from SLAM system.
        
        :param msg: Incoming pose message
        :type msg: geometry_msgs.msg.PoseStamped
        """
        self.current_pose = msg

    def path_callback(self, msg):
        """
        Handles new navigation paths and resets mission status.
        
        :param msg: Incoming path message
        :type msg: nav_msgs.msg.Path
        """
        self.path = msg
        self.mission_completed = False  # Reset mission completion status
        rospy.loginfo("Received a new path with %d points.", len(msg.poses))

    def lidar_callback(self, msg):
        """
        Processes lidar data for obstacle detection.
        Implements simple threshold-based obstacle detection.
        
        :param msg: Incoming laser scan data
        :type msg: sensor_msgs.msg.LaserScan
        """
        # Dummy obstacle detection: if anything closer than 0.06 meters
        if any(distance < 0.06 for distance in msg.ranges if distance > 0):
            self.obstacle_detected = True
            rospy.logwarn_throttle(5, "Obstacle detected! Stopping robot.")
        else:
            self.obstacle_detected = False

    def move_along_path(self):
        """
        Generates velocity commands for path following.
        Implements simple constant-speed movement when path is available and no obstacles.
        """
        twist = Twist()
        if self.path and not self.obstacle_detected:
            # Dummy movement: always move forward with constant speed
            twist.linear.x = 0.2
            rospy.loginfo_throttle(5, "Moving along path.")
        else:
            twist.linear.x = 0.0
            rospy.loginfo_throttle(5, "Stopped due to obstacle or missing path.")
        
        self.cmd_pub.publish(twist)

    def check_mission_completion(self):
        """
        Checks mission completion status based on distance to final path point.
        Publishes mission report when completion criteria are met.
        Uses simplified distance threshold for demonstration purposes.
        """
        if self.path and not self.obstacle_detected and not self.mission_completed:
            # For simplicity, we'll assume the mission is complete when we reach the last point of the path
            if self.current_pose and self.path.poses:
                # Check if robot has reached the end of the path
                last_point = self.path.poses[-1]
                distance_to_goal = self.calculate_distance(self.current_pose.pose, last_point.pose)
                if distance_to_goal < 2:  # Threshold for reaching the goal
                    self.mission_completed = True
                    self.publish_mission_report()

    def calculate_distance(self, pose1, pose2):
        """
        Calculates Euclidean distance between two poses in 3D space.
        
        :param pose1: First position to compare
        :type pose1: geometry_msgs.msg.Pose
        :param pose2: Second position to compare
        :type pose2: geometry_msgs.msg.Pose
        :return: Straight-line distance between poses
        :rtype: float
        """
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        return (dx**2 + dy**2 + dz**2)**0.5

    def publish_mission_report(self):
        """
        Publishes mission completion status to ROS network.
        Implements simple success reporting for demonstration.
        """
        mission_status = "Mission successfully completed!"
        rospy.loginfo(f"[NavigationController] Publishing mission success: {mission_status}")
        self.mission_report_pub.publish(mission_status)

    def run(self):
        """
        Main control loop running at 10Hz.
        Coordinates movement and mission status checks.
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.move_along_path()
            self.check_mission_completion()
            rate.sleep()

if __name__ == '__main__':
    """
    Node entry point with exception handling for ROS interrupts.
    """
    try:
        node = NavigationController()
        node.run()
    except rospy.ROSInterruptException:
        pass
