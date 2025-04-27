#!/usr/bin/env python3

"""
.. module:: slam_node
    :platform: ROS
    :synopsis: Python module for SLAM (Simultaneous Localization and Mapping) implementation

.. moduleauthor:: Your Name <your.email@example.com>

This node handles sensor data integration and maintains a 3D map of the environment while tracking the robot's pose.
"""

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Range, PointCloud2
from geometry_msgs.msg import PoseStamped

class SlamNode:
    """
    Main SLAM node class that handles sensor subscriptions and map/pose publishing.
    
    Attributes:
        latest_pose (PoseStamped): Stores the most recent robot pose estimate
        map_pub (rospy.Publisher): Publisher for 3D map data
        pose_pub (rospy.Publisher): Publisher for current robot pose
    """
    
    def __init__(self):
        """
        Initializes the SLAM node, subscribers, and publishers.
        """
        rospy.init_node('slam_node')

        # Subscribers
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/xtion/depth/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/sonar_base', Range, self.sonar_callback)

        # Publishers
        self.map_pub = rospy.Publisher('/slam_3d/map', PointCloud2, queue_size=1)
        self.pose_pub = rospy.Publisher('/slam_3d/current_pose', PoseStamped, queue_size=1)

        # Latest pose
        self.latest_pose = PoseStamped()
        self.latest_pose.header.frame_id = "map"

    def odom_callback(self, msg):
        """
        Odometry callback function that updates the robot's pose estimation.
        
        :param msg: Incoming Odometry message
        :type msg: nav_msgs.msg.Odometry
        """
        # Convert odometry to pose in map frame (assuming tf is handled externally)
        self.latest_pose.header.stamp = rospy.Time.now()
        self.latest_pose.pose = msg.pose.pose
        rospy.loginfo_throttle(5, "Updated robot pose from odometry.")

    def lidar_callback(self, msg):
        """
        LiDAR callback function that processes 2D laser scan data.
        
        :param msg: Incoming LaserScan message
        :type msg: sensor_msgs.msg.LaserScan
        """
        rospy.loginfo_throttle(5, "Received LiDAR data.")

    def depth_callback(self, msg):
        """
        Depth image callback function that processes 3D depth information.
        
        :param msg: Incoming Image message from depth camera
        :type msg: sensor_msgs.msg.Image
        """
        rospy.loginfo_throttle(5, "Received depth camera data.")

    def sonar_callback(self, msg):
        """
        Sonar callback function that processes range data.
        
        :param msg: Incoming Range message from sonar
        :type msg: sensor_msgs.msg.Range
        """
        rospy.loginfo_throttle(5, "Received sonar data.")

    def publish_outputs(self):
        """
        Publishes SLAM outputs including 3D map and current pose estimate.
        Creates dummy map data for demonstration purposes.
        """
        # Dummy map
        dummy_map = PointCloud2()
        dummy_map.header.stamp = rospy.Time.now()
        dummy_map.header.frame_id = "map"
        self.map_pub.publish(dummy_map)

        # Current pose to path planner
        self.pose_pub.publish(self.latest_pose)

    def run(self):
        """
        Main node loop running at 10Hz. Continuously publishes SLAM outputs.
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_outputs()
            rate.sleep()

if __name__ == '__main__':
    """
    Entry point for the SLAM node.
    """
    try:
        node = SlamNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
