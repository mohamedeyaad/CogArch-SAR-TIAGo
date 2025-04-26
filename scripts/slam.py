#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, LaserScan, Range, PointCloud2
from geometry_msgs.msg import PoseStamped

class SlamNode:
    def __init__(self):
        rospy.init_node('slam_node')

        # Subscribers
        rospy.Subscriber('/mobile_base_controller/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self.depth_callback)
        rospy.Subscriber('/sonar_base', Range, self.sonar_callback)

        # Publishers
        self.map_pub = rospy.Publisher('/slam_3d/map', PointCloud2, queue_size=1)
        self.pose_pub = rospy.Publisher('/slam_3d/current_pose', PoseStamped, queue_size=1)

        # Latest pose
        self.latest_pose = PoseStamped()
        self.latest_pose.header.frame_id = "map"

    def odom_callback(self, msg):
        # Convert odometry to pose in map frame (assuming tf is handled externally)
        self.latest_pose.header.stamp = rospy.Time.now()
        self.latest_pose.pose = msg.pose.pose
        rospy.loginfo_throttle(5, "Updated robot pose from odometry.")

    def lidar_callback(self, msg):
        rospy.loginfo_throttle(5, "Received LiDAR data.")

    def depth_callback(self, msg):
        rospy.loginfo_throttle(5, "Received depth camera data.")

    def sonar_callback(self, msg):
        rospy.loginfo_throttle(5, "Received sonar data.")

    def publish_outputs(self):
        # Dummy map
        dummy_map = PointCloud2()
        dummy_map.header.stamp = rospy.Time.now()
        dummy_map.header.frame_id = "map"
        self.map_pub.publish(dummy_map)

        # Current pose to path planner
        self.pose_pub.publish(self.latest_pose)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.publish_outputs()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = SlamNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

