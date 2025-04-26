#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

class NavigationController:
    def __init__(self):
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
        self.current_pose = msg

    def path_callback(self, msg):
        self.path = msg
        self.mission_completed = False  # Reset mission completion status
        rospy.loginfo("Received a new path with %d points.", len(msg.poses))

    def lidar_callback(self, msg):
        # Dummy obstacle detection: if anything closer than 0.06 meters
        if any(distance < 0.06 for distance in msg.ranges if distance > 0):
            self.obstacle_detected = True
            rospy.logwarn_throttle(5, "Obstacle detected! Stopping robot.")
        else:
            self.obstacle_detected = False

    def move_along_path(self):
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
        Check if the mission is completed (e.g., if the path is fully traversed).
        If mission is complete, publish a mission report.
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
        Calculate Euclidean distance between two poses.
        """
        dx = pose2.position.x - pose1.position.x
        dy = pose2.position.y - pose1.position.y
        dz = pose2.position.z - pose1.position.z
        return (dx**2 + dy**2 + dz**2)**0.5

    def publish_mission_report(self):
        """
        Publish mission report when mission is completed.
        """
        mission_status = "Mission successfully completed!"
        rospy.loginfo(f"[NavigationController] Publishing mission success: {mission_status}")
        self.mission_report_pub.publish(mission_status)

    def run(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.move_along_path()
            self.check_mission_completion()
            rate.sleep()

if __name__ == '__main__':
    try:
        node = NavigationController()
        node.run()
    except rospy.ROSInterruptException:
        pass
