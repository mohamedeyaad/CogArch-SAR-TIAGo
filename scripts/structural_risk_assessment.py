#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, WrenchStamped  
from std_msgs.msg import String
from tiago_sar_cogarch.msg import RiskReport

class StructuralRiskDummy:
    def __init__(self):
        rospy.init_node('structural_risk_dummy')

        # Subscribers
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/sonar_base", Range, self.sonar_callback)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/wrist_right_ft", WrenchStamped, self.wrench_callback)  

        # Subscriber for manual requests
        rospy.Subscriber("/manual_request", String, self.manual_request_callback)

        # Publishers
        self.risk_pub = rospy.Publisher("/risk_alert", RiskReport, queue_size=10)  # Modify the publisher to use the custom message
        self.move_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.reassessment_pub = rospy.Publisher("/reassessment_status", String, queue_size=10)

        # Dummy detection parameters
        self.crack_count = 0       # From RGB
        self.wall_anomalies = 0    # From LiDAR
        self.hollow_spaces = 0     # From Sonar
        self.current_pose = None
        self.force_magnitude = 0  # From WrenchStamped

        rospy.loginfo("Dummy Structural Risk Assessor Initialized")

    def rgb_callback(self, msg):
        """Dummy crack detection (0-4 cracks)"""
        if msg.height == 0 or msg.width == 0:
            rospy.logwarn("Invalid RGB image")
            return
        self.crack_count = np.random.randint(0, 5)
        rospy.loginfo(f"Dummy: Detected {self.crack_count} cracks")
        self.evaluate_risk()

    def lidar_callback(self, msg):
        """Dummy wall anomaly detection (0-2 anomalies)"""
        if not any(np.isfinite(r) for r in msg.ranges):
            rospy.logwarn("Invalid LiDAR reading")
            return
        self.wall_anomalies = np.random.randint(0, 3)
        rospy.loginfo(f"Dummy: Detected {self.wall_anomalies} wall anomalies")
        self.evaluate_risk()

    def sonar_callback(self, msg):
        """Dummy hollow space detection (0-2 voids)"""
        if not np.isfinite(msg.range):
            rospy.logwarn("Invalid sonar reading")
            return
        self.hollow_spaces = np.random.randint(0, 3)
        rospy.loginfo(f"Dummy: Detected {self.hollow_spaces} hollow spaces")
        self.evaluate_risk()

    def odom_callback(self, msg):
        """Store current position for dummy navigation"""
        self.current_pose = msg.pose.pose

    def wrench_callback(self, msg):
        """Dummy force detection from wrist sensor"""
        force = msg.wrench.force
        self.force_magnitude = np.sqrt(force.x**2 + force.y**2 + force.z**2)  # Calculate force magnitude
        rospy.loginfo(f"Dummy: Detected force magnitude = {self.force_magnitude:.2f} N")
        self.evaluate_risk()

    def manual_request_callback(self, msg):
        """
        Callback for handling manual requests.
        """
        rospy.loginfo(f"Received manual request: {msg.data}")
        # Process the manual request (trigger reassessment)
        if msg.data.lower() == "reassess":
            rospy.loginfo("Manual request: Triggering reassessment.")
            self.trigger_reassessment()
        else:
            rospy.logwarn(f"Unknown manual request: {msg.data}")

    def evaluate_risk(self):
        """Dummy risk classification logic"""
        risk_score = (
            (self.crack_count * 2) +
            self.wall_anomalies +
            (self.hollow_spaces * 3) +
            (self.force_magnitude > 8) * 2
        )

        if risk_score > 8:
            status = "HIGH"
        elif risk_score > 4:
            status = "MEDIUM"
        else:
            status = "LOW"

        # Create and populate the custom message
        report = RiskReport()
        report.status = status
        report.crack_count = self.crack_count
        report.wall_anomalies = self.wall_anomalies
        report.hollow_spaces = self.hollow_spaces
        report.force_magnitude = self.force_magnitude
        report.risk_score = risk_score

        self.risk_pub.publish(report)
        rospy.loginfo(f"Published Risk Alert: {report}")

    def trigger_reassessment(self):
        """Dummy autonomous movement for closer inspection"""
        if self.current_pose:
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = self.current_pose.position.x + 1.0  # Move 1m forward
            goal.pose.orientation.w = 1.0

            self.move_pub.publish(goal)
            self.reassessment_pub.publish("Moving closer for reassessment")
            rospy.loginfo("Dummy: Moving 1m closer for reassessment")

if __name__ == "__main__":
    node = StructuralRiskDummy()
    rospy.spin()
