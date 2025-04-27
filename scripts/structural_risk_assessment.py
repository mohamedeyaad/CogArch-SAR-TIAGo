#!/usr/bin/env python3
"""
Structural Risk Dummy Node.

This script defines a dummy structural risk assessment node in ROS that simulates 
the detection of cracks, wall anomalies, hollow spaces, and force magnitudes. 
It publishes risk alerts based on synthetic data and responds to manual reassessment requests.
"""

import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, WrenchStamped  
from std_msgs.msg import String
from tiago_sar_cogarch.msg import RiskReport

class StructuralRiskDummy:
    """
    A dummy class for simulating structural risk assessment.

    This class subscribes to various sensor topics, generates dummy detections,
    evaluates a risk score based on these detections, and publishes a risk report.
    """

    def __init__(self):
        """
        Initializes the StructuralRiskDummy node, sets up subscribers and publishers,
        and initializes dummy detection parameters.
        """
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
        self.risk_pub = rospy.Publisher("/risk_alert", RiskReport, queue_size=10)
        """Publisher for broadcasting risk alerts using a custom RiskReport message."""

        self.move_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        """Publisher for sending movement goals to navigate closer for reassessment."""

        self.reassessment_pub = rospy.Publisher("/reassessment_status", String, queue_size=10)
        """Publisher for notifying about reassessment status."""

        # Dummy detection parameters
        self.crack_count = 0       #: Dummy crack count detected from RGB images.
        self.wall_anomalies = 0    #: Dummy wall anomalies detected from LiDAR scans.
        self.hollow_spaces = 0     #: Dummy hollow spaces detected from sonar readings.
        self.current_pose = None   #: Current robot pose from odometry.
        self.force_magnitude = 0   #: Magnitude of force detected from wrist sensor.

        rospy.loginfo("Dummy Structural Risk Assessor Initialized")

    def rgb_callback(self, msg):
        """
        Callback for RGB image topic.

        Simulates the detection of cracks by randomly generating a number between 0 and 4.
        Logs the number of detected cracks and triggers risk evaluation.

        :param msg: Incoming RGB image message.
        :type msg: sensor_msgs.msg.Image
        """
        if msg.height == 0 or msg.width == 0:
            rospy.logwarn("Invalid RGB image")
            return
        self.crack_count = np.random.randint(0, 5)
        rospy.loginfo(f"Dummy: Detected {self.crack_count} cracks")
        self.evaluate_risk()

    def lidar_callback(self, msg):
        """
        Callback for LiDAR scan topic.

        Simulates the detection of wall anomalies by randomly generating a number between 0 and 2.
        Logs the number of detected anomalies and triggers risk evaluation.

        :param msg: Incoming LiDAR scan message.
        :type msg: sensor_msgs.msg.LaserScan
        """
        if not any(np.isfinite(r) for r in msg.ranges):
            rospy.logwarn("Invalid LiDAR reading")
            return
        self.wall_anomalies = np.random.randint(0, 3)
        rospy.loginfo(f"Dummy: Detected {self.wall_anomalies} wall anomalies")
        self.evaluate_risk()

    def sonar_callback(self, msg):
        """
        Callback for sonar range topic.

        Simulates the detection of hollow spaces by randomly generating a number between 0 and 2.
        Logs the number of detected hollow spaces and triggers risk evaluation.

        :param msg: Incoming sonar range message.
        :type msg: sensor_msgs.msg.Range
        """
        if not np.isfinite(msg.range):
            rospy.logwarn("Invalid sonar reading")
            return
        self.hollow_spaces = np.random.randint(0, 3)
        rospy.loginfo(f"Dummy: Detected {self.hollow_spaces} hollow spaces")
        self.evaluate_risk()

    def odom_callback(self, msg):
        """
        Callback for odometry topic.

        Stores the current pose of the robot for future navigation tasks (e.g., moving closer to inspect).

        :param msg: Incoming odometry message.
        :type msg: nav_msgs.msg.Odometry
        """
        self.current_pose = msg.pose.pose

    def wrench_callback(self, msg):
        """
        Callback for force/torque sensor topic.

        Calculates the magnitude of the force vector from the wrench message.
        Logs the detected force and triggers risk evaluation.

        :param msg: Incoming wrench message containing force data.
        :type msg: geometry_msgs.msg.WrenchStamped
        """
        force = msg.wrench.force
        self.force_magnitude = np.sqrt(force.x**2 + force.y**2 + force.z**2)
        rospy.loginfo(f"Dummy: Detected force magnitude = {self.force_magnitude:.2f} N")
        self.evaluate_risk()

    def manual_request_callback(self, msg):
        """
        Callback for manual reassessment requests.

        Processes the manual command received (e.g., "reassess" command to trigger reassessment).

        :param msg: Incoming manual request message.
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"Received manual request: {msg.data}")
        if msg.data.lower() == "reassess":
            rospy.loginfo("Manual request: Triggering reassessment.")
            self.trigger_reassessment()
        else:
            rospy.logwarn(f"Unknown manual request: {msg.data}")

    def evaluate_risk(self):
        """
        Evaluates the current structural risk based on dummy detection values.

        Computes a risk score and categorizes it into LOW, MEDIUM, or HIGH risk.
        Publishes a RiskReport message with detailed detection data and the computed risk status.
        """
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
        """
        Triggers a dummy reassessment movement.

        If the current pose is known, sends a goal 1 meter forward from the current position
        and notifies that the robot is moving for reassessment.
        """
        if self.current_pose:
            goal = PoseStamped()
            goal.header.stamp = rospy.Time.now()
            goal.header.frame_id = "map"
            goal.pose.position.x = self.current_pose.position.x + 1.0  # Move 1 meter forward
            goal.pose.orientation.w = 1.0

            self.move_pub.publish(goal)
            self.reassessment_pub.publish("Moving closer for reassessment")
            rospy.loginfo("Dummy: Moving 1m closer for reassessment")

if __name__ == "__main__":
    """
    Main execution block.

    Instantiates the StructuralRiskDummy node and enters the ROS event loop.
    """
    node = StructuralRiskDummy()
    rospy.spin()

