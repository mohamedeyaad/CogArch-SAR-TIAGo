#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import Image, LaserScan, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

counter_flag =0

class StructuralRiskDummy:
    def __init__(self):
        rospy.init_node('structural_risk_dummy')

        # Subscribers
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/scan", LaserScan, self.lidar_callback)
        rospy.Subscriber("/sonar_base", Range, self.sonar_callback)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.odom_callback)

        # Publishers
        self.risk_pub = rospy.Publisher("/risk_alert", String, queue_size=10)
        self.move_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.reassessment_pub = rospy.Publisher("/reassessment_status", String, queue_size=10)

        # Dummy detection parameters
        self.crack_count = 0       # From RGB
        self.wall_anomalies = 0    # From LiDAR
        self.hollow_spaces = 0     # From Sonar
        self.current_pose = None

        rospy.loginfo("Dummy Structural Risk Assessor Initialized")

    def rgb_callback(self, msg):
        global counter_flag
        """Dummy crack detection (0-4 cracks)"""
        if msg.height == 0 or msg.width == 0:
            rospy.logwarn("Invalid RGB image")
            return
        self.crack_count = np.random.randint(0, 5)
        rospy.loginfo(f"Dummy: Detected {self.crack_count} cracks")
        counter_flag=counter_flag+1
        self.evaluate_risk()

    def lidar_callback(self, msg):
        global counter_flag
        """Dummy wall anomaly detection (0-2 anomalies)"""
        if not any(np.isfinite(r) for r in msg.ranges):
            rospy.logwarn("Invalid LiDAR reading")
            return
        self.wall_anomalies = np.random.randint(0, 3)
        rospy.loginfo(f"Dummy: Detected {self.wall_anomalies} wall anomalies")
        counter_flag=counter_flag+1
        self.evaluate_risk()

    def sonar_callback(self, msg):
        global counter_flag
        """Dummy hollow space detection (0-2 voids)"""
        if not np.isfinite(msg.range):
            rospy.logwarn("Invalid sonar reading")
            return
        self.hollow_spaces = np.random.randint(0, 3)
        rospy.loginfo(f"Dummy: Detected {self.hollow_spaces} hollow spaces")
        counter_flag=counter_flag+1
        self.evaluate_risk()

    def odom_callback(self, msg):
        """Store current position for dummy navigation"""
        self.current_pose = msg.pose.pose

    def evaluate_accuracy(self):
        global counter_flag
        if counter_flag == 3:    
            counter_flag = 0
            """Randomly determine whether to trigger reassessment"""
            accuracy = np.random.rand()  # Random float in [0.0, 1.0)
            rospy.loginfo(f"Dummy: Evaluated accuracy = {accuracy:.2f}")
            if accuracy < 0.4:
                self.crack_count=0
                self.wall_anomalies=0
                self.hollow_spaces=0
                self.trigger_reassessment()
                return False  # 👈 Tell the caller not to continue
            return True


    def evaluate_risk(self):
        """Dummy risk classification logic"""
        risk_score = (self.crack_count * 2) + self.wall_anomalies + (self.hollow_spaces * 3)

        # 👇 Skip the rest if accuracy is too low
        if not self.evaluate_accuracy():
            return

        if risk_score > 8:
            status = "HIGH"
        elif risk_score > 4:
            status = "MEDIUM"
        else:
            status = "LOW"

        report = (
            f"[{status} RISK] "
            f"Cracks: {self.crack_count}, "
            f"Anomalies: {self.wall_anomalies}, "
            f"Hollows: {self.hollow_spaces} "
            f"(Score: {risk_score})"
        )

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
