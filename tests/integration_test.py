#!/usr/bin/env python3
import rospy
import unittest
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from tiago_sar_cogarch.msg import TriageReport, RiskReport
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, PointCloud2

class TestSARIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node('test_sar_integration')

    def setUp(self):
        # Message storage
        self.victim_alerts = []
        self.triage_reports = []
        self.risk_alerts = []
        self.movement_commands = []
        self.slam_poses = []
        self.planned_paths = []
        self.mission_reports = []
        self.cmd_vel_msgs = []
        self.scan_data = []
        self.slam_maps = []

        # Setup subscribers
        self.setup_subscribers()
        time.sleep(5)  # Wait for nodes to initialize

    def setup_subscribers(self):
        # Existing subscribers
        rospy.Subscriber('/victim_alert', String, self._victim_cb)
        rospy.Subscriber('/triage_status', TriageReport, self._triage_cb)
        rospy.Subscriber('/risk_alert', RiskReport, self._risk_cb)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._move_cb)

        # New subscribers for additional topics
        rospy.Subscriber('/slam_3d/current_pose', PoseStamped, self._slam_pose_cb)
        rospy.Subscriber('/planned_path', Path, self._path_cb)
        rospy.Subscriber('/mission_report', String, self._mission_cb)
        rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self._cmd_vel_cb)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)
        rospy.Subscriber('/slam_3d/map', PointCloud2, self._map_cb)

    def _victim_cb(self, msg):
        self.victim_alerts.append(msg.data)

    def _triage_cb(self, msg):
        self.triage_reports.append(msg)

    def _risk_cb(self, msg):
        # Append the entire RiskReport message to the list for detailed analysis
        self.risk_alerts.append(msg)

    def _move_cb(self, msg):
        self.movement_commands.append(msg)

    def _slam_pose_cb(self, msg):
        self.slam_poses.append(msg)

    def _path_cb(self, msg):
        self.planned_paths.append(msg)

    def _mission_cb(self, msg):
        self.mission_reports.append(msg.data)

    def _cmd_vel_cb(self, msg):
        self.cmd_vel_msgs.append(msg)

    def _scan_cb(self, msg):
        self.scan_data.append(msg)

    def _map_cb(self, msg):
        self.slam_maps.append(msg)

    def test_navigation_chain(self):
        """Verify complete navigation pipeline"""
        # Verify SLAM outputs
        self.wait_for_condition(
            lambda: len(self.slam_poses) > 0 and len(self.slam_maps) > 0,
            "SLAM not producing pose/map data",
            30
        )

        # Check path planning
        self.wait_for_condition(
            lambda: len(self.planned_paths) > 0 and len(self.planned_paths[0].poses) > 0,
            "Path planning failed",
            30
        )

        # Verify movement commands
        self.wait_for_condition(
            lambda: len(self.cmd_vel_msgs) > 0,
            "No movement commands generated",
            20
        )

        # Check mission completion
        self.wait_for_condition(
            lambda: any("success" in report.lower() for report in self.mission_reports),
            "Mission not completed",
            60
        )

    def test_victim_processing_chain(self):
        """Verify victim detection triggers triage assessment"""
        self.wait_for_condition(
            lambda: len(self.victim_alerts) > 0 and len(self.triage_reports) > 0,
            "Victim processing chain failed",
            40
        )

        report = self.triage_reports[0]
        self.assertIn(report.priority, ['URGENT', 'STABLE'],
                    "Invalid triage priority level")
        self.assertIsNotNone(report.location,
                           "Missing location in triage report")

    def test_risk_assessment_flow(self):
        """Verify structural risk alerts are generated"""
        self.wait_for_condition(
            lambda: len(self.risk_alerts) > 0,
            "No risk alerts generated",
            30
        )

        alert_msg = self.risk_alerts[0]
        # Validate the fields in the RiskReport message
        self.assertIn(alert_msg.status, ['LOW', 'MEDIUM', 'HIGH'], 
                      "Invalid risk level in alert")
        self.assertGreaterEqual(alert_msg.crack_count, 0, 
                                "Invalid crack count in alert")
        self.assertGreaterEqual(alert_msg.wall_anomalies, 0, 
                                "Invalid wall anomalies count in alert")
        self.assertGreaterEqual(alert_msg.hollow_spaces, 0, 
                                "Invalid hollow spaces count in alert")
        self.assertGreater(alert_msg.force_magnitude, 0.0, 
                           "Invalid force magnitude in alert")
        self.assertGreater(alert_msg.risk_score, 0.0, 
                           "Invalid risk score in alert")

    def test_reassessment_mechanism(self):
        """Verify system triggers reassessment when needed"""
        self.wait_for_condition(
            lambda: len(self.movement_commands) > 0 or len(self.risk_alerts) == 0,
            "Reassessment mechanism failed",
            30
        )
        if len(self.risk_alerts) > 0 and self.risk_alerts[0].status != 'LOW':
            self.assertGreater(len(self.movement_commands), 0,
                               "Failed to trigger reassessment movement")

    def test_sensor_data_flow(self):
        """Verify sensor data is being processed"""
        self.wait_for_condition(
            lambda: len(self.scan_data) > 0,
            "No scan data processed",
            20
        )

    def wait_for_condition(self, condition, fail_msg, timeout):
        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            if condition():
                return
            time.sleep(0.1)
        self.fail(fail_msg)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('tiago_sar_cogarch', 'integration_test', TestSARIntegration)