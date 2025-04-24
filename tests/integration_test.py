#!/usr/bin/env python3
import rospy
import unittest
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from tiago_sar_cogarch.msg import TriageReport
from geometry_msgs.msg import PoseStamped

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

        # Setup subscribers
        self.victim_sub = rospy.Subscriber('/victim_alert', String, self._victim_cb)
        self.triage_sub = rospy.Subscriber('/triage_status', TriageReport, self._triage_cb)
        self.risk_sub = rospy.Subscriber('/risk_alert', String, self._risk_cb)
        self.move_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._move_cb)
        
        # Wait for nodes to initialize
        time.sleep(5)

    def _victim_cb(self, msg):
        self.victim_alerts.append(msg.data)

    def _triage_cb(self, msg):
        self.triage_reports.append(msg)

    def _risk_cb(self, msg):
        self.risk_alerts.append(msg.data)

    def _move_cb(self, msg):
        self.movement_commands.append(msg)

    def test_victim_processing_chain(self):
        rospy.loginfo("Running test_victim_processing_chain")
        """Verify victim detection triggers triage assessment"""
        timeout = 30  # seconds
        start_time = time.time()

        while (time.time() - start_time) < timeout and not self.triage_reports:
            time.sleep(0.1)

        self.assertGreater(len(self.victim_alerts), 0, 
                         "No victim alerts detected")
        self.assertGreater(len(self.triage_reports), 0,
                         "Triage system failed to process victims")
        
        report = self.triage_reports[0]
        self.assertIn(report.priority, ['URGENT', 'STABLE'],
                    "Invalid triage priority level")
        self.assertIsNotNone(report.location,
                           "Missing location in triage report")

    def test_risk_assessment_flow(self):
        """Verify structural risk alerts are generated"""
        timeout = 30
        start_time = time.time()

        while (time.time() - start_time) < timeout and not self.risk_alerts:
            time.sleep(0.1)

        self.assertGreater(len(self.risk_alerts), 0,
                         "No risk alerts generated")
        
        alert_msg = self.risk_alerts[0]
        self.assertTrue(any(status in alert_msg 
                          for status in ['LOW', 'MEDIUM', 'HIGH']),
                        "Invalid risk level in alert")

    def test_reassessment_mechanism(self):
        """Verify system triggers reassessment when needed"""
        timeout = 30
        start_time = time.time()

        while (time.time() - start_time) < timeout and not self.movement_commands:
            time.sleep(0.1)

        if len(self.risk_alerts) > 0 and 'LOW' not in self.risk_alerts[0]:
            self.assertGreater(len(self.movement_commands), 0,
                              "Failed to trigger reassessment movement")

if __name__ == '__main__':
    import rostest
    rostest.rosrun('tiago_sar_cogarch', 
                  'integration_test',
                  TestSARIntegration)