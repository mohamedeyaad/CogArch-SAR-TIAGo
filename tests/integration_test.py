#!/usr/bin/env python3
import rospy
import unittest
import time
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from tiago_sar_cogarch.msg import TriageReport, RiskReport
from geometry_msgs.msg import PoseStamped, Twist, WrenchStamped
from sensor_msgs.msg import LaserScan, PointCloud2, Range, Image
from audio_common_msgs.msg import AudioData

class TestSARIntegration(unittest.TestCase):
    """
    Integration test class to integrate and validate the Search and Rescue (SAR) system functionality.

    This class subscribes to various ROS topics related to victim alerts, triage reports, 
    risk assessment, navigation, sensor data, and more. It verifies that the system
    is functioning correctly by checking the data flow between different components.
    """

    @classmethod
    def setUpClass(cls):
        """
        Initialize the ROS node for the SAR integration test.

        This method is called once before all test cases are run to initialize the ROS node
        necessary for subscribing to the required topics.
        """
        rospy.init_node('test_sar_integration')

    def setUp(self):
        """
        Set up the test environment by initializing message storage and subscribers.

        This method is called before each test case to initialize the message containers
        and set up all necessary ROS subscribers. It also waits for 5 seconds to ensure
        all components are initialized properly.
        """
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
        self.sonar_data = []
        self.audio_data = []
        self.force_sensor_data = []
        self.depth_images = []
        self.rgb_images = []

        # Setup subscribers
        self.setup_subscribers()
        time.sleep(5)  # Wait for everything to initialize

    def setup_subscribers(self):
        """
        Set up all the ROS subscribers for various topics.

        This method subscribes to multiple ROS topics related to victim alerts, triage
        reports, movement commands, sensor data, and other relevant information.
        """
        rospy.Subscriber('/victim_alert', String, self._victim_cb)
        rospy.Subscriber('/triage_status', TriageReport, self._triage_cb)
        rospy.Subscriber('/risk_alert', RiskReport, self._risk_cb)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self._move_cb)
        rospy.Subscriber('/slam_3d/current_pose', PoseStamped, self._slam_pose_cb)
        rospy.Subscriber('/planned_path', Path, self._path_cb)
        rospy.Subscriber('/mission_report', String, self._mission_cb)
        rospy.Subscriber('/mobile_base_controller/cmd_vel', Twist, self._cmd_vel_cb)
        rospy.Subscriber('/scan', LaserScan, self._scan_cb)
        rospy.Subscriber('/slam_3d/map', PointCloud2, self._map_cb)
        rospy.Subscriber('/sonar_base', Range, self._sonar_cb)
        rospy.Subscriber('/audio', AudioData, self._audio_cb)
        rospy.Subscriber('/wrist_right_ft', WrenchStamped, self._force_sensor_cb)
        rospy.Subscriber('/xtion/depth/image_raw', Image, self._depth_image_cb)
        rospy.Subscriber('/xtion/rgb/image_raw', Image, self._rgb_image_cb)

    # Callback functions for each subscriber
    def _victim_cb(self, msg):
        """Callback function for victim alerts."""
        self.victim_alerts.append(msg.data)

    def _triage_cb(self, msg):
        """Callback function for triage reports."""
        self.triage_reports.append(msg)

    def _risk_cb(self, msg):
        """Callback function for risk assessment alerts."""
        self.risk_alerts.append(msg)

    def _move_cb(self, msg):
        """Callback function for movement commands."""
        self.movement_commands.append(msg)

    def _slam_pose_cb(self, msg):
        """Callback function for SLAM poses."""
        self.slam_poses.append(msg)

    def _path_cb(self, msg):
        """Callback function for planned paths."""
        self.planned_paths.append(msg)

    def _mission_cb(self, msg):
        """Callback function for mission reports."""
        self.mission_reports.append(msg.data)

    def _cmd_vel_cb(self, msg):
        """Callback function for velocity commands."""
        self.cmd_vel_msgs.append(msg)

    def _scan_cb(self, msg):
        """Callback function for LiDAR scan data."""
        self.scan_data.append(msg)

    def _map_cb(self, msg):
        """Callback function for SLAM map data."""
        self.slam_maps.append(msg)

    def _sonar_cb(self, msg):
        """Callback function for sonar data."""
        self.sonar_data.append(msg)

    def _audio_cb(self, msg):
        """Callback function for audio data."""
        self.audio_data.append(msg)

    def _force_sensor_cb(self, msg):
        """Callback function for force sensor data."""
        self.force_sensor_data.append(msg)

    def _depth_image_cb(self, msg):
        """Callback function for depth image data."""
        self.depth_images.append(msg)

    def _rgb_image_cb(self, msg):
        """Callback function for RGB image data."""
        self.rgb_images.append(msg)

    # Test cases
    def test_navigation_chain(self):
        """
        Verify the complete navigation pipeline.

        This test case verifies that the SLAM system produces valid pose and map data, 
        that path planning works correctly, and that movement commands are generated as expected.
        Finally, it checks if the mission has been completed successfully.
        """
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
        """
        Verify victim detection triggers triage assessment.

        This test case ensures that once a victim alert is received, a triage report is 
        generated and contains valid information such as priority and location.
        """
        self.wait_for_condition(
            lambda: len(self.victim_alerts) > 0 and len(self.triage_reports) > 0,
            "Victim processing chain failed",
            40
        )
        report = self.triage_reports[0]
        self.assertIn(report.priority, ['URGENT', 'STABLE'], "Invalid triage priority level")
        self.assertIsNotNone(report.location, "Missing location in triage report")

    def test_risk_assessment_flow(self):
        """
        Verify structural risk alerts are generated.

        This test case checks if the risk assessment system generates alerts with valid data 
        such as risk level, crack count, anomalies, and risk score.
        """
        self.wait_for_condition(
            lambda: len(self.risk_alerts) > 0,
            "No risk alerts generated",
            30
        )
        alert_msg = self.risk_alerts[0]
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

    def test_sensor_data_flow(self):
        """
        Verify all sensors are active.

        This test case verifies that all sensors (LiDAR, sonar, audio, force sensor, 
        depth images, RGB images) are providing data.
        """
        self.wait_for_condition(lambda: len(self.scan_data) > 0, "No LiDAR scan data", 20)
        self.wait_for_condition(lambda: len(self.sonar_data) > 0, "No SONAR data", 20)
        self.wait_for_condition(lambda: len(self.audio_data) > 0, "No microphone audio data", 20)
        self.wait_for_condition(lambda: len(self.force_sensor_data) > 0, "No force sensor data", 20)
        self.wait_for_condition(lambda: len(self.depth_images) > 0, "No depth images", 20)
        self.wait_for_condition(lambda: len(self.rgb_images) > 0, "No RGB images", 20)

    def wait_for_condition(self, condition, fail_msg, timeout):
        """
        Wait for a condition to be met within a specified timeout period.

        This utility method repeatedly checks if a given condition is true. If the condition 
        is not met within the specified timeout, the test fails with the provided failure message.

        :param condition: A lambda function representing the condition to check
        :param fail_msg: The message to display if the condition is not met
        :param timeout: The maximum time to wait for the condition (in seconds)
        """
        start_time = time.time()
        while not rospy.is_shutdown() and (time.time() - start_time) < timeout:
            if condition():
                return
            time.sleep(0.1)
        self.fail(fail_msg)

if __name__ == '__main__':
    import rostest
    rostest.rosrun('tiago_sar_cogarch', 'integration_test', TestSARIntegration)