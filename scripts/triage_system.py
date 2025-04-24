#!/usr/bin/env python3
"""
TriageSystem Node
------------------
A ROS-based triage system that evaluates victim status in disaster response
scenarios using RGB camera, audio, and location data. It detects:
    - Consciousness via movement (RGB)
    - Responsiveness via audio response
    - Injuries via visual cues
And publishes a comprehensive triage report.
"""

import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from audio_common_msgs.msg import AudioData
from nav_msgs.msg import Odometry
from tiago_sar_cogarch.srv import Speaker, SpeakerRequest
from tiago_sar_cogarch.msg import TriageReport

class TriageSystem:
    def __init__(self):
        """Initialize node, subscribers, publisher, and service client."""
        self.bridge = CvBridge()

        # ROS topic subscribers
        rospy.Subscriber("/victim_location", Odometry, self.victim_callback)
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/audio", AudioData, self.audio_callback)

        # Triage report publisher
        self.triage_pub = rospy.Publisher("/triage_status", TriageReport, queue_size=10)

        # Text-to-speech service client
        self.speaker_srv = rospy.ServiceProxy('/speaker', Speaker)

        # Internal tracking state
        self.current_victim = None              # Odometry.pose of current victim
        self.last_question_time = None          # Time question was asked
        self.response_timeout = 0.1             # Response window (seconds)
        self.prev_img = None                    # For optical flow detection

    def victim_callback(self, msg):
        """
        Called when a victim is detected.
        Stores victim's pose from Odometry data.
        """
        self.current_victim = msg.pose.pose
        rospy.loginfo(f"[TriageSystem] New victim located at: {self.current_victim.position}")

    def rgb_callback(self, msg):
        """
        Processes incoming RGB image:
        - Detects movement (consciousness)
        - Detects visual injuries
        - Prompts audio response if needed
        """
        if not self.current_victim:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            conscious = self.detect_movement(cv_image)
            injuries = self.detect_injuries(cv_image)

            if conscious:
                self.publish_report(conscious=True, responsive=None, injuries=injuries)
            else:
                try:
                    response = self.speaker_srv("Can you hear me? Say 'help' if you need assistance.")
                    if response.success:
                        self.last_question_time = rospy.Time.now()
                except rospy.ServiceException:
                    rospy.logwarn("[TriageSystem] Speaker service call failed.")
                    self.publish_report(conscious=False, responsive=False, injuries=injuries)

        except Exception as e:
            rospy.logerr(f"[TriageSystem] Error processing image: {str(e)}")

    def audio_callback(self, msg):
        """
        Listens for voice response if question has been asked recently.
        If voice is detected, victim is marked as responsive.
        """
        if self.last_question_time and (rospy.Time.now() - self.last_question_time).to_sec() < self.response_timeout:
            audio_data = np.frombuffer(msg.data, dtype=np.int16)
            if self.detect_voice(audio_data):
                img = self.prev_img if self.prev_img is not None else np.zeros((1, 1, 3), dtype=np.uint8)
                injuries = self.detect_injuries(img)
                self.publish_report(conscious=False, responsive=True, injuries=injuries)
                self.last_question_time = None

    def detect_movement(self, image):
        """
        Detect movement by comparing current and previous image frames.
        Returns True if significant motion is detected.
        """
        if self.prev_img is None:
            self.prev_img = image
            return False

        diff = cv2.absdiff(self.prev_img, image)
        self.prev_img = image
        return diff.mean() > 10  # Arbitrary threshold

    def detect_voice(self, data, threshold=0.02):
        """
        Basic voice activity detection using RMS energy.
        Returns True if energy exceeds given threshold.
        """
        rms = np.sqrt(np.mean(np.square(data.astype(np.float32))))
        return rms > threshold

    def detect_injuries(self, image):
        """
        Dummy injury detection using basic image features:
        - Bleeding: large red regions in HSV space
        - Suspected fractures: high contour complexity
        Returns a list of injury descriptors.
        """
        injuries = []

        # Detect bleeding
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        if cv2.countNonZero(mask) > 1000:
            injuries.append("bleeding")

        # Detect suspected fracture via edge complexity
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 20:
            injuries.append("suspected_fracture")

        return injuries

    def publish_report(self, conscious, responsive, injuries):
        """
        Constructs and publishes a TriageReport message with:
        - Consciousness status
        - Responsiveness status
        - Detected injuries
        - Priority level
        - Victim location
        """
        report = TriageReport()
        report.header.stamp = rospy.Time.now()
        report.consciousness = "conscious" if conscious else "unconscious"
        report.responsive = "yes" if responsive else "no"
        report.injuries = injuries

        # Compute urgency
        urgent = not conscious or not responsive or len(injuries) > 0
        report.priority = "URGENT" if urgent else "STABLE"
        report.location = self.current_victim

        self.triage_pub.publish(report)
        rospy.loginfo(f"[TriageSystem] Published triage report: {report}")
        self.current_victim = None  # Reset victim after reporting

if __name__ == "__main__":
    rospy.init_node("triage_system")
    rospy.loginfo("[TriageSystem] Node started.")
    TriageSystem()
    rospy.spin()

