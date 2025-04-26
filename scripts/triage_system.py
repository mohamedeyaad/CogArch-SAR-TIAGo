#!/usr/bin/env python3
"""
TriageSystem Node
------------------
A ROS-based triage system that evaluates victim status in disaster response scenarios.

Subscribes to:
- /victim_location (nav_msgs/Odometry): Location of detected victims
- /xtion/rgb/image_raw (sensor_msgs/Image): RGB camera feed for visual analysis
- /audio (audio_common_msgs/AudioData): Audio stream for responsiveness checks

Publishes:
- /triage_status (tiago_sar_cogarch/TriageReport): Comprehensive triage assessment

Services:
- Uses /speaker (tiago_sar_cogarch/Speaker): Text-to-speech service for victim interaction
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
    """Main class implementing triage assessment logic.
    
    The system evaluates victims through three primary modalities:
    1. Visual consciousness detection (movement analysis)
    2. Audio responsiveness verification
    3. Injury detection through computer vision
    
    Attributes:
        bridge (CvBridge): ROS-OpenCV image converter
        triage_pub (rospy.Publisher): Triage report publisher
        speaker_srv (rospy.ServiceProxy): Text-to-speech service client
        current_victim (geometry_msgs.Pose): Current victim location
        last_question_time (rospy.Time): Timestamp of last audio prompt
        response_timeout (float): Time window for victim response (seconds)
        prev_img (numpy.ndarray): Previous image frame for motion detection
    """

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
        """Handle new victim location updates.
        
        Args:
            msg (nav_msgs/Odometry): Victim position message
        """
        self.current_victim = msg.pose.pose
        rospy.loginfo(f"[TriageSystem] New victim located at: {self.current_victim.position}")

    def rgb_callback(self, msg):
        """Process RGB images for visual assessment.
        
        Args:
            msg (sensor_msgs/Image): Incoming RGB image
            
        Performs:
        - Movement detection for consciousness assessment
        - Injury detection through computer vision
        - Audio prompt triggering for unresponsive victims
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
        """Process audio data for victim response detection.
        
        Args:
            msg (audio_common_msgs/AudioData): Raw audio samples
            
        Triggers responsiveness assessment if within response timeout window.
        """
        if self.last_question_time and (rospy.Time.now() - self.last_question_time).to_sec() < self.response_timeout:
            audio_data = np.frombuffer(msg.data, dtype=np.int16)
            if self.detect_voice(audio_data):
                img = self.prev_img if self.prev_img is not None else np.zeros((1, 1, 3), dtype=np.uint8)
                injuries = self.detect_injuries(img)
                self.publish_report(conscious=False, responsive=True, injuries=injuries)
                self.last_question_time = None

    def detect_movement(self, image):
        """Detect inter-frame motion using absolute difference.
        
        Args:
            image (numpy.ndarray): Current frame in BGR format
            
        Returns:
            bool: True if mean frame difference exceeds threshold (10)
        """
        if self.prev_img is None:
            self.prev_img = image
            return False

        diff = cv2.absdiff(self.prev_img, image)
        self.prev_img = image
        return diff.mean() > 10  # Arbitrary threshold

    def detect_voice(self, data, threshold=0.02):
        """Basic voice activity detection using RMS energy.
        
        Args:
            data (numpy.ndarray): Audio samples as 16-bit integers
            threshold (float): RMS threshold for voice detection
            
        Returns:
            bool: True if audio energy exceeds threshold
        """
        rms = np.sqrt(np.mean(np.square(data.astype(np.float32))))
        return rms > threshold

    def detect_injuries(self, image):
        """Detect potential injuries through visual analysis.
        
        Args:
            image (numpy.ndarray): Input image in BGR format
            
        Returns:
            list: Detected injury descriptors (bleeding, suspected_fracture)
            
        Algorithm:
        1. Bleeding detection: Large red regions in HSV color space
        2. Fracture suspicion: High edge complexity through Canny/contour analysis
        """
        injuries = []

        # Detect bleeding in HSV space
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        if cv2.countNonZero(mask) > 1000:
            injuries.append("bleeding")

        # Detect fracture suspicion through edge complexity
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 100, 200)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 20:
            injuries.append("suspected_fracture")

        return injuries

    def publish_report(self, conscious, responsive, injuries):
        """Publish triage assessment report.

        Args:
            conscious (bool): Consciousness status from movement detection
            responsive (bool): Responsiveness status from audio interaction
            injuries (list): Detected injury descriptors
        """
        if not self.current_victim:
            return

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
    """Main entry point for TriageSystem node."""
    rospy.init_node("triage_system")
    rospy.loginfo("[TriageSystem] Node started.")
    TriageSystem()
    rospy.spin()