#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from audio_common_msgs.msg import AudioData
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_msgs.msg import String
from cv_bridge import CvBridge


class VictimDetector:
    """
    A ROS-based victim detection class using RGB images, depth data, audio, and odometry.

    Subscribes to:
        - /xtion/rgb/image_raw: RGB camera feed.
        - /xtion/depth/image_raw: Depth image feed.
        - /mobile_base_controller/odom: Robot odometry.
        - /audio: Audio data.

    Publishes:
        - /victim_location: Estimated location of detected victim.
        - /victim_alert: Alert message when a victim is detected.
    """

    def __init__(self):
        """
        Initializes the VictimDetector class by subscribing to the necessary topics
        and setting up publishers.
        """	
        rospy.Subscriber("/xtion/rgb/image_raw", Image, self.rgb_callback)
        rospy.Subscriber("/xtion/depth/image_raw", Image, self.depth_callback)
        rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/audio", AudioData, self.audio_callback)
        self.victim_loc_pub = rospy.Publisher("/victim_location", Odometry, queue_size=10)
        self.victim_alert_pub = rospy.Publisher("/victim_alert", String, queue_size=10)
        
        # Initialize attributes
        self.current_odom = None
        self.current_depth = None
        self.latest_depth_image = None

    def rgb_callback(self, rgb_msg):
        """
        Callback for RGB image messages.

        :param rgb_msg: Image message containing RGB data.
        :type rgb_msg: sensor_msgs.msg.Image
        """
        rospy.loginfo("RGB image received!")
        see_victim = self.detect_victim_rgb(rgb_msg)
        rospy.loginfo(f"Victim detected by camera: {see_victim}")
        if see_victim:
            self.report_victim()  
    
    def audio_callback(self, audio_msg):
        """
        Callback for audio messages.

        :param audio_msg: AudioData message containing sound samples.
        :type audio_msg: audio_common_msgs.msg.AudioData
        """
        rospy.loginfo("Audio received!")
        hear_victim = self.detect_victim_audio(audio_msg, threshold=500)
        rospy.loginfo(f"Victim detected by audio: {hear_victim}")
        if hear_victim:
            self.report_victim()
        
    def odom_callback(self, odom_msg):
        """
        Callback for odometry messages.

        :param odom_msg: Odometry data.
        :type odom_msg: nav_msgs.msg.Odometry
        """
        self.current_odom = odom_msg
        rospy.loginfo("Odometry received!")    
    
    def depth_callback(self, depth_msg):
        """
        Callback for depth image messages.

        :param depth_msg: Depth image.
        :type depth_msg: sensor_msgs.msg.Image
        """
        rospy.loginfo("Depth data received!")  
        try:
            bridge = CvBridge()
            self.latest_depth_image = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
            self.depth_header = depth_msg.header
            
        except Exception as e:
            rospy.logerr(f"Failed to convert depth image: {e}")
            self.latest_depth_image = None

    def detect_victim_rgb(self, rgb_msg):
        """
        Detects a victim using RGB image data by looking for red color blobs.

        :param rgb_msg: Image message with RGB data.
        :type rgb_msg: sensor_msgs.msg.Image
        :return: True if a victim is detected, otherwise False.
        :rtype: bool
        """
        try:
            # Convert ROS Image to OpenCV
            img = np.frombuffer(rgb_msg.data, dtype=np.uint8).reshape(rgb_msg.height, rgb_msg.width, -1)
            
            # Handle possible alpha channel (4 channels)
            if img.shape[2] == 4:
                img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
            
            # Detect red regions
            lower_red = np.array([0, 0, 100])
            upper_red = np.array([100, 100, 255])  # Wider range
            mask = cv2.inRange(img, lower_red, upper_red)
            
            # Debug: Save images
            cv2.imwrite("/tmp/latest_image.png", img)
            cv2.imwrite("/tmp/red_mask.png", mask)
            
            return cv2.countNonZero(mask) > 100  # Lower threshold for testing
        except Exception as e:
            rospy.logerr(f"Detection failed: {e}")
            return False   
    
    def detect_victim_audio(self, audio_msg, threshold=500):
        """
        Detects a victim using audio data based on volume threshold.

        :param audio_msg: AudioData message.
        :type audio_msg: audio_common_msgs.msg.AudioData
        :param threshold: Volume threshold for detection.
        :type threshold: int
        :return: True if volume exceeds threshold, otherwise False.
        :rtype: bool
        """
        try:
            # Convert audio data from ROS AudioData message (uint8 array) to bytes
            audio_bytes = bytes(audio_msg.data)

            # Try interpreting the bytes as signed 16-bit PCM audio
            audio_np = np.frombuffer(audio_bytes, dtype=np.int16)

            # Compute the mean absolute amplitude (volume indicator)
            volume = np.mean(np.abs(audio_np))

            # Debug: Log or save volume level (optional)
            # rospy.loginfo(f"Current volume level: {volume}")

            # Return True if the volume exceeds the threshold
            return volume > threshold

        except Exception as e:
            # Log any exception that occurs during processing
            rospy.logerr(f"Audio volume detection failed: {e}")
            return False

    def report_victim(self):
        """
        Publishes a victim alert and a dummy location.
        """
        # Publish a victim alert message
        self.victim_alert_pub.publish("Victim detected!")
        rospy.loginfo("Published victim alert!")

        # Check if depth image is available
        if self.latest_depth_image is None:
            return
        
        # Get the depth value at the center of the image
        center_x = self.latest_depth_image.shape[1] // 2
        center_y = self.latest_depth_image.shape[0] // 2
        depth_value = self.latest_depth_image[center_y, center_x]

        # Use a default depth value if the center depth is invalid
        if depth_value <= 0:
            depth_value = 1000  # Default to 1 meter

        # Publish a dummy victim location
        victim_odom = Odometry()
        victim_odom.header = Header(stamp=rospy.Time.now(), frame_id="xtion_depth_optical_frame")
        victim_odom.pose.pose.position.z = depth_value / 1000.0  # Convert mm to meters
        self.victim_loc_pub.publish(victim_odom)
        rospy.loginfo("Published victim location!")

if __name__ == "__main__":
    """
    Initializes the ROS node and starts the victim detection process.
    """
    rospy.init_node("victim_detector")
    detector = VictimDetector()
    rospy.spin()

