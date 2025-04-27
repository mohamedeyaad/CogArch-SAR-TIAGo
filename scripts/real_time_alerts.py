#!/usr/bin/env python3

"""
.. module:: real_time_report
    :platform: ROS
    :synopsis: ROS node with GUI for real-time mission monitoring and manual control

.. moduleauthor:: Your Name <your.email@example.com>

This node provides a graphical interface for monitoring search and rescue mission status,
including victim triage, robot location, risk alerts, and mission reports. It also enables
manual control through ROS topics.
"""

import rospy
import threading
import tkinter as tk
from tiago_sar_cogarch.msg import TriageReport, RiskReport
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tkinter import font  # Import font module for custom fonts

class RealTimeReport:
    """
    Main GUI application class for real-time mission monitoring and control.
    
    Attributes:
        manual_request_pub (rospy.Publisher): Publisher for manual intervention requests
        root (tk.Tk): Main GUI window instance
        ros_thread (threading.Thread): Separate thread for ROS operations
        Various GUI widgets (labels, buttons, text fields)
    """
    
    def __init__(self):
        """
        Initializes ROS node, GUI components, and ROS subscribers/publishers.
        Configures multi-threaded architecture for simultaneous GUI and ROS operations.
        """
        # Initialize ROS node
        rospy.init_node('real_time_report')

        # Publisher for manual requests
        self.manual_request_pub = rospy.Publisher("/manual_request", String, queue_size=10)

        # GUI elements
        self.root = tk.Tk()
        self.root.title("Real-Time Report")
        self.root.geometry("400x750")  # Set default window size

        # Custom font
        header_font = font.Font(family="Helvetica", size=12, weight="bold")
        label_font = font.Font(family="Arial", size=10)

        # Frames for better organization
        top_frame = tk.Frame(self.root, padx=10, pady=10)
        top_frame.pack(fill=tk.BOTH, expand=True)

        middle_frame = tk.Frame(self.root, padx=10, pady=10)
        middle_frame.pack(fill=tk.BOTH, expand=True)

        bottom_frame = tk.Frame(self.root, padx=10, pady=10)
        bottom_frame.pack(fill=tk.BOTH, expand=True)

        # Labels for displaying information
        self.triage_label = tk.Label(top_frame, text="Triage Report: Waiting...", width=50, anchor='w', font=label_font)
        self.triage_label.pack(pady=5)

        self.victim_alert_label = tk.Label(top_frame, text="Victim Alert: Waiting...", width=50, anchor='w', font=label_font)
        self.victim_alert_label.pack(pady=5)

        self.robot_location_label = tk.Label(top_frame, text="Robot Location: Waiting...", width=50, anchor='w', font=label_font)
        self.robot_location_label.pack(pady=5)

        self.mission_report_label = tk.Label(top_frame, text="Mission Report: Waiting...", width=50, anchor='w', font=label_font)
        self.mission_report_label.pack(pady=5)

        self.risk_alert_label = tk.Label(top_frame, text="Risk Alert: Waiting...", width=50, anchor='w', font=label_font)
        self.risk_alert_label.pack(pady=5)

        # Buttons for manual actions
        self.refresh_button = tk.Button(middle_frame, text="Refresh Data", command=self.refresh_data, font=header_font, bg="#4CAF50", fg="white")
        self.refresh_button.pack(pady=10, anchor='center')  # Center-align with padding

        self.test_button = tk.Button(middle_frame, text="Send Test Update", command=self.send_test_update, font=header_font, bg="#2196F3", fg="white")
        self.test_button.pack(pady=10, anchor='center')  # Center-align with padding

        # Input field for manual input
        self.input_label = tk.Label(middle_frame, text="Manual Request:", font=label_font)
        self.input_label.pack(pady=5, anchor='center')  # Center-align with padding
        self.input_field = tk.Entry(middle_frame, width=50)
        self.input_field.pack(pady=5, anchor='center')  # Center-align with padding

        # Log window
        self.log_label = tk.Label(bottom_frame, text="Logs:", font=header_font)
        self.log_label.pack(pady=5)
        self.log_window = tk.Text(bottom_frame, height=10, width=70, state='disabled', bg="#f0f0f0")
        self.log_window.pack(pady=5)

        # Status bar
        self.status_bar = tk.Label(self.root, text="Status: Ready", bd=1, relief=tk.SUNKEN, anchor='w', font=label_font, bg="#e0e0e0")
        self.status_bar.pack(side=tk.BOTTOM, fill=tk.X)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ROS Node running in a separate thread
        self.ros_thread = threading.Thread(target=self.run_ros)
        self.ros_thread.start()

        # Move ROS subscribers here to ensure GUI elements are initialized
        rospy.Subscriber("/triage_status", TriageReport, self.triage_callback)
        rospy.Subscriber("/victim_alert", String, self.victim_alert_callback)
        rospy.Subscriber("/victim_location", Odometry, self.victim_location_callback)
        rospy.Subscriber("/risk_alert", RiskReport, self.risk_alert_callback)
        rospy.Subscriber("/slam_3d/current_pose", PoseStamped, self.robot_location_callback)
        rospy.Subscriber("/mission_report", String, self.mission_report_callback)

        # Tkinter loop running in the main thread
        self.root.mainloop()

    def refresh_data(self):
        """
        Handles manual data refresh request from GUI.
        Updates status bar and logs the action.
        """
        self.log_message("Refreshing data...")
        # Add logic to refresh data if needed
        self.status_bar.config(text="Status: Data refreshed")

    def send_test_update(self):
        """
        Sends manual request from input field to ROS network.
        Validates input and publishes to /manual_request topic.
        """
        manual_request = self.input_field.get()
        if manual_request:
            self.log_message(f"Sending manual request: {manual_request}")
            self.manual_request_pub.publish(manual_request)
            self.status_bar.config(text="Status: Manual request sent")
        else:
            self.log_message("No input provided for manual request.")

    def log_message(self, message):
        """
        Manages log window updates with timestamped messages.
        
        :param message: Message text to display in log
        :type message: str
        """
        self.log_window.config(state='normal')
        self.log_window.insert(tk.END, f"{message}\n")
        self.log_window.config(state='disabled')
        self.log_window.see(tk.END)

    def run_ros(self):
        """
        Maintains ROS node functionality in separate thread.
        Handles ROS message processing without blocking GUI.
        """
        rospy.spin()

    def triage_callback(self, msg):
        """
        Processes incoming triage reports from /triage_status topic.
        
        :param msg: Triage report message
        :type msg: tiago_sar_cogarch.msg.TriageReport
        """
        rospy.loginfo("[RealTimeReport] Received triage report.")
        update_msg = self.process_report(msg)
        self.update_gui("Triage Report", update_msg)

    def process_report(self, report):
        """
        Formats TriageReport message for GUI display.
        
        :param report: Raw triage report data
        :type report: tiago_sar_cogarch.msg.TriageReport
        :return: Formatted string for GUI display
        :rtype: str
        """
        conscious_status = report.consciousness.capitalize()
        responsive_status = report.responsive.capitalize()
        injury_status = ", ".join(report.injuries) if report.injuries else "No injuries detected"
        priority = report.priority

        # Format location to 2 decimal places
        location = (f"({report.location.position.x:.2f}, "
                    f"{report.location.position.y:.2f}, "
                    f"{report.location.position.z:.2f})")

        update_str = (f"Victim Status:\n"
                      f"Conscious: {conscious_status}\n"
                      f"Responsive: {responsive_status}\n"
                      f"Injuries: {injury_status}\n"
                      f"Priority: {priority}\n"
                      f"Location: {location}")

        return update_str

    def victim_alert_callback(self, msg):
        """
        Handles victim alerts from /victim_alert topic.
        
        :param msg: Alert message containing victim information
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"[RealTimeReport] Received victim alert: {msg.data}")
        self.update_gui("Victim Alert", msg.data)

    def victim_location_callback(self, msg):
        """
        Processes victim location updates from /victim_location topic.
        
        :param msg: Odometry message with victim coordinates
        :type msg: nav_msgs.msg.Odometry
        """
        # Format location to 2 decimal places
        location = (f"({msg.pose.pose.position.x:.2f}, "
                    f"{msg.pose.pose.position.y:.2f}, "
                    f"{msg.pose.pose.position.z:.2f})")
        rospy.loginfo(f"[RealTimeReport] Received victim location: {location}")
        self.update_gui("Victim Location", location)

    def robot_location_callback(self, msg):
        """
        Updates robot position from /slam_3d/current_pose topic.
        
        :param msg: PoseStamped message with current robot location
        :type msg: geometry_msgs.msg.PoseStamped
        """
        # Format location to 2 decimal places
        location = (f"({msg.pose.position.x:.2f}, "
                    f"{msg.pose.position.y:.2f}, "
                    f"{msg.pose.position.z:.2f})")
        rospy.loginfo(f"[RealTimeReport] Received robot location: {location}")
        self.update_gui("Robot Location", location)

    def risk_alert_callback(self, msg):
        """
        Processes structural risk alerts from /risk_alert topic.
        
        :param msg: RiskReport message with structural integrity data
        :type msg: tiago_sar_cogarch.msg.RiskReport
        """
        rospy.loginfo(f"[RealTimeReport] Received risk alert: Status={msg.status}, "
                      f"Cracks={msg.crack_count}, Wall Anomalies={msg.wall_anomalies}, "
                      f"Hollow Spaces={msg.hollow_spaces}, Force Magnitude={msg.force_magnitude:.2f}, "
                      f"Risk Score={msg.risk_score:.2f}")

        # Format the risk alert message for display
        alert_message = (f"Risk Status: {msg.status}\n"
                         f"Cracks: {msg.crack_count}\n"
                         f"Wall Anomalies: {msg.wall_anomalies}\n"
                         f"Hollow Spaces: {msg.hollow_spaces}\n"
                         f"Force Magnitude: {msg.force_magnitude:.2f} N\n"
                         f"Risk Score: {msg.risk_score:.2f}")

        self.update_gui("Risk Alert", alert_message)

    def mission_report_callback(self, msg):
        """
        Handles mission completion notifications from /mission_report topic.
        
        :param msg: String message with mission status
        :type msg: std_msgs.msg.String
        """
        rospy.loginfo(f"[RealTimeReport] Mission success: {msg.data}")
        self.update_gui("Mission Report", msg.data)

    def update_gui(self, label, message):
        """
        Updates specified GUI element with new information.
        
        :param label: Which GUI label to update
        :type label: str
        :param message: Content to display in the label
        :type message: str
        """
        if label == "Triage Report":
            self.triage_label.config(text=f"Triage Report: {message}")
        elif label == "Victim Alert":
            self.victim_alert_label.config(text=f"Victim Alert: {message}")
        elif label == "Robot Location":
            self.robot_location_label.config(text=f"Robot Location: {message}")
        elif label == "Mission Report":
            self.mission_report_label.config(text=f"Mission Report: {message}")
        elif label == "Risk Alert":
            self.risk_alert_label.config(text=f"Risk Alert: {message}")

    def on_close(self):
        """
        Handles clean shutdown procedures for both ROS and GUI.
        Ensures proper resource cleanup on window closure.
        """
        rospy.signal_shutdown("GUI closed.")
        self.root.quit()


if __name__ == "__main__":
    # Create an instance of the RealTimeReport class
    RealTimeReport()
