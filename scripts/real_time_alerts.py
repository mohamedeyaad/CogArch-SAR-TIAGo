#!/usr/bin/env python3

import rospy
import threading
import tkinter as tk
from tiago_sar_cogarch.msg import TriageReport
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

class RealTimeReport:
    """
    A node that listens to triage reports, risk alerts, victim locations,
    victim alerts, robot location, and mission success notifications.
    """

    def __init__(self):
        """
        Initializes the real-time report node, subscribes to various topics
        to receive updates on triage reports, risk alerts, victim location,
        robot location, and mission status.
        """
        # Initialize ROS node
        rospy.init_node('real_time_report')

        # Subscribe to ROS topics
        rospy.Subscriber("/triage_status", TriageReport, self.triage_callback)
        rospy.Subscriber("/victim_alert", String, self.victim_alert_callback)
        rospy.Subscriber("/victim_location", Odometry, self.victim_location_callback)
        rospy.Subscriber("/risk_alert", String, self.risk_alert_callback)
        rospy.Subscriber("/slam_3d/current_pose", PoseStamped, self.robot_location_callback)
        rospy.Subscriber("/mission_report", String, self.mission_report_callback)

        # Publisher for real-time updates
        self.update_pub = rospy.Publisher("/real_time_updates", String, queue_size=10)

        # GUI elements
        self.root = tk.Tk()
        self.root.title("Real-Time Report")

        # Labels for displaying information
        self.triage_label = tk.Label(self.root, text="Triage Report: Waiting...", width=50, anchor='w')
        self.triage_label.pack()

        self.victim_alert_label = tk.Label(self.root, text="Victim Alert: Waiting...", width=50, anchor='w')
        self.victim_alert_label.pack()

        self.robot_location_label = tk.Label(self.root, text="Robot Location: Waiting...", width=50, anchor='w')
        self.robot_location_label.pack()

        self.mission_report_label = tk.Label(self.root, text="Mission Report: Waiting...", width=50, anchor='w')
        self.mission_report_label.pack()

        self.risk_alert_label = tk.Label(self.root, text="Risk Alert: Waiting...", width=50, anchor='w')
        self.risk_alert_label.pack()

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

        # ROS Node running in a separate thread
        self.ros_thread = threading.Thread(target=self.run_ros)
        self.ros_thread.start()

        # Tkinter loop running in the main thread
        self.root.mainloop()

    def run_ros(self):
        """
        Run the ROS spin function in a separate thread.
        """
        rospy.spin()

    def triage_callback(self, msg):
        """
        Callback for receiving triage reports. This processes each report
        and generates an update message to publish.
        """
        rospy.loginfo("[RealTimeReport] Received triage report.")
        
        # Process the triage report and update the GUI
        update_msg = self.process_report(msg)
        self.update_gui("Triage Report", update_msg)
        
        # Publish the update
        self.update_pub.publish(update_msg)

    def process_report(self, report):
        """
        Processes the triage report and generates a string update.
        """
        conscious_status = report.consciousness.capitalize()
        responsive_status = report.responsive.capitalize()
        injury_status = ", ".join(report.injuries) if report.injuries else "No injuries detected"
        priority = report.priority

        update_str = (f"Victim Status:\n"
                      f"Conscious: {conscious_status}\n"
                      f"Responsive: {responsive_status}\n"
                      f"Injuries: {injury_status}\n"
                      f"Priority: {priority}\n"
                      f"Location: ({report.location.position.x}, {report.location.position.y}, {report.location.position.z})")

        return update_str

    def victim_alert_callback(self, msg):
        """
        Callback for receiving victim alert messages. Processes the alert
        and updates the GUI.
        """
        rospy.loginfo(f"[RealTimeReport] Received victim alert: {msg.data}")
        self.update_gui("Victim Alert", msg.data)

    def victim_location_callback(self, msg):
        """
        Callback for receiving victim location data. Processes the location
        and updates the GUI.
        """
        location = f"({msg.pose.pose.position.x}, {msg.pose.pose.position.y}, {msg.pose.pose.position.z})"
        rospy.loginfo(f"[RealTimeReport] Received victim location: {location}")
        self.update_gui("Victim Location", location)

    def robot_location_callback(self, msg):
        """
        Callback for receiving robot location data (from SLAM). Processes
        the robot's position and updates the GUI.
        """
        location = f"({msg.pose.position.x}, {msg.pose.position.y}, {msg.pose.position.z})"
        rospy.loginfo(f"[RealTimeReport] Received robot location: {location}")
        self.update_gui("Robot Location", location)

    def risk_alert_callback(self, msg):
        """
        Callback for receiving risk alert messages. Processes the alert
        and updates the GUI.
        """
        rospy.loginfo(f"[RealTimeReport] Received risk alert: {msg.data}")
        self.update_gui("Risk Alert", msg.data)

    def mission_report_callback(self, msg):
        """
        Callback for receiving mission success notifications. Processes the
        mission report and updates the GUI.
        """
        rospy.loginfo(f"[RealTimeReport] Mission success: {msg.data}")
        self.update_gui("Mission Report", msg.data)

    def update_gui(self, label, message):
        """
        Updates the appropriate label in the Tkinter GUI with the new message.
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
        Called when the GUI window is closed.
        """
        rospy.signal_shutdown("GUI closed.")
        self.root.quit()


if __name__ == "__main__":
    # Create an instance of the RealTimeReport class
    RealTimeReport()

