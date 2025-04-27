#!/usr/bin/env python3

"""
.. module:: path_planner_node
    :platform: ROS
    :synopsis: Python node for generating navigation paths between current pose and goal

.. moduleauthor:: Your Name <your.email@example.com>

This node creates simple straight-line paths between the robot's current position and requested goals.
"""

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf

class PathPlannerNode:
    """
    Path planning node that generates navigation paths between current position and goals.
    
    Attributes:
        current_pose (PoseStamped): Stores latest known robot position
        goal_pose (PoseStamped): Stores target destination coordinates
        path_pub (rospy.Publisher): Publisher for generated navigation paths
    """
    
    def __init__(self):
        """
        Initializes node, subscribers, and publisher. Sets up ROS infrastructure.
        """
        rospy.init_node('path_planner_node')

        # Subscribers
        rospy.Subscriber('/slam_3d/current_pose', PoseStamped, self.current_pose_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_callback)

        # Publisher
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        self.current_pose = None
        self.goal_pose = None

    def current_pose_callback(self, msg):
        """
        Updates current robot position from SLAM system.
        
        :param msg: Current pose estimate from SLAM
        :type msg: geometry_msgs.msg.PoseStamped
        """
        self.current_pose = msg
        rospy.loginfo_throttle(5, "Received current pose.")

    def goal_pose_callback(self, msg):
        """
        Handles new navigation goals and triggers path generation.
        
        :param msg: Target pose from user or higher-level system
        :type msg: geometry_msgs.msg.PoseStamped
        """
        self.goal_pose = msg
        rospy.loginfo("Received new goal pose.")
        self.generate_dummy_path()

    def generate_dummy_path(self):
        """
        Creates simple straight-line path with interpolated poses.
        Uses linear interpolation between start and goal positions.
        Generates fixed forward-facing orientations using tf quaternions.
        """
        if self.current_pose is None or self.goal_pose is None:
            rospy.logwarn("Missing current or goal pose, can't generate path.")
            return

        # Create a dummy straight-line path from current to goal
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "map"

        start = self.current_pose
        goal = self.goal_pose

        steps = 10
        for i in range(steps + 1):
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = start.pose.position.x + (goal.pose.position.x - start.pose.position.x) * i / steps
            pose.pose.position.y = start.pose.position.y + (goal.pose.position.y - start.pose.position.y) * i / steps
            pose.pose.position.z = 0

            # Dummy orientation (facing forward)
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 0)
            pose.pose.orientation.x = quaternion[0]
            pose.pose.orientation.y = quaternion[1]
            pose.pose.orientation.z = quaternion[2]
            pose.pose.orientation.w = quaternion[3]

            path.poses.append(pose)

        self.path_pub.publish(path)
        rospy.loginfo("Published dummy path.")

    def run(self):
        """
        Main node loop. Maintains ROS connection and processes callbacks.
        """
        rospy.spin()

if __name__ == '__main__':
    """
    Node entry point. Handles ROS initialization and exception catching.
    """
    try:
        node = PathPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
