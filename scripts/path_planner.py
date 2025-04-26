#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf

class PathPlannerNode:
    def __init__(self):
        rospy.init_node('path_planner_node')

        # Subscribers
        rospy.Subscriber('/slam_3d/current_pose', PoseStamped, self.current_pose_callback)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_pose_callback)

        # Publisher
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1)

        self.current_pose = None
        self.goal_pose = None

    def current_pose_callback(self, msg):
        self.current_pose = msg
        rospy.loginfo_throttle(5, "Received current pose.")

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        rospy.loginfo("Received new goal pose.")
        self.generate_dummy_path()

    def generate_dummy_path(self):
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
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PathPlannerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass

