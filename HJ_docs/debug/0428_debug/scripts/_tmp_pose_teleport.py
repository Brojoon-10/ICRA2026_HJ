#!/usr/bin/env python3
"""Teleport the robot in Gazebo via /initialpose. From user, 2026-04-28.
Run: python3 _tmp_pose_teleport.py
Then publish /initialpose (RViz 2D Pose Estimate) or via rostopic pub.
"""
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState


class PoseTeleport:
    def __init__(self):
        rospy.init_node('pose_teleport')
        self.model_name = rospy.get_param('~model_name', 'unicorn')
        rospy.wait_for_service('/gazebo/set_model_state', timeout=10)
        self.set_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        rospy.Subscriber('/initialpose', PoseWithCovarianceStamped, self.pose_cb)
        rospy.loginfo("pose_teleport ready: publish 2D Pose Estimate in RViz to teleport '%s'", self.model_name)

    def pose_cb(self, msg):
        state = ModelState()
        state.model_name = self.model_name
        state.pose.position.x = msg.pose.pose.position.x
        state.pose.position.y = msg.pose.pose.position.y
        state.pose.position.z = 0.7
        state.pose.orientation = msg.pose.pose.orientation
        state.reference_frame = 'world'
        try:
            self.set_state(state)
            rospy.loginfo("Teleported to (%.2f, %.2f)",
                          msg.pose.pose.position.x, msg.pose.pose.position.y)
        except rospy.ServiceException as e:
            rospy.logerr("Teleport failed: %s", e)


if __name__ == '__main__':
    PoseTeleport()
    rospy.spin()
