#!/usr/bin/env python3

from math import pi

import angles
import numpy
import rospy
import actionlib
import PyKDL
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from grobot_utilities.msg import PatrolAction, PatrolFeedback, PatrolResult


class PatrolServer(object):
    def __init__(self, ):
        self._action_server = actionlib.SimpleActionServer(rospy.get_name(), PatrolAction,
                                                           execute_cb=self.execute_cb, auto_start=False)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.get_odom)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self._current_yaw = 0
        self._action_server.start()
        rospy.loginfo('PatrolServer: Initialized')

    def get_odom(self, odom):
        q = odom.pose.pose.orientation
        self._current_yaw = PyKDL.Rotation.Quaternion(q.x, q.y, q.z, q.w).GetRPY()[2]

    def run_patrol(self, initial_yaw, twist):
        target_yaw = initial_yaw + pi
        half_patrol = False
        update_rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if numpy.isclose(angles.shortest_angular_distance(self._current_yaw, target_yaw), 0.0, atol=0.0174533):
                if half_patrol:
                    break
                target_yaw = target_yaw + pi
                half_patrol = True

            self.cmd_vel_pub.publish(twist)
            update_rate.sleep()

        self.cmd_vel_pub.publish(Twist())

    def execute_cb(self, goal):
        twist = Twist()
        twist.linear.x = 0.1
        sign = -1 if goal.clockwise else 1
        twist.angular.z = sign * twist.linear.x / goal.radius

        initial_yaw = self._current_yaw
        current_patrol = 0
        patrols_done = False
        while not patrols_done:
            try:
                self.run_patrol(initial_yaw, twist)
                current_patrol = current_patrol + 1
                self._action_server.publish_feedback(PatrolFeedback(completed_patrols=current_patrol))
                if current_patrol == goal.num_patrols:
                    patrols_done = True
            except rospy.ROSInterruptException:
                break

        self._action_server.set_succeeded(PatrolResult(success=True))
        rospy.loginfo('PatrolServer: Goal succeeded')


if __name__ == '__main__':
    rospy.init_node('patrol_server')
    server = PatrolServer()
    rospy.spin()
