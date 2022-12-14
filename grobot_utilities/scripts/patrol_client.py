#! /usr/bin/env python3

import sys
import rospy
from actionlib import SimpleActionClient
from grobot_utilities.msg import PatrolAction, PatrolGoal


def main():
    rospy.init_node('patrol_client')

    rospy.sleep(2)
    action_client = SimpleActionClient('patrol_server', PatrolAction)
    action_client.wait_for_server()

    goal = PatrolGoal()
    assert len(sys.argv) >= 3
    goal.radius = float(sys.argv[1])
    goal.clockwise = bool(sys.argv[2].lower() == 'true')
    goal.num_patrols = 0
    action_client.send_goal(goal)
    rospy.loginfo('PatrolClient: Sent goal to %s', rospy.get_namespace()[:-1])
    action_client.wait_for_result()


if __name__ == '__main__':
    main()
