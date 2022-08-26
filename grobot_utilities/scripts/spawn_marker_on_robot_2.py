#! /usr/bin/env python3

import time
import rospy
from math import pi
from std_srvs.srv import Empty
from gazebo_ros_link_attacher.srv import Attach
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_from_euler

id = rospy.get_param('id')

def get_model_pose(model_name):
    poll_rate = rospy.Rate(1)
    for i in range(10):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates, 1)
        if model_name in model_states.name:
            model_pose = model_states.pose[model_states.name.index(model_name)]
            break
        poll_rate.sleep()
    else:
        raise RuntimeError('Failed to get ' + model_name + ' model state')
    return model_pose


def main():
    rospy.init_node('spawn_marker_on_robot')
    while not rospy.is_shutdown() and rospy.Time.now() == rospy.Time(0):
        time.sleep(0.1)

    # Initialize service clients
    pause_physics_client = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    unpause_physics_client = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    spawn_sdf_model_client = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
    link_attacher_client = rospy.ServiceProxy('/link_attacher_node/attach', Attach)

    # Initialize model names
    robot_name = "robot_2"
    marker_name = robot_name + '_marker'

    # Compute marker pose based on current robot pose
    marker_pose = get_model_pose(robot_name)
    # marker_pose.position.x -= 0.00001
    marker_pose.position.z = 0.05 + (id * 0.01)
    marker_pose.orientation = Quaternion(*quaternion_from_euler(0, 0, pi))

    # Compute spawn model request
    spawn_request = SpawnModelRequest()
    spawn_request.model_name = marker_name
    spawn_request.model_xml = """
    <sdf version="1.6">
      <world name="default">
        <include>
          <uri>model://marker_2</uri>
        </include>
      </world>
    </sdf>"""
    spawn_request.robot_namespace = robot_name
    spawn_request.initial_pose = marker_pose

    # Spawn and attach marker to the robot
    pause_physics_client()
    spawn_sdf_model_client(spawn_request)
    link_attacher_client(robot_name, 'base_link', marker_name, 'marker')
    unpause_physics_client()


if __name__ == '__main__':
    main()
