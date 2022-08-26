#! /usr/bin/env python3

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from tf import transformations
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform

position_ = Point()
desired_position_ = Point()
transforms_ = FiducialTransform()
camera_name = "kinect_ros"
robot_name = rospy.get_param('name')
id = rospy.get_param('id')
# robot_name = rospy.get_param('name')

# callbacks
def clbk_fid(msg):
    global transforms_
    transforms_ = msg.transforms
    # print(camera_pose)
    fid_tranform()
    # if len(transforms_) != 0:
        # print(transforms_)
        # fid_tranform()

def fid_tranform():
    global transforms_, pub_fid
    camera_pose = get_model_pose(camera_name)
    # my_fid_transforms = Point()
    # my_fid_transforms = transforms_[0].transform.translation
    
    # my_fid_transforms = camera_pose + my_fid_transforms
    camera_pose.x = camera_pose.x - 0.2*(id - 1)
    camera_pose.z = 0
    pub_fid.publish(camera_pose)
    # print(camera_pose)
    # print(my_fid_transforms)

def get_model_pose(model_name):
    poll_rate = rospy.Rate(1)
    for i in range(10):
        model_states = rospy.wait_for_message('/gazebo/model_states', ModelStates, 1)
        if model_name in model_states.name:
            model_pose = model_states.pose[model_states.name.index(model_name)].position
            break
        poll_rate.sleep()
    else:
        raise RuntimeError('Failed to get ' + model_name + ' model state')
    return model_pose

def main():
    global transforms_, pub_fid
    
    rospy.init_node('marker_transform')

    sub_fid = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, clbk_fid)
    pub_fid = rospy.Publisher(robot_name + '/camera_pose', Point, queue_size=10)
    
    rospy.spin()

if __name__ == "__main__":
    main()