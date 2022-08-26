#! /usr/bin/env python3

# import ros stuff
import rospy
# import ros message
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf import transformations
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
# import ros service
from std_srvs.srv import *


import math

srv_client_go_to_point_ = None
srv_client_wall_follower_ = None
yaw_ = 0
yaw_error_allowed_ = 5 * (math.pi / 180) # 5 degrees
position_ = Point()
initial_position_ = Point()
initial_position_.x = rospy.get_param('initial_x')
initial_position_.y = rospy.get_param('initial_y')
initial_position_.z = 0
desired_position_ = Point()
# desired_position_.x = rospy.get_param('des_pos_x')
# desired_position_.y = rospy.get_param('des_pos_y')
# desired_position_.z = 0
robot_name = rospy.get_param('name')
regions_ = None
state_desc_ = ['Go to point', 'wall following']
state_ = 0
count_state_time_ = 0 # seconds the robot is in a state
count_loop_ = 0
# 0 - go to point
# 1 - wall following

# callbacks
def clbk_cam(msg):
    global desired_position_
    desired_position_ = msg
    # print(desired_position_)

def clbk_odom(msg):
    global position_, yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def clbk_laser(msg):
    global regions_
    regions_ = {
        'right':  msg.ranges[0],
        'fright': min(msg.ranges[1:2]),
        'front':  min(msg.ranges[3:4]),
        'fleft':  min(msg.ranges[5:6]),
        'left':   msg.ranges[7],
    }

def change_state(state):
    global state_, state_desc_
    global srv_client_wall_follower_, srv_client_go_to_point_
    global count_state_time_
    count_state_time_ = 0
    state_ = state
    print(robot_name)
    log = "state changed: %s" % state_desc_[state]
    rospy.loginfo(log)
    if state_ == 0:
        resp = srv_client_go_to_point_(True)
        resp = srv_client_wall_follower_(False)
    if state_ == 1:
        resp = srv_client_go_to_point_(False)
        resp = srv_client_wall_follower_(True)

def distance_to_line(p0):
    # p0 is the current position
    # p1 and p2 points define the line
    global initial_position_, desired_position_
    p1 = initial_position_
    p2 = desired_position_
    # here goes the equation
    up_eq = math.fabs((p2.y - p1.y) * p0.x - (p2.x - p1.x) * p0.y + (p2.x * p1.y) - (p2.y * p1.x))
    lo_eq = math.sqrt(pow(p2.y - p1.y, 2) + pow(p2.x - p1.x, 2))
    distance = up_eq / lo_eq
    
    return distance
    

def normalize_angle(angle):
    if(math.fabs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def main():
    global regions_, position_, desired_position_, state_, yaw_, yaw_error_allowed_
    global srv_client_go_to_point_, srv_client_wall_follower_
    global count_state_time_, count_loop_
    
    rospy.init_node('go_to_camera')

    scan_topic = robot_name + '/scan'
    odom_topic = robot_name + '/odom_diffdrive'
    cam_topic = robot_name + '/camera_pose'
    
    sub_laser = rospy.Subscriber(scan_topic, LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber(odom_topic, Odometry, clbk_odom)
    sub_cam = rospy.Subscriber(cam_topic, Point, clbk_cam)

    rospy.wait_for_service(robot_name + '/go_to_point_switch')
    rospy.wait_for_service(robot_name + '/wall_follower_switch')
    rospy.wait_for_service('/gazebo/set_model_state')
    
    srv_client_go_to_point_ = rospy.ServiceProxy(robot_name + '/go_to_point_switch', SetBool)
    srv_client_wall_follower_ = rospy.ServiceProxy(robot_name + '/wall_follower_switch', SetBool)
    srv_client_set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    
    # set robot position
    model_state = ModelState()
    model_state.model_name = robot_name
    model_state.pose.position.x = initial_position_.x
    model_state.pose.position.y = initial_position_.y
    resp = srv_client_set_model_state(model_state)
    
    # initialize going to the point
    change_state(0)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        if regions_ == None:
            continue
        
        distance_position_to_line = distance_to_line(position_)
        error = math.sqrt(pow(desired_position_.y - position_.y, 2) + pow(desired_position_.x - position_.x, 2))
        
        if state_ == 0:
            if (regions_['front'] < 0.15 or regions_['fright'] < 0.15 or regions_['fleft'] < 0.15) and error > 0.3:
                change_state(1)
        
        elif state_ == 1:
            if (count_state_time_ > 5 and distance_position_to_line < 0.01) and (regions_['front'] > 0.15 or regions_['fright'] > 0.15 or regions_['fleft'] > 0.15):
                change_state(0)
                
        count_loop_ = count_loop_ + 1
        if count_loop_ == 10:
            count_state_time_ = count_state_time_ + 1
            count_loop_ = 0
            
        # rospy.loginfo("distance to line: [%.2f], position: [%.2f, %.2f]", distance_to_line(position_), position_.x, position_.y)
        rate.sleep()

if __name__ == "__main__":
    main()