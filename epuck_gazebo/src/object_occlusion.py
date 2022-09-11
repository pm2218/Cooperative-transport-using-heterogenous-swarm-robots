#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import numpy as np
import math
import sys
from fiducial_msgs.msg import FiducialTransformArray, FiducialTransform
from gazebo_msgs.msg import ModelStates

bridge = CvBridge()
camera_name = "kinect_ros"

class occlusion(object):

    def get_model_pose(self,model_name):
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

    def clbk_fid(self, msg):
        transforms = msg.transforms
        # print(transforms)
        for i in range(len(transforms)):
            if transforms[i].fiducial_id == 5:
                object_pose = transforms[i].transform.translation
                break
        camera_pose = self.get_model_pose(camera_name)
        print(object_pose)
        # print(camera_pose)
        # radius_real = self.radius * abs(object_pose.x/(320 - self.centre_image.x))
        radius_real = 0.5
        print(radius_real)
        N = 16
        offset =  math.pi - math.atan2(camera_pose.y-object_pose.y,camera_pose.x-object_pose.x)
        print(math.degrees(offset))
        robot_1 = Point()
        robot_2 = Point()
        robot_3 = Point()
        robot_4 = Point()
        robots = {1:robot_1, 2:robot_2, 3:robot_3, 4:robot_4}
        for i in range(-2,2):
            robots[i+3].x = object_pose.x + radius_real*math.cos(2*math.pi*i/N - offset)
            robots[i+3].y = object_pose.y + radius_real*math.sin(2*math.pi*i/N - offset)
            # print(x,y)
        self.pub_fid_1.publish(robots[1])
        self.pub_fid_2.publish(robots[2])
        self.pub_fid_3.publish(robots[3])
        self.pub_fid_4.publish(robots[4])
    
    def image_callback(self,msg):
        print("Received an image!")
        try:
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
        else:
            self.colour_detect(cv2_img)           

    def colour_detect(self,image):

        # https://dev.to/erol/object-detection-with-color-knl
        # https://stackoverflow.com/questions/51531414/opencv-findcontours-just-returning-one-external-contour
        # https://stackoverflow.com/questions/36817133/identifying-the-range-of-a-color-in-hsv-using-opencv
        # RED HSV
        
        redLower = (0,50,70)
        redUpper = (9,255,255)

        # blur
        blurred = cv2.GaussianBlur(image, (11,11), 0)

        # hsv
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

        # mask for red
        mask1 = cv2.inRange(hsv, redLower, redUpper)
        # deleting noises which are in area of mask
        mask1 = cv2.erode(mask1, None, iterations=2)
        mask1 = cv2.dilate(mask1, None, iterations=2)

        contours, _ = cv2.findContours(mask1.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for i, contour in enumerate(contours):
            center = None
            rect = cv2.minAreaRect(contour)

            # box
            box = cv2.boxPoints(rect)
            box = np.int64(box)
            
            # moment
            M = cv2.moments(contour)
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

            # point in center
            cv2.circle(image, center, 2, (255, 0, 255), -1)

            # draw contour
            # cv2.drawContours(image, [box], 0, (0, 255, 255), 2)

        self.centre_image = Point()
        self.centre_image.x = center[0]
        self.centre_image.y = center[1]
        self.centre_image.z = 0
        self.radius = (math.sqrt(math.pow(box[0][0]-box[1][0],2)+math.pow(box[0][1]-box[1][1],2)))/2
        N = 16
        offset =  math.pi - math.atan2(240-center[1],320-center[0])
        for i in range(-2,2):
            x = int(center[0] + self.radius*math.cos(2*math.pi*i/N - offset))
            y = int(center[1] + self.radius*math.sin(2*math.pi*i/N - offset))
            cv2.circle(image, (x,y), 2, (0, 255, 255), -1)
        self.show_image(image)

    def show_image(self, img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)  

    def main(self):
        self.sub = rospy.Subscriber("/fiducial_images", Image, self.image_callback)
        sub_fid = rospy.Subscriber('/fiducial_transforms', FiducialTransformArray, self.clbk_fid)
        self.pub_fid_1 = rospy.Publisher('robot_1/camera_pose', Point, queue_size=20)
        self.pub_fid_2 = rospy.Publisher('robot_2/camera_pose', Point, queue_size=20)
        self.pub_fid_3 = rospy.Publisher('robot_3/camera_pose', Point, queue_size=20)
        self.pub_fid_4 = rospy.Publisher('robot_4/camera_pose', Point, queue_size=20)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('object_occlusion')
    my_node = occlusion()
    my_node.main()