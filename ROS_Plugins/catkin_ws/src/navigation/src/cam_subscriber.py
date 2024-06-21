#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

LOWER_RED = np.array([10, 10, 80])
UPPER_RED = np.array([255, 255, 130])
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_X_CENTER = IMAGE_WIDTH / 2
IMAGE_Y_CENTER = IMAGE_HEIGHT / 2
SMALL_CONTOUR_AREA = 400.0 # area of red logo next to camera is around 370

class CameraProcessor:
    
    def __init__(self, rate):
        self.bridge = CvBridge()
        rospy.init_node('cam_subscriber', anonymous = True)

        # we don't need queues since we only care about the most recent image
        self.pub = rospy.Publisher('/husky_model/husky/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/husky_model/husky/camera", Image, self.callback, queue_size=1)
        self.rate = rospy.Rate(rate)


    def publish(self, message):
        self.pub.publish(message)
        rospy.loginfo(message)
        self.rate.sleep()

    def callback(self, data):
        try:
            # we need to convert the image to correct color format
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8') 
            
        except CvBridgeError as e:
            print(e)
            
        cv2.imshow("Image window", cv_image)
        cv2.imwrite("./image.jpg", cv_image)

        # detect red
        # process image
        # get velocity
        
        linear = Vector3(1.0, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, 0.0)
        velocity = Twist(linear, angular)
        rospy.loginfo(velocity)
        self.publish(velocity)

    def process_image(self, image):
        pass

    def is_red_light(self, image):
                     
        blurred = cv2.GaussianBlur(image, (5, 5), 0) # reduces noise and makes detection more accurate
        mask_red = cv2.inRange(blurred, LOWER_RED, UPPER_RED)
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        # remove camera logo 
         
        im = np.copy(image)
        c = max(contours, key = cv2.contourArea) # largest contour 
        if cv2.contourArea(c) < SMALL_CONTOUR_AREA:
            return False
        
    def find_x_shift(self, image):

        # cv2.drawContours(im, c, -1, (0, 255, 0), 1)
        M = cv2.moments(c)
        if M['m00'] != 0: # finds center of contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        print(f"x: {cx} y: {cy}")
        return cx - IMAGE_X_CENTER # returns the difference between the center of the image and the center of the red light

    def detect_red(self,image):
        pass
    
    def get_velocity(self, x_shift):
        pass

def process(rate):
    cp = CameraProcessor(rate)
    rospy.spin()

if __name__ == '__main__':
    process(rate = 1)