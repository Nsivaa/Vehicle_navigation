#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from navigation.msg import img_result
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import sys

# retrieve parameters from the ROS parameter server
RATE = rospy.get_param("navigation/rate")
DEBUG_FOLDER = rospy.get_param("navigation/debug_folder")
RAW_IMAGES_FOLDER = DEBUG_FOLDER + rospy.get_param("navigation/raw_images_folder")
CONTOUR_IMAGES_FOLDER = DEBUG_FOLDER + rospy.get_param("navigation/contour_images_folder")
HSV_LOWER_RED_1 = rospy.get_param("navigation/hsv_lower_red_1")
HSV_UPPER_RED_1 = rospy.get_param("navigation/hsv_upper_red_1")
HSV_LOWER_RED_2 = rospy.get_param("navigation/hsv_lower_red_2")
HSV_UPPER_RED_2 = rospy.get_param("navigation/hsv_upper_red_2")
IMAGE_WIDTH = rospy.get_param("navigation/image_width")
IMAGE_HEIGHT = rospy.get_param("navigation/image_height")
MIN_CONTOUR_AREA = rospy.get_param("navigation/min_contour_area") # area of red logo next to camera is around 80
IMAGE_X_CENTER = int(IMAGE_WIDTH / 2)
IMAGE_Y_CENTER = int(IMAGE_HEIGHT / 2)

class CameraProcessor:
    
    def __init__(self, rate, debug):
        self.bridge = CvBridge()
        rospy.init_node('cam_subscriber', anonymous = True)

        # we don't need queues since we only care about the most recent data
        self.pub = rospy.Publisher('/img_result', img_result, queue_size=1)
        self.sub = rospy.Subscriber("/husky_model/husky/camera", Image, self.callback, queue_size=1)
        self.rate = rospy.Rate(rate)
        self.debug = debug
        self.i = 0
        self.save_every = 10
        self.contour_path = CONTOUR_IMAGES_FOLDER
        self.raw_path = RAW_IMAGES_FOLDER

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

        red, shift = self.find_x_shift(cv_image)
        message = img_result(red, shift)
        self.publish(message)

    def find_x_shift(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red_detected = "no"
        shift = 0
        # Create masks for the red color
        mask1 = cv2.inRange(hsv_image, np.array(HSV_LOWER_RED_1), np.array(HSV_UPPER_RED_1))
        mask2 = cv2.inRange(hsv_image, np.array(HSV_LOWER_RED_2),np.array( HSV_UPPER_RED_2))
        mask_red = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        try:
            c = max(contours, key = cv2.contourArea) # largest contour 
            area = cv2.contourArea(c)
            if area < MIN_CONTOUR_AREA:
                return red_detected, shift
            else:
                red_detected = "yes"

        except ValueError:
            return red_detected, shift

        M = cv2.moments(c)
        if M['m00'] != 0: # finds center of contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        shift = cx - IMAGE_X_CENTER
        if self.debug:
            image = cv2.circle(image, (cx, cy), radius=2, color=(200, 200, 200), thickness=-1)
            image = cv2.circle(image, (IMAGE_X_CENTER, IMAGE_Y_CENTER), radius=2, color=(255, 200, 0), thickness=-1)
            image = add_text(image, f"area: {area}, shift: {shift}")
            self.i += 1

            if self.i % self.save_every == 0:
                filename = self.raw_path + str(rospy.get_time()) + ".jpg"
                cv2.imwrite(filename, image)
                cv2.drawContours(image, c, -1, (0, 255, 0), 1)
                filename = self.contour_path + str(rospy.get_time()) + ".jpg"
                cv2.imwrite(filename, image)

        return red_detected, shift # distance between the center of the image and the center of the red light

def process(rate, debug):
    cp = CameraProcessor(rate, debug)
    rospy.spin()

def add_text(img, text):
    BLACK = (255,255,255)
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_size = 0.5
    font_color = BLACK
    font_thickness = 2
    x,y = 10,10
    img = cv2.putText(img, text, (x,y), font, font_size, font_color, font_thickness, cv2.LINE_AA)
    return img

if __name__ == '__main__':

    rate = rospy.get_param("navigation/rate")
    debug = rospy.get_param("navigation/debug")
    if debug != True and debug != False:
        rospy.logwarn("Invalid argument. Running without debug mode.")
        debug = False
    process(rate = RATE, debug = debug) # rate chosen to match the camera's frame rate