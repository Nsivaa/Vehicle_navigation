#!/usr/bin/python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Constants
BGR_LOWER_RED = np.array([10, 10, 80]) # lower bound for red masking
BGR_UPPER_RED = np.array([255, 255, 130]) # upper bound for red masking
HSV_LOWER_RED = np.array([155,25,0])
HSV_UPPER_RED = np.array([179,255,255])
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240
IMAGE_X_CENTER = int(IMAGE_WIDTH / 2)
IMAGE_Y_CENTER = int(IMAGE_HEIGHT / 2)
SMALL_CONTOUR_AREA = 100.0 # area of red logo next to camera is around 370
X_SHIFT_EPSILON = 20 # error tolerance for x_shift

class CameraProcessor:
    
    def __init__(self, rate, debug):
        self.bridge = CvBridge()
        rospy.init_node('cam_subscriber', anonymous = True)

        # we don't need queues since we only care about the most recent image
        self.pub = rospy.Publisher('/husky_model/husky/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/husky_model/husky/camera", Image, self.callback, queue_size=1)
        self.rate = rospy.Rate(rate)
        self.i = 0
        self.save_every = 10
        self.debug = debug
        self.lower_red_1 = np.array([0, 100, 100])
        self.upper_red_1 = np.array([10, 255, 255])
        self.lower_red_2 = np.array([160, 100, 100])
        self.upper_red_2 = np.array([180, 255, 255])

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

        velocity = self.get_velocity(cv_image)
        self.publish(velocity)

    def get_velocity(self, image):
        shift = self.find_x_shift(image)
        print(f"shift: {shift}")

        # ADD CONTROL ON STUCK?
        if shift == False:
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 1)) # turn left
        if abs(shift) < X_SHIFT_EPSILON:
            return Twist(Vector3(1, 0, 0), Vector3(0, 0, 0)) # move forward
        if shift > 0:
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, -1)) # go right
        if shift < 0:
            return Twist(Vector3(0, 0, 0), Vector3(0, 0, 1)) # go left
        
    def find_x_shift(self, image):

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create masks for the red color
        mask1 = cv2.inRange(hsv_image, self.lower_red_1, self.upper_red_1)
        mask2 = cv2.inRange(hsv_image, self.lower_red_2, self.upper_red_2)
        mask_red = cv2.bitwise_or(mask1, mask2)

        contours, _ = cv2.findContours(mask_red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        try:
            c = max(contours, key = cv2.contourArea) # largest contour 
            area = cv2.contourArea(c)
            if area < SMALL_CONTOUR_AREA:
                return False
        except ValueError:
            return False

        M = cv2.moments(c)
        if M['m00'] != 0: # finds center of contour
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
        shift = cx - IMAGE_X_CENTER
        if self.debug:
            image = cv2.circle(image, (cx, cy), radius=1, color=(200, 200, 200), thickness=-1)
            image = cv2.circle(image, (IMAGE_X_CENTER, IMAGE_Y_CENTER), radius=1, color=(255, 200, 0), thickness=-1)
            image = add_text(image, f"area: {area}, shift: {shift}")
            if self.i % self.save_every == 0:
                cv2.drawContours(image, c, -1, (0, 255, 0), 1)
                filename = str(rospy.get_time()) + ".jpg"
                path = "../images/" + filename
                cv2.imwrite(path, image)
                self.i += 1

        return shift # returns the difference between the center of the image and the center of the red light

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
    process(rate = 20, debug = False)