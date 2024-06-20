#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraProcessor:
    
    def __init__(self, rate):
        self.bridge = CvBridge()
        rospy.init_node('cam_subscriber', anonymous = True)
        self.pub = rospy.Publisher('/husky_model/husky/cmd_vel', Twist, queue_size=1)
        self.sub = rospy.Subscriber("/husky_model/husky/camera", Image, self.callback)
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
        # process image
        # get velocity
        
        linear = Vector3(1.0, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, 0.0)
        velocity = Twist(linear, angular)
        rospy.loginfo(velocity)
        self.publish(velocity)


def process(rate):
    cp = CameraProcessor(rate)
    rospy.spin()

if __name__ == '__main__':
    process(rate = 1)