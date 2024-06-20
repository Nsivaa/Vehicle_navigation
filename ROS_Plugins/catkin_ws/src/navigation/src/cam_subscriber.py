#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Vector3
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraProcessor:
    
    def __init__(self, rate):
        self.bridge = CvBridge()
        self.pub = rospy.Publisher('/husky_model/husky/cmd_vel', Twist, queue_size=1)

        self.sub = rospy.Subscriber("/husky_model/husky/camera", Image, self.callback, queue_size=1)


        rospy.init_node('cam_subscriber', anonymous = True)

        self.rate = rospy.Rate(rate)
        rospy.spin()


    def publish(self, message):
        self.pub.publish(message)
        rospy.loginfo(message)

        self.rate.sleep()
        rospy.spin()

    def callback(self, data):

        try:
            # passthrough maintains the same encoding of the starting image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough') 
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", cv_image)
        # process image
        # get velocity
        linear = Vector3(1.0, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, 1.0)
        velocity = Twist(linear, angular)
        rospy.loginfo(velocity)
        self.publish(velocity)
        rospy.spin()


def process(rate):
    cp = CameraProcessor(rate)
    rospy.spin()

if __name__ == '__main__':
    process(rate = 1)