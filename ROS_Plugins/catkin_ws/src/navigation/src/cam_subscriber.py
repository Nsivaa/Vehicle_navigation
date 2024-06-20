#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class CameraProcessor:
    
    def __init__(self):
        self.bridge = CvBridge()
        rospy.init_node('cam_subscriber', anonymous = True)

        self.sub = rospy.Subscriber("/husky_model/husky/camera", Image, self.callback, queue_size=1)

    def callback(self, data):
        # rospy.loginfo(rospy.get_caller_id() + f"Received {data.data}")
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except CvBridgeError as e:
            print(e)
        cv2.imshow("Image window", cv_image)
        cv2.waitKey(3)

def listener():
    cp = CameraProcessor()
    rospy.spin()

if __name__ == '__main__':
    listener()