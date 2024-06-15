#!/usr/bin/python3
import rospy
from sensor_msgs.msg import Image

def callback(data):
    rospy.loginfo(rospy.get_caller_id () + f"Received {data.data}")

def listener():
    rospy.init_node('cam_subscriber', anonymous = True)
    rospy.Subscriber("/husky_model/husky/camera", Image, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()