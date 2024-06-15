#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id () + f"Received {data.data}")

def listener():
    rospy.init_node('cam_subscriber', anonymous = True)
    rospy.Subscriber("/husky_model/husky/camera", String, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()