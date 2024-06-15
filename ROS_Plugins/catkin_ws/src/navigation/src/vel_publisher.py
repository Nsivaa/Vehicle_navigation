#!/usr/bin/python3
import rospy
from std_msgs.msg import String

def talker(hz = 10):
    rospy.init_node('vel_publisher', anonymous=True)
    pub = rospy.Publisher('/husky_model/husky/cmd_vel', String, queue_size=10)
    rate = rospy.Rate(hz) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass