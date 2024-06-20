#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Twist, Vector3

def talker(hz = 10):
    rospy.init_node('vel_publisher', anonymous=True)
    pub = rospy.Publisher('/husky_model/husky/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(hz) # 10hz
    while not rospy.is_shutdown():
        linear = Vector3(1.0, 0.0, 0.0)
        angular = Vector3(0.0, 0.0, 1.0)
        velocity = Twist(linear, angular)        
        rospy.loginfo(velocity)
        pub.publish(velocity)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass