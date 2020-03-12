#!/usr/bin/env python
#coding:utf-8
import rospy
from std_msgs.msg import String
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    a = 0
    while not rospy.is_shutdown():
        a = a+1
        NameandNumber = "魏祎_1811586_第"+str(a)+"次消息"
        rospy.loginfo(NameandNumber)
        pub.publish(NameandNumber)
        rate.sleep()
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
