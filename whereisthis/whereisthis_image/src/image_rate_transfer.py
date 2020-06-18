#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
from sensor_msgs.msg import Image

class RateTrans():
    
    def __init__(self):
        rospy.init_node('image_rate_transfer')
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.rateTransfer, queue_size = 1)
        self.publisher=rospy.Publisher('/image_rate_transfer/image_raw', Image, queue_size = 1)
        self.time = time.time()

    def rateTransfer(self, msg):
        t = time.time()
        if t - self.time > 5:
            self.publisher.publish(msg)
            self.time = t


if __name__ == '__main__':
    RateTrans()
    rospy.spin()
