#!/usr/bin/env python
# -*- coding: utf-8 -*

"""

arm_whereisthis.py

author: Jin Wei
last edit: 2020/3/23

subscribed topic: /navi/markpoint
structure name: markpoint
structure content: 
    id: uint8, from 1 to n, refering to the i th object
    side: uint8, 0 for left side, 1 for right side, 2 for forward

"""

import rospy
import thread
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from whereisthis_control.msg import whereisthis
from whereisthis_control.msg import markpoint

class arm_whereisthis:

    def __init__(self):
 
        # init the node
        rospy.init_node('arm')

        # publish command message to joints/servos of arm
        self.joint1 = rospy.Publisher('/waist_controller/command',Float64)    # positive angle: counterclockwise 
        self.joint2 = rospy.Publisher('/shoulder_controller/command',Float64) # positive angle: counterclockwise (look from the left)
        self.joint3 = rospy.Publisher('/elbow_controller/command',Float64)    # positive angle: same as above
        self.joint4 = rospy.Publisher('/wrist_controller/command',Float64)    # positive angle: same as above
        self.joint5 = rospy.Publisher('/hand_controller/command',Float64)     # positive angle: close the gripper
        self.pos1 = Float64()
        self.pos2 = Float64()
        self.pos3 = Float64()
        self.pos4 = Float64()
        self.pos5 = Float64()

        rospy.Subscriber('/navi/markpoint', markpoint, self.Callback)

        rospy.spin()    

    def Callback(self, msg):
    # arm starts to point after NAVI part found the robot has reached some markpoint's range

        if msg.side == 0:
            # Manipulation...
            # Pointing at left side
            rospy.loginfo('Pointing at left side...')

            self.pos1 = 1.565
            self.pos2 = -0.628
            self.pos3 = -0.593
            self.pos4 = -0.349
            self.pos5 = 0.0
            self.joint1.publish(self.pos1)
            self.joint2.publish(self.pos2)
            self.joint3.publish(self.pos3)
            self.joint4.publish(self.pos4)
            self.joint5.publish(self.pos5)
            rospy.sleep(10)        

            # wait for 5 sec and the arm goes back to initial pose
            self.back2init()

        elif msg.side == 1:
            # Manipulation...
            # Pointing at right side
            rospy.loginfo('Pointing at right side...')

            self.pos1 = -1.565
            self.pos2 = -0.628
            self.pos3 = -0.593
            self.pos4 = -0.349
            self.pos5 = 0.0
            self.joint1.publish(self.pos1)
            self.joint2.publish(self.pos2)
            self.joint3.publish(self.pos3)
            self.joint4.publish(self.pos4)
            self.joint5.publish(self.pos5)
            rospy.sleep(10)        

            # wait for 5 sec and the arm goes back to initial pose
            self.back2init()

        elif msg.data == 2:
            # Manipulation ...
            # Pointing forward
            rospy.loginfo('Pointing forward...')		

	    self.pos1 = 0.0
	    self.pos2 = -0.628
	    self.pos3 = -0.593
	    self.pos4 = -0.349
	    self.pos5 = 0.0
	    self.joint1.publish(self.pos1)
	    self.joint2.publish(self.pos2)
	    self.joint3.publish(self.pos3)
	    self.joint4.publish(self.pos4)
	    self.joint5.publish(self.pos5)
	    rospy.sleep(10)

            # wait for 5 sec and the arm goes back to initial pose
            self.back2init()

    def back2init(self):
        # Robot arm back to initial pose

        # Initial gesture of robot arm
        # This is the natural pose for the robot arm
        rospy.sleep(10) # wait for SPEECH part to finish which lasts for about 5 seconds
        rospy.loginfo('initial gesture of robot arm: back2init')

        self.pos1 = 0.0
        self.pos2 = 2.102
        self.pos3 = -2.439
        self.pos4 = -1.294
        self.pos5 = 0.0
        self.joint1.publish(self.pos1)
        self.joint2.publish(self.pos2)
        self.joint3.publish(self.pos3)
        self.joint4.publish(self.pos4)
        self.joint5.publish(self.pos5)
        

if __name__=="__main__":
    arm_whereisthis()
