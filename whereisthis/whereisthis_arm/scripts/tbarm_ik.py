#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Jin Wei

Function: a simple method to solve all the joint angle given 3-D coorinate.

Note:
    Input: Cartesian coordinates (x,y,z), describing the relative position from
           the base of turtlebot arm to the expected position of its end 
           effector.
    Output: a vetor that describes how many degrees every angle should rotate.
    Constriction: the pose of end effector should be 45° above horizon.
"""

import rospy
import math
from whereisthis_control.msg import xyz
from std_msgs.msg import Float64

## length of links
L1 = 0.04 # meters
L2 = 0.105
L3 = L2
L4 = 0.12

    
class arm_ik:


    def __init__(self):

        rospy.init_node('arm_ik')

	# publish command message to joints/servos of arm
    	self.joint1 = rospy.Publisher('/waist_controller/command',Float64, queue_size=1)    # positive angle: counterclockwise 
	self.joint2 = rospy.Publisher('/shoulder_controller/command',Float64, queue_size=1) # positive angle: counterclockwise (look from the left)
    	self.joint3 = rospy.Publisher('/elbow_controller/command',Float64, queue_size=1)    # positive angle: same as above
    	self.joint4 = rospy.Publisher('/wrist_controller/command',Float64, queue_size=1)    # positive angle: same as above
	self.joint5 = rospy.Publisher('/hand_controller/command',Float64, queue_size=1)     # positive angle: close the gripper
	self.pos1 = Float64()
    	self.pos2 = Float64()
    	self.pos3 = Float64()
    	self.pos4 = Float64()
    	self.pos5 = Float64()

        rospy.Subscriber('/image/xyz', xyz, self.ikCallback)

        rospy.spin()


    def ikCallback(self, msg):

        rospy.loginfo('ready to go back to zero pose...')
        self.joint1.publish(0.0)
        self.joint2.publish(0.0)
        self.joint3.publish(0.0)
        self.joint4.publish(0.0)
        self.joint5.publish(-0.2)
        rospy.sleep(10)
        rospy.loginfo('initial pose finished.')

        x = msg.x
        y = msg.y
        z = msg.z

        [w, theta1, theta2, theta3] = self.ik(x, y, z+0.05)

        rospy.loginfo('waist: '+str(w)+'; shoulder: '+str(theta1)+'; elbow: '+str(theta2)+'; wrist: '+str(theta3))
        self.joint1.publish(w)
        self.joint2.publish(theta1)
        self.joint3.publish(theta2)
        rospy.sleep(15)
        self.joint4.publish(theta3)
        rospy.sleep(5)
        rospy.loginfo('above the destination.')



        [w, theta1, theta2, theta3] = self.ik(x, y, z)

        rospy.loginfo('waist: '+str(w)+'; shoulder: '+str(theta1)+'; elbow: '+str(theta2)+'; wrist: '+str(theta3))
        self.joint1.publish(w)
        self.joint2.publish(theta1)
        self.joint3.publish(theta2)
        self.joint4.publish(theta3)
        rospy.sleep(5)
        rospy.loginfo('reached the destination.')

        self.joint5.publish(0.4) # close the gripper
        rospy.sleep(5)
        rospy.loginfo('gripper closed.')

        [w, theta1, theta2, theta3] = self.ik(x, y, z+0.03)

        rospy.loginfo('waist: '+str(w)+'; shoulder: '+str(theta1)+'; elbow: '+str(theta2)+'; wrist: '+str(theta3))
        self.joint1.publish(w)
        self.joint2.publish(theta1)
        self.joint3.publish(theta2)
        self.joint4.publish(theta3)
        rospy.sleep(5)
        rospy.loginfo('got the object.')

        self.joint5.publish(0.0) # close the gripper
        rospy.sleep(5)
        rospy.loginfo('gripper opened.')


    def ik(self, x, y, z):

        # solving: from (x,y,z) to (q1,q2,q3,q4)
        # theta1: shoulder; theta2: elbow; theta3: wrist
        # w: waist; g: grpper

        global L1
        global L2
        global L3
        global L4
        print(str(L1)+' '+str(L2)+' '+str(L3)+' '+str(L4))
        pi = math.pi

        px = z + L4/(math.sqrt(2)) - L1
        py = math.sqrt(math.pow(x,2)+math.pow(y,2)) - L4/(math.sqrt(2))
        rho = math.sqrt(math.pow(px,2)+math.pow(py,2))
        print(str(px) + ' ' + str(py) + ' ' + str(rho))
        print(str(((math.pow(L1,2)+math.pow(L2,2))-math.pow(rho,2))/(2.0*L1*L2)))
        phi = math.acos(((math.pow(L1,2)+math.pow(L2,2))-math.pow(rho,2))/(2.0*L1*L2))
    
        theta2 = pi - phi
        theta2_2 = phi - pi

        tau = math.acos(((math.pow(L1,2)+math.pow(rho,2))-math.pow(L2,2))/(2*L1*rho))
        gamma = math.atan2(py, px)
    
        theta1 = gamma - tau
        theta1_2 = gamma + tau
    
        theta3 = 0.75*pi - theta1 - theta2
        theta3_2 = 0.75*pi - theta1_2 - theta2_2

        w = math.atan2(y,x) - 0.5*pi

        return [w, theta1, theta2, theta3]


if __name__ == '__main__':
    
    arm_ik()

