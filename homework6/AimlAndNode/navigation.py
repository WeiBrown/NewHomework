#!/usr/bin/env python

import rospy

import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String

def listens():
    rospy.init_node("location",anonymous=True)
    rospy.loginfo("start mission")
    #post msg to nav() 
    rospy.Subscriber("/voiceWords",String,nav)
    rospy.spin()
def clean():
    rospy.loginfo("mission complete")


def nav(data):
    rospy.on_shutdown(clean)
    sign = 0
    text = data.data
    if text == 'HEY JACK':
        rospy.loginfo("start mission")
    if text == 'GO TO THE WASHROOM':
        aim_x = -1.8
        aim_y = 1.52
        aim_theta = -0.00143
        sign = 1
    if text == 'GO TO THE KITCHEN':
        aim_x = -0.768
        aim_y = -2.38
        aim_theta = -0.00143
        sign = 1
    if text == 'GO TO THE DOOR':
        aim_x = 2.15
        aim_y = -2.31
        aim_theta = -0.00143
        sign = 1
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
    rospy.loginfo("Waiting for move_base action server...")
    move_base.wait_for_server(rospy.Duration(120))


    rospy.loginfo('Ready')

    locations = dict()

    print(sign)
    if sign == 1:
        quaternion = quaternion_from_euler(0.0, 0.0, aim_theta)
        locations['A'] = Pose(Point(aim_x, aim_y, 0.000), Quaternion(quaternion[0], quaternion[1], quaternion[2], quaternion[3]))
    
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    goal.target_pose.header.stamp = rospy.Time.now()
    print('start')
    if sign == 1:
        print('start')
        rospy.loginfo("Going to point")
        rospy.sleep(2)
        goal.target_pose.pose = locations['A']
        move_base.send_goal(goal)
        waiting = move_base.wait_for_result(rospy.Duration(300))
        sign = 0

    move_base.cancel_goal()

if __name__=="__main__":
    listens()
    rospy.spin()
