#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" nav_test.py - Version 1.1 2013-12-20
实现多点导航,并实现ttb自定位
在导航途中判断信息点在路径的左侧还是右侧
按照任务流完成where is this

"""

import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
from std_msgs.msg import Int8
from whereisthis_control.msg import markpoint
from whereisthis_control.msg import whereisthis


# 判断字符串是否相等
def str_equal(a,b):                               #没用到？
    a_ = "".join(a.split())
    b_ = "".join(b.split())
    flag = a_ == b_
    return flag


class NavTest():

    def __init__(self):

        # -----------------初始化节点-----------------
        rospy.init_node('where_nav_core', anonymous=False)
        # ----------------建立响应函数-----------------
        rospy.on_shutdown(self.shutdown)
        # 控制频率
        self.rate = rospy.Rate(1)
        # 速度控制话题,用于速度缓冲 Publisher to manually control the robot (e.g. to stop it, queue_size=5)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        # 目标点状态Goal state return values
        goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED',
                       'SUCCEEDED', 'ABORTED', 'REJECTED',
                       'PREEMPTING', 'RECALLING', 'RECALLED',
                       'LOST']


        # --------------------定义初始的目标序列--------------------
        self.target_goals = dict()
        # TODO:需要初始定义的地方
        self.init_position()
        # 记录当前任务下需要前往的目标点
        self.current_goal = Pose()
        # 是否开始导航
        self.start_navigation = False
        # 是否得到控制命令
        self.get_control_order = False
        #---------------------用于记录任意时刻的位置与姿态-------------
        # 按照需求记录位姿
        rospy.Subscriber("/navi/present_pose", Pose, self.get_current_pose, queue_size=1)
        # 同时记录目标位置标记
        self.current_pose_label = str()

        #--------------------------------链接move_base 服务------------------------------
        # 链接服务器
        # Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        # Wait 60 seconds for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move base server")
        #-------------------------------设置初始位姿--------------------------------------
        #-------------------------------阻塞进程-----------------------------------------
        # 阻塞进程,获取初始位姿
        # A variable to hold the initial pose of the robot to be set by
        # the user in RViz
        #self.initial_pose = PoseWithCovarianceStamped()
        rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
        #rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
        #rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        self.pub_current_pose = rospy.Publisher('initialpose',PoseWithCovarianceStamped,queue_size=1)
        # 阻塞进程
        # Make sure we have the initial pose
        #while initial_pose.header.stamp == "":
            #rospy.loginfo("header.stamp...")
            #rospy.sleep(1)

        #-------------------------设置初始变量------------------------------------------------
        # 下面都是一些简单的信息,用于记录导航状态
        # Variables to keep track of success rate, running time,
        # and distance traveled
        i = 0
        n_goals = 0
        n_successes = 0
        distance_traveled = 0
        start_time = rospy.Time.now()
        running_time = 0
        # Get the initial pose from the user
        # -------------------------等待控制流唤醒---------------------------------------------
        rospy.Subscriber("/speech/infopoint", Int8, self.get_infopoint, queue_size=1)
        rospy.Subscriber("/speech/markpoint", Int8, self.get_goalpoint, queue_size=1)
        rospy.Subscriber("/whereisthis_control", whereisthis, self.control_callback, queue_size=1)
        self.control_pub = rospy.Publisher("/whereisthis_control", whereisthis, queue_size=1)
        self.markpoint_pub = rospy.Publisher("/navi/markpoint", markpoint, queue_size=1)
        #self.guide_pub = rospy.Publisher("/control", markpoint, queue_size=1)
        self.msg = whereisthis()
        self.pointmsg = markpoint()
        #---------------------等待goal唤醒---------------------------------
        self.last_location = Pose()
        self.location = Pose()
        #----------------------开始导航----------------------------------------
        # Begin the main loop and run through a sequence of locations
        rospy.loginfo("Starting navigation test")
        # Begin the main loop and run through a sequence of locations
        while not rospy.is_shutdown():
            # If we've gone through the current sequence,
            # start with a new random sequence
            if not self.start_navigation:
                # rospy.loginfo("Wait for new goal!")
                self.rate.sleep()
                continue
            # 避免重复进入
            self.start_navigation = False
            rospy.loginfo("%s",self.current_goal)
            self.location = self.current_goal
            # 更新初始化位姿,实时更新位姿
            #if self.initial_pose.header.stamp == "":
                #distance = sqrt(pow(self.location.position.x -
                                    #self.last_location.position.x, 2) +
                                #pow(self.location.position.y -
                                    #self.last_location.position.y, 2))
            #else:
            #rospy.loginfo("Updating current pose.")
            distance = sqrt(pow(self.location.position.x -
                                    self.last_location.position.x, 2) +
                                pow(self.location.position.y -
                                    self.last_location.position.y, 2))
                #self.initial_pose.header.stamp = ""

            # Store the last location for distance calculations
            self.last_location = self.location
            # Increment the counters
            i += 1
            n_goals += 1

            # Set up the next goal location
            self.goal = MoveBaseGoal()
            self.goal.target_pose.pose = self.location
            self.goal.target_pose.header.frame_id = 'map'
            self.goal.target_pose.header.stamp = rospy.Time.now()

            # Let the user know where the robot is going next
            rospy.loginfo("Going to: %f %f %f"%(self.location.position.x,self.location.position.y,self.location.position.z))
            # Start the robot toward the next location
            # 开始机器人向下一个节点
            self.move_base.send_goal(self.goal,done_cb= self.donecb,active_cb=self.activecb,feedback_cb=self.feedbackcb)

            # Allow 5 minutes to get there
            finished_within_time = self.move_base.wait_for_result(rospy.Duration(300))

            # Check for success or failure
            if not finished_within_time:
                self.move_base.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
            else:
                state = self.move_base.get_state()
                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Goal succeeded!")
                    n_successes += 1
                    distance_traveled += distance
                    rospy.loginfo("State:" + str(state))
                    self.pub_control()
                else:
                  rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))

            # How long have we been running?
            running_time = rospy.Time.now() - start_time
            running_time = running_time.secs / 60.0

            # Print a summary success/failure, distance traveled and time elapsed
            rospy.loginfo("Success so far: " + str(n_successes) + "/" +
                          str(n_goals) + " = " +
                          str(100 * n_successes/n_goals) + "%")
            rospy.loginfo("Running time: " + str(trunc(running_time, 1)) +
                          " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
            # 接收下一个命令
            self.get_control_order = False # no use
            self.rate.sleep()


    def get_infopoint(self, msg):

        self.destination = msg.data
        self.infopoint = msg.data


    def get_goalpoint(self, msg):

        self.destination = msg.data


    def control_callback(self, msg=whereisthis()):

        # 避免重复获取命令
        # 执行完成之后就可以转为False
        #if self.get_control_order == True :
            #return
        self.get_control_order = True # no use
        self.msg = msg

        if self.msg.NowTask == msg.GotoPoint or self.msg.NowTask == msg.Guiding or self.msg.NowTask == msg.BacktoPoint:

            if self.msg.NowTask == msg.BacktoPoint:
                self.destination = self.infopoint

            if self.destination == 1 and msg.FinishState== False:
                # 去A
                self.start_navigation = True
                self.current_goal = self.target_goals["A"]
            elif self.destination == 2 and msg.FinishState== False:
                # B
                self.start_navigation = True
                self.current_goal = self.target_goals["B"]
            elif self.destination == 3 and msg.FinishState== False:
                # C
                self.start_navigation = True
                self.current_goal = self.target_goals["C"]
            elif self.destination == 4 and msg.FinishState== False:
                # D
                self.start_navigation = True
                self.current_goal = self.target_goals["D"]
        

    # TODO:设置当前的位姿为一预设姿态
    # BUG:预设一般姿态,需要测试
    def set_initial_pose(self,target_pose):

        self.init_position = PoseWithCovarianceStamped
        self.initial_pose.pose = target_pose
        self.initial_pose.header.stamp = rospy.Time.now()
        self.pub_current_pose.publish(self.initial_pose)


    # 初始化位置
    def init_position(self):

        # Door
        self.set_goal("A",Pose(Point(2.86, -5.08, 0.000), Quaternion(0.000, 0.000, 0.589653, 0.807656)))
        # Table
        self.set_goal("B",Pose(Point(0.279, -0.265, 0.000), Quaternion(0.000, 0.000, 0.541621, 0.840623)) )
        # Chair
        self.set_goal("C",Pose(Point(1.24, -3.63, 0.000), Quaternion(0.000, 0.000, -0.201015, 0.979588)) )
        # test
        self.set_goal("D",Pose(Point(1.62, -1.34, 0.000), Quaternion(0.000, 0.000, 0.940019, 0.341123)) )


    def set_goal(self, name ,target_pose):

        self.target_goals[str(name)] = target_pose


    # 获取当前的位置和姿态
    # 这个可能没有这么简单
    def get_current_pose(self, msg=Pose(),msg2=whereisthis()):

        print("------------------------GETTING POSE-------------")
        rospy.loginfo("get %s : %f %f %f %f"%("current_pose",msg.position.x,msg.position.y,msg.orientation.z,msg.orientation.w))
        distanceA = sqrt(pow(msg.position.x - self.target_goals["A"].position.x, 2) +pow(msg.position.y -self.target_goals["A"].position.y, 2))
        distanceB = sqrt(pow(msg.position.x - self.target_goals["B"].position.x, 2) +pow(msg.position.y -self.target_goals["B"].position.y, 2))
        distanceC = sqrt(pow(msg.position.x - self.target_goals["C"].position.x, 2) +pow(msg.position.y -self.target_goals["C"].position.y, 2))
        distanceD = sqrt(pow(msg.position.x - self.target_goals["D"].position.x, 2) +pow(msg.position.y -self.target_goals["D"].position.y, 2))
        ky=self.current_goal.position.y-msg.position.y
        kx=self.current_goal.position.x-msg.position.x
        xky=self.current_goal.position.x*ky
        ykx=self.current_goal.position.y*kx
##########################################################
         
        
        if self.msg.NowTask == self.msg.Guiding:
            if distanceA < 0.6 and self.current_goal != self.target_goals["A"] and self.last_location !=self.target_goals["A"]:
                rospy.loginfo("I'm near A!")
                self.pointmsg.id = 1
                if ky*kx >0:
                    if ky*self.target_goals["A"].position.x-self.target_goals["A"].position.y*kx-xky+ykx >0:
                        rospy.loginfo("A is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("A is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky*kx <0:
                    if ky*self.target_goals["A"].position.x-self.target_goals["A"].position.y*kx-xky+ykx <0:
                        rospy.loginfo("A is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("A is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["A"].position.x > self.current_goal.position.x:
                        rospy.loginfo("A is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("A is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["A"].position.y < self.current_goal.position.y:
                        rospy.loginfo("A is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("A is on my LEFT!")
                        self.pointmsg.side = 0
                elif kx==0 and ky>0:
                    if self.target_goals["A"].position.x > self.current_goal.position.x:
                        rospy.loginfo("A is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("A is on my LEFT!")
                        self.pointmsg.side = 1
                self.markpoint_pub.publish(self.pointmsg)

            if distanceB < 0.6 and self.current_goal != self.target_goals["B"] and self.last_location !=self.target_goals["B"]:
                rospy.loginfo("I'm near B!")
                self.pointmsg.id = 2
                if ky*kx >0:
                    if ky*self.target_goals["B"].position.x-self.target_goals["B"].position.y*kx-xky+ykx >0:
                        rospy.loginfo("B is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("B is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky*kx <0:
                    if ky*self.target_goals["B"].position.x-self.target_goals["B"].position.y*kx-xky+ykx <0:
                        rospy.loginfo("B is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("B is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["B"].position.x > self.current_goal.position.x:
                        rospy.loginfo("B is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("B is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["B"].position.y < self.current_goal.position.y:
                        rospy.loginfo("B is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("B is on my LEFT!")
                        self.pointmsg.side = 0
                elif kx==0 and ky>0:
                    if self.target_goals["B"].position.x > self.current_goal.position.x:
                        rospy.loginfo("B is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("B is on my LEFT!")
                        self.pointmsg.side = 1  
                self.markpoint_pub.publish(self.pointmsg)  

            if distanceC < 0.6 and self.current_goal != self.target_goals["C"] and self.last_location !=self.target_goals["C"]:
                rospy.loginfo("I'm near C!")
                self.pointmsg.id = 3
                if ky*kx >0:
                    if ky*self.target_goals["C"].position.x-self.target_goals["C"].position.y*kx-xky+ykx >0:
                        rospy.loginfo("C is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("C is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky*kx <0:
                    if ky*self.target_goals["C"].position.x-self.target_goals["C"].position.y*kx-xky+ykx <0:
                        rospy.loginfo("C is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("C is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["C"].position.x > self.current_goal.position.x:
                        rospy.loginfo("C is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("C is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["C"].position.y < self.current_goal.position.y:
                        rospy.loginfo("C is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("C is on my LEFT!")
                        self.pointmsg.side = 0
                elif kx==0 and ky>0:
                    if self.target_goals["C"].position.x > self.current_goal.position.x:
                        rospy.loginfo("C is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("C is on my LEFT!")
                        self.pointmsg.side = 1
                self.markpoint_pub.publish(self.pointmsg)    

            if distanceD < 0.6 and self.current_goal != self.target_goals["D"] and self.last_location !=self.target_goals["D"]:
                rospy.loginfo("I'm near D!")
                self.pointmsg.id = 2
                if ky*kx >0:
                    if ky*self.target_goals["D"].position.x-self.target_goals["D"].position.y*kx-xky+ykx >0:
                        rospy.loginfo("D is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("D is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky*kx <0:
                    if ky*self.target_goals["D"].position.x-self.target_goals["D"].position.y*kx-xky+ykx <0:
                        rospy.loginfo("D is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("D is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["D"].position.x > self.current_goal.position.x:
                        rospy.loginfo("D is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("D is on my LEFT!")
                        self.pointmsg.side = 0
                elif ky==0 and kx>0:
                    if self.target_goals["D"].position.y < self.current_goal.position.y:
                        rospy.loginfo("D is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("D is on my LEFT!")
                        self.pointmsg.side = 0
                elif kx==0 and ky>0:
                    if self.target_goals["D"].position.x > self.current_goal.position.x:
                        rospy.loginfo("D is on my RIGHT!")
                        self.pointmsg.side = 1
                    else:
                        rospy.loginfo("D is on my LEFT!")
                        self.pointmsg.side = 1    

                self.markpoint_pub.publish(self.pointmsg)       
                 
    def donecb(self,state,result):

        pass


    def activecb(self):

        pass


    def feedbackcb(self,fb):

        if self.start_navigation :
            self.move_base.cancel_goal()
            rospy.loginfo("cancel_goal and send new goal")


    def update_initial_pose(self, initial_pose):

        self.initial_pose = initial_pose


    def shutdown(self):

        rospy.loginfo("Stopping the robot...")
        self.move_base.cancel_goal()
        rospy.sleep(2)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)


    def pub_control(self, total_time=5):

        self.msg.FinishState = True
        #self.pub_rate = rospy.Rate(10)
        rospy.sleep(1)
        #self.current_task = self.msg.NowTask
        #self.Next_task = self.msg.NextTask
        self.FinishState = self.msg.FinishState # no use
        self.Need_help = self.msg.NeedHelp # no use
        self.control_pub.publish(self.msg)
            #self.pub_rate.sleep()
        #print("feedback %d task"%self.msg.NowTask)
	print("feedback: %s"%str(self.msg))


def trunc(f, n):

    # Truncates/pads a float f to n decimal places without rounding
    slen = len('%.*f' % (n, f))
    return float(str(f)[:slen])


if __name__ == '__main__':

    try:
        NavTest()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
