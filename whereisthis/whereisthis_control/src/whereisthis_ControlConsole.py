#!/usr/bin/env python
# -*- coding: utf-8 -*

import os
import thread
import rospy
import roslib
from whereisthis_control.msg import whereisthis
from whereisthis_control.msg import markpoint
from std_msgs.msg import String
from std_msgs.msg import Bool
import sys
import nltk
import time
import wave
import datetime
import pyaudio
from sound_play.libsoundplay import SoundClient
import random

# GetPoint -> DoorOpening -> GotoPoint -> GoalDescription -> Guiding -> BacktoPoint ─╮
#                                                 ↑                                                             │
#                                                 ╰──────────────────────────────────╯(6 circles)

# GetPoint: speech
# DoorOpening: navi
# GotoPoint: navi
# GoalDescription: image & speech
# Guiding: navi, arm & speech
# BacktoPoint: navi

class Whereisthis_Control:

    def __init__(self):

        msg = whereisthis()
        self.now_state = msg.GetPoint
        self.next_state = msg.DoorOpening
        self.Return_time = 0
        self.need_help = False
        self.control_sub = rospy.Subscriber("/whereisthis_control", whereisthis, self.controlCallback)
        self.control_pub = rospy.Publisher("/whereisthis_control", whereisthis, queue_size = 1)

        self.sh = SoundClient(blocking = True)
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")

        msg.NowTask = msg.GetPoint
        msg.NextTask = msg.DoorOpening
        msg.NeedHelp = False
        msg.FinishState = False
        rospy.sleep(0.5)

        self.sh.say('I am ready to start!', self.voice)

        # for i in range(5):
        self.control_pub.publish(msg)
        print("Start task Whereisthis...")
        print("----------GetPoint-----------")


    def controlCallback(self, msg):
        
        if msg.NeedHelp == True:
            print("Need help while no help way available.")
            # TODO:发布求救的节点（目前没有）

        elif msg.FinishState == True:
            #self.sh.say('hello!', self.voice)
            n_msg = whereisthis() #new msg
            # TODO:发布新的消息
            n_msg.NeedHelp = False
            n_msg.FinishState = False

            if msg.NowTask == n_msg.GetPoint and self.now_state == n_msg.GetPoint:
		self.sh.say('I got the control command.', self.voice)
                n_msg.NowTask = n_msg.DoorOpening
                n_msg.NextTask = n_msg.GotoPoint
                # self.sh.say('i am ready to enter the arena now', self.voice)
                n_msg.FinishState = True
                self.control_pub.publish(n_msg)
                print("-------DoorOpening-------")
                # for i in range(5):
                #     self.control_pub.publish(n_msg)
                self.now_state = n_msg.NowTask
                self.next_state= n_msg.NextTask

            elif msg.NowTask == n_msg.DoorOpening and self.now_state == n_msg.DoorOpening:
                n_msg.NowTask = n_msg.GotoPoint
                n_msg.NextTask = n_msg.GoalDescription
                self.sh.say('thank you. now i will go to the information point.', self.voice)
                print("-------GotoPoint-------")
                for i in range(5):
                    self.control_pub.publish(n_msg)
                self.now_state = n_msg.NowTask
                self.next_state= n_msg.NextTask
                            
            elif (msg.NowTask == n_msg.GotoPoint and self.now_state == n_msg.GotoPoint) or (msg.NowTask == n_msg.BacktoPoint and self.now_state == n_msg.BacktoPoint):
                # self.sh.say(str(msg), self.voice)
                if msg.NowTask == n_msg.BacktoPoint and self.now_state == n_msg.BacktoPoint:
                    if self.Return_time >= 3: # task finish
                        self.sh.say('I have finished the task.', self.voice)
                    else:
                        self.sh.say('I am ready to serve another guest', self.voice)
                        self.Return_time += 1
                n_msg.NowTask = n_msg.GoalDescription
                n_msg.NextTask = n_msg.Guiding
                self.sh.say('hi dear guest, please stand in front of me, thank you.', self.voice)
                print("-------GoalDescription-------")
                thread.start_new_thread(self.launch_image_core, ())
                rospy.sleep(2) # wait for image function to be ready & guest to stand in front of camera
                #for i in range(5):
                self.sh.say('i am ready to recognize you,and i will take a photo for you.', self.voice)
                self.control_pub.publish(n_msg)
                self.now_state = n_msg.NowTask
                self.next_state= n_msg.NextTask

            elif msg.NowTask == n_msg.GoalDescription and self.now_state == n_msg.GoalDescription:
                n_msg.NowTask = n_msg.Guiding
                n_msg.NextTask = n_msg.BacktoPoint
                self.sh.say('now i will guide you there. please follow me.', self.voice)
                print("-------Guiding-------")
                for i in range(5):
                    self.control_pub.publish(n_msg)
                self.now_state = n_msg.NowTask
                self.next_state= n_msg.NextTask
                
            elif msg.NowTask == n_msg.Guiding and self.now_state == n_msg.Guiding:
                n_msg.NowTask = n_msg.BacktoPoint
                n_msg.NextTask = n_msg.GoalDescription
                self.sh.say('now i will go back to the information point', self.voice)
                print("-------BacktoPoint-------")
                for i in range(5):
                    self.control_pub.publish(n_msg)
                self.now_state = n_msg.NowTask
                self.next_state= n_msg.NextTask
                

    def launch_image_core(self):
        os.system('rosrun whereisthis_image image_whereisthis.py')


if __name__ == '__main__':
    rospy.init_node('control_task', anonymous=False)
    control_ = Whereisthis_Control()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("-------Shutting down-------")

