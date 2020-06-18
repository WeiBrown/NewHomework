#! /usr/bin/env python
# -*- coding: utf-8 -*-
'''
roslib.load_manifest ('speech')
roslib.load_manifest () reads the package manifest and sets up the python library path based on the package dependencies.
It's required for older rosbuild-based packages, but is no longer needed on catki
'''
import roslib
import rospy
import os
import sys
import random
import nltk
#nltk.download('punkt')
import time
import wave
import datetime
import pyaudio
import Levenshtein as leven
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Int8
from sound_play.libsoundplay import SoundClient
from whereisthis_control.msg import markpoint
from whereisthis_control.msg import whereisthis
#nltk.download()
#problem: guiding one person for 2 times continuously? if so, is returning to information point required?

# markpointid  | name
#       1      | door
#       2      | table
#       3      | chair
#       4      | test('dining table' cannot be detected)
#       5      | tv / television
#       6      | bed
#       7      | wardrobe

# publisher:
# /speech/infopoint: publish the id of information point
# /speech/markpoint: publish the id of goal point

# subscriber:
# /whereisthis_control: subscribe the id of the current sub-task
# /image/findperson: subscribe the id of the person
# /navi/markpoint: subscribe the if of passing-by point
class speech_transform(object):

    def __init__(self):

        self.nowmission = 0

        print("will launch terminal")
        os.system("gnome-terminal -x bash -c 'rosrun xfei_asr speech_recognition'")

        self.sh = SoundClient(blocking = True)
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
	self.inforid = 8 # information point (0~6)
	self.destid = 8 # goal point (0~6)
        self.nowperson = 0 # current person (1~3)
        self.personinfo = [[0,0], [0,0], [0,0]] # for every person: [number of show-up times, first-time goal point]

        self.information_point = rospy.Publisher('/speech/infopoint', Int8, queue_size=1) # information_point
        self.mark_point = rospy.Publisher('/speech/markpoint', Int8, queue_size=1) # goal point
        self.controlFeedback = rospy.Publisher('/whereisthis_control', whereisthis, queue_size=1) # finish signal for a sub-task

        print("Launching subscriber")

        self.hash_id = ['door', 'table', 'chair', 'test', 'television', 'bed', 'wardrobe','out of range'] 
        # hash_id[id-1] = reference point (no static ref. point however)
        self.generate_description() # a 7x7 matrix; self.description[i][j] description the route from infopoint(i) to goalpoint(j).

        rospy.Subscriber("/whereisthis_control", whereisthis, self.whereisthis_Callback)	
        rospy.Subscriber("/image/findperson", Int8, self.image_findperson_Callback)
        rospy.Subscriber("/navi/markpoint", markpoint, self.guide_Callback)
        rospy.Subscriber("/xunfei_to_control", String, self.xfeiCallback)


    def generate_description(self): # func.: generate the guiding description from one place to another.

        self.description = []
        tmp = []
        tmp.append('I will tell you the route from door to door.')
        tmp.append('I will tell you the route from door to table.  First you turn right, then you go straight for 2 meters. Then on your right is table.')
        tmp.append('I will tell you the route from door to chair. First you turn left, then you go straight for 1 and a half meter. In front of you is chair.')
        tmp.append('I will tell you the route from door to test.')
        tmp.append('I will tell you the route from door to television.')
        tmp.append('I will tell you the route from door to bed.')
        tmp.append('I will tell you the route from door to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from table to door.')
        tmp.append('I will tell you the route from table to tabel.')
        tmp.append('I will tell you the route from table to chair. First you turn right, then you go straight for 4 meters. In front of you is chair.')
        tmp.append('I will tell you the route from table to test.')
        tmp.append('I will tell you the route from table to television.')
        tmp.append('I will tell you the route from table to bed.')
        tmp.append('I will tell you the route from table to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from chair to door.')
        tmp.append('I will tell you the route from chair to table.')
        tmp.append('I will tell you the route from chair to chair.')
        tmp.append('I will tell you the route from chair to test.')
        tmp.append('I will tell you the route from chair to television.')
        tmp.append('I will tell you the route from chair to bed.')
        tmp.append('I will tell you the route from chair to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from test to door.')
        tmp.append('I will tell you the route from test to table.')
        tmp.append('I will tell you the route from test to chair.')
        tmp.append('I will tell you the route from test to test.')
        tmp.append('I will tell you the route from test to television.')
        tmp.append('I will tell you the route from test to bed.')
        tmp.append('I will tell you the route from test to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from television to door.')
        tmp.append('I will tell you the route from television to table.')
        tmp.append('I will tell you the route from television to chair.')
        tmp.append('I will tell you the route from television to test.')
        tmp.append('I will tell you the route from television to television.')
        tmp.append('I will tell you the route from television to bed.')
        tmp.append('I will tell you the route from television to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from bed to door.')
        tmp.append('I will tell you the route from bed to table.')
        tmp.append('I will tell you the route from bed to chair.')
        tmp.append('I will tell you the route from bed to test.')
        tmp.append('I will tell you the route from bed to television.')
        tmp.append('I will tell you the route from bed to bed.')
        tmp.append('I will tell you the route from bed to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from wardrobe to door.')
        tmp.append('I will tell you the route from wardrobe to table.')
        tmp.append('I will tell you the route from wardrobe to chair.')
        tmp.append('I will tell you the route from wardrobe to test.')
        tmp.append('I will tell you the route from wardrobe to television.')
        tmp.append('I will tell you the route from wardrobe to bed.')
        tmp.append('I will tell you the route from wardrobe to wardrobe.')
        self.description.append(tmp)


    def generate_minimum_word(self, matrix):

        minimum_word = ["unknown", 10]
        for i in matrix:
            for j in range(len(i)):
                if i[j] < minimum_word[1]:
                    minimum_word[0] = self.hash_id[j]
                    minimum_word[1] = i[j]

        for i in range(len(self.hash_id)):
            if self.hash_id[i] == minimum_word[0]:
                return i+1


    def image_findperson_Callback(self, msg): # expected func.: to show the 'memory' when describing the route.

        # the code below is only for testing
        self.nowperson = msg.data
        self.personinfo[self.nowperson-1][0] += 1
        self.sh.say("Please tell me, what are you looking for? If you are ready to tell me, please say jack to launch me first", self.voice)
        self.nowmission = 2 # get goal point


    def guide_Callback(self, msg): # func.: introduce the objects around while guiding the guest to goal, just like a guidance

        position_des = "Unknown"
        reference_des = "Unknown"

        if msg.side == 0:
            position_des = "on your left side"
        elif msg.side == 1:
            position_des = "on your right side"
        elif msg.side == 2:
            position_des = "in your front"
        reference_des = self.hash_id[msg.id - 1]

        self.sh.say('You can see a {}, {}'.format(reference_des, position_des), self.voice)
       

    def whereisthis_Callback(self, msg): # func.: get the point (info point or goal point)

        if msg.NowTask == msg.GetPoint and msg.FinishState == False:
            self.sh.say("Please tell me the information point. If you are ready to tell me, please say jack to launch me first.", self.voice)
            self.nowmission = 1 # get information point
            

    def xfeiCallback(self,msg):

        if msg.data.strip()=='':
            self.sh.say("Sorry I did not hear what you just said clearly, please tell me again", self.voice)

        elif self.nowmission == 1: # 'Guiding' task, get information point and publish it

            # recognize information point
            edit_distance = []
            ans = str(msg.data)
            tokens = nltk.word_tokenize(ans)
            tokens = nltk.pos_tag(tokens)
            for token in tokens:
                if (token[1] == "NNP") or (token[1] == "NN") or (token[1] == "NNS") :
                    edit_distance.append([])
                    for i in range(len(self.hash_id)):
                        edit_distance[-1].append(leven.distance(token[0], self.hash_id[i]))
            self.inforid = self.generate_minimum_word(edit_distance)
            
            if self.inforid >= 1 and self.inforid <= 8: # legal information point (includin cases of 'outta range')

                #publish information point
                t = Int8()
                t.data = self.inforid
                self.information_point.publish(t)
                self.sh.say('My information point is {}. Thank you.'.format(self.hash_id[self.inforid-1]), self.voice)

                #publish finish signal to control console
                t = whereisthis()
                t.NowTask = t.GetPoint
                t.NextTask = t.DoorOpening
                t.FinishState = True
                t.NeedHelp = False
                self.controlFeedback.publish(t)

            else: # didn't catch the word, i.e. self.inforid is 'None'
                self.sh.say('Sorry, can you tell me again? If you are ready to tell me, please say jack to launch me first.', self.voice)

        elif self.nowmission == 2: # 'GoalDescription' task, get destination point and publish it

            # recognize destination
            edit_distance = []
            ans = str(msg.data)
            tokens = nltk.word_tokenize(ans)
            tokens = nltk.pos_tag(tokens)
            for token in tokens:
                if (token[1] == "NNP") or (token[1] == "NN") or (token[1] == "NNS") :
                    edit_distance.append([])
                    for i in range(len(self.hash_id)):
                        edit_distance[-1].append(leven.distance(token[0], self.hash_id[i]))

            self.destid = self.generate_minimum_word(edit_distance)  
            
            if self.destid >= 1 and self.destid <= 8: # legal destination point (includin cases of 'outta range': only for debugging)

                #publish destination point
                t = Int8()
                t.data = self.destid
                self.mark_point.publish(t)

                # describe the route
                if self.personinfo[self.nowperson-1][0] == 1: # didn't show up before

                    self.personinfo[self.nowperson-1][1] = self.destid # store the goal point
                    self.sh.say(self.description[self.inforid-1][self.destid-1], self.voice) # route: from info to dest

                elif self.personinfo[self.nowperson-1][0] >= 2: # has come before
    
                    self.sh.say(self.description[self.inforid-1][self.destid-1], self.voice) # route from info to dest
                    pre_dest_id = self.personinfo[self.nowperson-1][1]
                    pre_dest_name = self.hash_id[pre_dest_id-1] # previous destination
                    cur_dest_name = self.hash_id[self.destid-1] # current destination
                    self.sh.say('last time you asked me to go to {}. now I will also tell you how to go to {} from there.'.format(pre_dest_name, cur_dest_name), self.voice) 
                    # route: from previous goal to current goal
                    self.sh.say(self.description[pre_dest_id-1][self.destid-1], self.voice)

                # publish finish signal to control console
                t = whereisthis()
                t.NowTask = t.GoalDescription
                t.NextTask = t.Guiding
                t.FinishState = True
                t.NeedHelp = False
                self.controlFeedback.publish(t)

            else: # didn't catch the word, i.e. self.destid is 'None'
                self.sh.say('Sorry, can you tell me again? If you are ready to tell me, please say jack to launch me first.', self.voice)

if __name__ == '__main__':
    rospy.init_node("speech_whereisthis", anonymous=False)
    rospy.loginfo('whereisthis_speech.py is running...')
    trans = speech_transform()
    rospy.spin()
