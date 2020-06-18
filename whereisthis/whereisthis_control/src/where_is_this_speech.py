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

#problem: guiding one person for 2 times continuously? if so, is returning to information point required?

# markpointid  | name
#       1      | sofa
#       2      | table
#       3      | chair
#       4      | dinning table (can this word be detected?)
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
	self.inforid = 10 # information point (0~6)
	self.destid = 10 # goal point (0~6)
        self.nowperson = 0 # current person (1~3)
        self.personinfo = [[0,0], [0,0], [0,0]] # for every person: [number of show-up times, first-time goal point]

        self.information_point = rospy.Publisher('/speech/infopoint', Int8, queue_size=1) # information_point
        self.mark_point = rospy.Publisher('/speech/markpoint', Int8, queue_size=1) # goal point

        print("Launching subscriber")

        self.hash_id = ['sofa', 'table', 'chair', 'dinning table', 'television', 'bed', 'wardrobe'] 
        # hash_id[id-1] = reference point (no static ref. point however)
        self.generate_description() # a 7x7 matrix; self.description[i][j] description the route from infopoint(i) to goalpoint(j).

        rospy.Subscriber("/whereisthis_control", whereisthis, self.whereisthis_Callback)	
        rospy.Subscriber("/image/findperson", Int8, self.image_findperson_Callback)
        rospy.Subscriber("/navi/markpoint", markpoint, self.guide_Callback)
        rospy.Subscriber("/xunfei_to_control", String, self.xfeiCallback)


    def generate_description(self): # func.: generate the guiding description from one place to another.

        self.description = []
        tmp = []
        tmp.append('I will tell you the route from sofa to table.')
        tmp.append('I will tell you the route from sofa to chair.')
        tmp.append('I will tell you the route from sofa to dinning table.')
        tmp.append('I will tell you the route from sofa to television.')
        tmp.append('I will tell you the route from sofa to bed.')
        tmp.append('I will tell you the route from sofa to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from table to sofa.')
        tmp.append('I will tell you the route from table to chair.')
        tmp.append('I will tell you the route from table to dinning table.')
        tmp.append('I will tell you the route from table to television.')
        tmp.append('I will tell you the route from table to bed.')
        tmp.append('I will tell you the route from table to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from chair to sofa.')
        tmp.append('I will tell you the route from chair to table.')
        tmp.append('I will tell you the route from chair to dinning table.')
        tmp.append('I will tell you the route from chair to television.')
        tmp.append('I will tell you the route from chair to bed.')
        tmp.append('I will tell you the route from chair to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from dinning table to sofa.')
        tmp.append('I will tell you the route from dinning table to table.')
        tmp.append('I will tell you the route from dinning table to chair.')
        tmp.append('I will tell you the route from dinning table to television.')
        tmp.append('I will tell you the route from dinning table to bed.')
        tmp.append('I will tell you the route from dinning table to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from television to sofa.')
        tmp.append('I will tell you the route from television to table.')
        tmp.append('I will tell you the route from television to chair.')
        tmp.append('I will tell you the route from television to dinning table.')
        tmp.append('I will tell you the route from television to bed.')
        tmp.append('I will tell you the route from television to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from bed to sofa.')
        tmp.append('I will tell you the route from bed to table.')
        tmp.append('I will tell you the route from bed to chair.')
        tmp.append('I will tell you the route from bed to dinning table.')
        tmp.append('I will tell you the route from bed to television.')
        tmp.append('I will tell you the route from bed to wardrobe.')
        self.description.append(tmp)

        tmp = []
        tmp.append('I will tell you the route from wardrobe to sofa.')
        tmp.append('I will tell you the route from wardrobe to table.')
        tmp.append('I will tell you the route from wardrobe to chair.')
        tmp.append('I will tell you the route from wardrobe to dinning table.')
        tmp.append('I will tell you the route from wardrobe to television.')
        tmp.append('I will tell you the route from wardrobe to bed.')
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

        self.sh.say('You can see a {}'.format(reference_des), self.voice)
        self.sh.say(position_des, self.voice)
       

    def whereisthis_Callback(self, msg): # func.: get the point (info point or goal point)

        if msg.NowTask == msg.GetPoint and msg.FinishState == False:
            self.sh.say("Please tell me the information point. If you are ready to tell me, please say jack to launch me first.", self.voice)
            self.nowmission = 1 # get information point
        elif msg.NowTask == msg.GoalDescription and msg.FinishState == False:
            self.sh.say("Please tell me what are you looking for? If you are ready to tell me, please say jack to launch me first.", self.voice)
            self.nowmission = 2 # get goal point
        elif msg.NowTask == msg.Guiding and msg.FinishState == False:
            self.nowmission = 3 # no use?
            

    def xfeiCallback(self,msg):

        if msg.data.strip()=='':
            self.sh.say("Sorry I did not hear what you just said clearly, please tell me again", self.voice)

        elif self.nowmission == 1:
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
            self.information_point.publish(self.inforid)

        elif self.nowmission == 2:
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
            self.mark_point.publish(self.destid)  

            if self.personinfo[self.nowperson-1][0] == 1: # didn't show up before
                self.personinfo[self.nowperson-1][1] = self.destid # store the goal point
                self.sh.say(self.description[self.inforid-1][self.destid-1], self.voice) # route: from info to goal
            elif self.personinfo[self.nowperson-1][0] == 2: # has come before
                self.sh.say('last time you asked me to go to {}, now I will tell you how to go to {} from there.'.format(self.hash_id[self.personinfo[self.nowperson-1][1]-1], self.hash_id[self.destid-1]), self.voice) # route: from previous goal to current goal
                            

if __name__ == '__main__':
    rospy.init_node("speech_whereisthis", anonymous=False)
    rospy.loginfo('whereisthis_speech.py is running...')
    trans = speech_transform()
    rospy.spin()
