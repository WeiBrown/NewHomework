#! /usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
'''
roslib.load_manifest ('speech')
roslib.load_manifest () reads the package manifest and sets up the python library path based on the package dependencies.
It's required for older rosbuild-based packages, but is no longer needed on catki
'''
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Uint8
import os
import sys
import nltk
import time
import wave
import datetime
import pyaudio
import Levenshtein as leven
from sound_play.libsoundplay import SoundClient
import random
from whereisthis_control.msg import markpoint
from whereisthis_control.msg import whereisthis
from whereisthis_control.msg import guest
class speech_transform(object):
    def __init__(self):
        self.nowmission = 0
        print("will launch terminal")
        os.system("gnome-terminal -x bash -c 'rosrun xfei_asr speech_recognition'")
        self.sh = SoundClient(blocking = True)
        self.voice = rospy.get_param("~voice", "voice_kal_diphone")
	self.inforid = 10
	self.destid = 10
        self.information_point = rospy.Publisher('speech/infopoint', Uint8) #information_point的信息
        self.mark_point = rospy.Publisher('speech/markpoint', Uint8)#目的地
        print "Launching subscriber"
        self.hash_id = ["place_id_1", "place_id_2", "place_id_3", "place_id_4", "place_id_5", "place_id_6", "place_id_7"]
        #hash_id[id-1] = reference point
        self.description = []# supposed to be a (7,7) matrix des[i][j] refers to the description from point i to point j.
        rospy.Subscriber("/whereisthis_control", whereisthis, self.whereisthis_Callback)	
        rospy.Subscriber("/image/findperson", guest, self.image_findperson_Callback)
        rospy.Subscriber("/navi/markpoint", markpoint, self.guide_Callback)
        rospy.Subscriber("/xunfei_to_control", String, self.xfeiCallback)
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
    def image_findpersonCallback(self, msg):
        return 0 
    def guide_Callback(self, msg):
        position_des = "Unknown"
        reference_des = "Unknown"
        if msg.side == 0:
            position_des = "on your left side"
        else:
             if msg.side == 1:
                position_des = "on your right side"
             else：
                position_des = "in your front"
        reference_des = self.hash_id[msg.id - 1]
        self.sh.say('You can see a {}'.format(reference_des), self.voice)
        self.sh.say(position_des, self.voice)
       
    def whereisthis_Callback(self,msg):

        if msg.NowTask == msg.GetPoint:
            self.sh.say("Please tell me where is the information point?", self.voice)
            self.sh.say("If you are ready to tell me, please say jack to launch me first.", self.voice)
            self.nowmission = 1
        if msg.NowTask == msg.GoalDescription:
            self.sh.say("Please tell me what are you looking for?", self.voice)
            self.sh.say("If you are ready to tell me, please say jack to launch me first.", self.voice)
            self.nowmission = 2
        if msg.NowTask = msg.Guiding:
            self.nowmission = 3
            
    def xfeiCallback(self,msg):
        if msg.data.strip()=='':
            self.sh.say("Sorry I did not hear what you just said clearly", self.voice)
            self.sh.say("please tell me again", self.voice)
        else:
            if self.nowmission == 1:
                edit_distance = []
                ans = str(msg.data)
                tokens = nltk.word_tokenize(ans)
                tokens = nltk.pos_tag(tokens)
                for token in tokens:
                    if (token[1] == "NNP") or (token[1] == "NN") or (token[1] == "NNS") :
                        edit_distance.append([])
                        for i in range(len(self.hash_id)):
                            edit_distance[-1].append(leven.distance(token[0], self.hash_id[i]))
                infor_id = self.generate_minimum_word(edit_distance)  
		self.inforid = infor_id-1   
                self.information_point.publish(infor_id)
		          
            else:
                if self.nowmission == 2:
                    edit_distance = []
                    ans = str(msg.data)
                    tokens = nltk.word_tokenize(ans)
                    tokens = nltk.pos_tag(tokens)
                    for token in tokens:
                        if (token[1] == "NNP") or (token[1] == "NN") or (token[1] == "NNS") :
                            edit_distance.append[]
                            for i in range(len(self.hash_id)):
                                edit_distance[-1].append(leven.distance(token[0], self.hash_id[i]))
                    infor_id = self.generate_minimum_word(edit_distance)  
   		    self.destid  = infor_id-1
                    self.mark_point.publish(infor_id)  
		    #self.sh.say(self.description[self.inforid, self.destid], self.voice)        
                            
if __name__ == '__main__':
    rospy.init_node("receptionist_test", anonymous=True)
    rospy.loginfo('whereisthis_speech.py is running...')
    trans = speech_transform()
    rospy.spin()


