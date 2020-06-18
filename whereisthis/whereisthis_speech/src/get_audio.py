#! /usr/bin/env python
# -*- coding: utf-8 -*-
import os
import sys
import time
import rospy
import wave
import pyaudio
from std_msgs.msg import String
from std_msgs.msg import Int8


CHUNK = 256
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 11025
RECORD_SECONDS = 10 #录音时间
project_name = '/home/dell/catkin_ws/src/Whereisthis/whereisthis_speech/recording/recording'
start_record = True # 用来控制录音的开始
class Audio_recorder():
    def __init__(self):
        self.sub = rospy.Subscriber("/kws_data", String, self.Record)
        self.stoprecord_sub = rospy.Subscriber("/stop_record", Int8, self.Stop)
        self.stoprecord_pub = rospy.Publisher("/stop_record", Int8)
        self.stop_record = False # 用来控制录音的结束

    def Stop(self, msg):
        self.stoprecord_pub.publish(0)
        if msg.data == 1:
            print('should stop recording now')
            self.stop_record = True
        
    def Record(self, msg):
        if msg.data.strip() == 'jack':
            print('starting get_audio()')
            self.get_audio()

    def get_audio(self):
        start_record = True
        if start_record:
            recorder = pyaudio.PyAudio()
            t = int(time.time())  #时间戳
            file_name = project_name + '_' + str(t) + '.wav'
            print ("[INFO] Start to record input audio and save to file: %s"%(file_name))
            stream = recorder.open(
                format=FORMAT,
                channels=CHANNELS,
                rate=RATE,
                input=True,
                frames_per_buffer=CHUNK
            )
            frames = []
            for i in range(int(RATE / CHUNK * RECORD_SECONDS)):
                if self.stop_record == True:
                    print ("[INFO] Stop recording")
                    break
                data = stream.read(CHUNK)
                frames.append(data)
            print ("[INFO] Recording finished, now save wav file")
            self.stop_record = False

            stream.stop_stream()
            stream.close()
            recorder.terminate()

            wf = wave.open(file_name, 'wb')
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(recorder.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b''.join(frames))
            wf.close()

            start_record = False


if __name__ == '__main__':
    rospy.init_node("launching_recorder", anonymous=True)
    print('get_audio.py staring...')
    trans = Audio_recorder()
    rospy.spin()
