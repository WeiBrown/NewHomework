#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib
import rospy
import numpy as np
from aip import AipFace
import cv2

from std_msgs.msg import String
from std_msgs.msg import Int8
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from whereisthis_control.msg import whereisthis

import os
import time
import base64
import copy

class user:

    def __init__(self):

        self.user_id = None
        self.facenum = 0


class Guest_Recognition():

    def __init__(self):

        rospy.init_node('whereisthis_image')
        # rospy.on_shutdown(self.cleanup)
        self.rate = rospy.Rate(1) # no use

        APP_ID = '18721308'
        API_KEY = 'lNQGdBNazTPv8LpSP4x0GQlI'
        SECRET_KEY = 'nW8grONY777n4I2KvpOVuKGDNiY03omI'
        self.client_face = AipFace(APP_ID, API_KEY, SECRET_KEY)
        
        self.bridge = CvBridge()
        self.image_type = "BASE64"
        self.filepath = "/home/dell/catkin_ws/src/Whereisthis/whereisthis_image/pictures/image.jpg"
        self.nowtask = 0

        self.guest_list = ['0','1','2','3']
        self.groupIdList = 'whereisthis_group'
        self.g1 = user()
        self.g1.user_id = '1'
        self.g1.facenum = 0
        self.g2 = user()
        self.g2.user_id = '2'
        self.g2.facenum = 0
        self.g3 = user()
        self.g3.user_id = '3'
        self.g3.facenum = 0
        self.new = [0, 0, 0, 0]
        self.old = [0, 0, 0, 0]

        self.options_face = {}
        self.options_face["face_field"] = "age,gender"
        self.options_face["max_face_num"] = 3
        self.options_face["face_type"] = "LIVE"

        # 如果人脸库之前有人脸建议先清掉数据,没有就忽略
        self.clear_data(flag = True)
        #发布器
        self.guest = rospy.Publisher('/image/findperson', Int8, queue_size=1)
        #订阅器
        # rospy.Subscriber("/camera/rgb/image_raw", Image, self.imgCallback, queue_size = 1)
        rospy.Subscriber("/image_rate_transfer/image_raw", Image, self.imgCallback, queue_size = 1)
        rospy.Subscriber('/whereisthis_control', whereisthis, self.controlCallback)

    def clear_data(self, flag):

        rospy.loginfo('deleting data from cloud...')
        # groupid: self.goupIdList    userid: '1' or '2' or '3'
        self.client_face.deleteUser(self.groupIdList, '1')
        self.client_face.deleteUser(self.groupIdList, '2')
        self.client_face.deleteUser(self.groupIdList, '3')
        rospy.loginfo('complete.')


    def controlCallback(self, msg):

        if msg.FinishState == False:
            self.nowtask = msg.NowTask


    def imgCallback(self, image_msg):

        t = whereisthis()
        if self.nowtask == t.GoalDescription:
            rospy.loginfo('imgCallback working...')
            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as e:
                print(e)
            self.add_user(self.filepath, cv_image)


    # 打开文件
    def fileopen(self, filepath):

        with open(filepath, 'rb') as fp:
            imgjson = base64.b64encode(fp.read())
            data = str(imgjson).encode('utf-8')
            return data


    # 判断人脸已知
    def judger(self ,result):

        if result['error_code'] == 0:
            result_sc = result['result']['user_list'][0]['score']
            # print(result_sc,type(result_sc))
            if result_sc > 80:
                return result['result']['user_list'][0]['user_id']
                # return True
            else:
                return 0
        else:
            return 0
        

    # 人脸库搜索   groupIdList="用户组名称"
    def face_search(self, filepath, groupid='whereisthis_group'):  

        image = self.fileopen(filepath)
        imageType = "BASE64"
        detect = self.client_face.detect(image, self.image_type, self.options_face)
        if detect:
            result = self.client_face.search(image, imageType, groupid)
            # print(result)  # 打印出所有信息
            return result
        else:
            return None
            # return self.judger(result)


    def face_add(self, filepath, image_msg, groupid, userid):  # 人脸库增加 地址 组 用户

        image = self.fileopen(filepath)
        imageType = "BASE64"
        result = self.client_face.addUser(image, imageType, groupid, userid)
        if result['error_code'] == 0:
            print("增加人脸成功")
        else:
            print("增加人脸失败")
            # print(result)


    def get_user_facenum(self, userid):

        if userid == '1':
            return self.g1
        elif userid == '2':
            return self.g2
        elif userid == '3':
            return self.g3
        else:
            rospy.loginfo('there is no user id that matches.')
            # TODO: return what


    def add_user(self, file_name, image_msg):

        cv2.imwrite(file_name, image_msg)
        
        search_result = self.face_search(file_name, 'whereisthis_group')  #id
        curusr = user()
        
        # 没人脸
        if search_result == None:
            return

        # 加入新人脸
        if search_result['error_code'] == 222207:
            userid = '1'
            self.g1.facenum += 1
            curusr = self.g1
            which = self.guest_list.index(userid)
            t = Int8()
            t.data = which
            self.guest.publish(t)
            rospy.loginfo('this is a new face: 222207')

        elif  search_result['error_code'] == 0 :
            if search_result['result']['user_list'][0]['score'] < 80:
                # ---------------------------------------
                num = len(search_result['result']['user_list'][0]['user_id'])
                if num == 0:
                    userid = '1'
                    self.g1.facenum += 1
                    curusr = self.g1
                elif num == 1:
                    userid = '2'
                    self.g2.facenum += 1
                    curusr = self.g2
                elif num == 2:
                    userid = '3'
                    self.g3.facenum += 1
                    curusr = self.g3
                else:
                    rospy.loginfo('wrong with num')
                    # TODO: what's next? 

                # publish guest id
                which = self.guest_list.index(userid)
                t = Int8()
                t.data = which
                self.guest.publish(t)
                rospy.loginfo('this is a new face: 0')

            # 旧人脸
            else:
                userid = search_result['result']['user_list'][0]['user_id']
                curusr = self.get_user_facenum(userid)
                which = self.guest_list.index(userid)
                t = Int8()
                t.data = which
                self.guest.publish(t)
                rospy.loginfo('this is an old face.')

        # 加入人脸张数信息 未达到上限20则持续添加，否则不作处理
        if curusr.facenum < 20 and curusr.user_id:
            self.face_add(file_name, image_msg, 'whereisthis_group', curusr.user_id)
            os.system('rosnode kill /whereisthis_image')
            # print(curusr.user_id , curusr.facenum)
        else:
            print('warning: no face detected. will keep detecting.')
            print('--------------------------')


if __name__ =='__main__':

    print('whereisthis_image node init...')  
    Guest_Recognition()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("whereisthis_image node terminated.")





# 基本思路
# 该有的  
# add search 
