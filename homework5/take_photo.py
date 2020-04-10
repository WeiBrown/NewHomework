#!/usr/bin/env python
# -*- coding: UTF-8 -*- 


# author: Tianyi Li
# time: 2020/3/19
# 使用百度API进行关键点视觉处理
# 实际上可以使用openpose，使用同方法替代，但是其算力要求过高。
# 现在是检测肘，腕，肩三个位置判断。
# 发布消息：
#   视野中目标的位置
#   带有目标的图像


# 视觉处理
import cv2

# 基本类型
import numpy as np
import time
import base64
import matplotlib.pyplot as plt
import base64
import copy



# ROS

import rospy
import roslib
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Int8
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Pose

from speech.msg import Description

import sys
sys.path.append('/home/dell/.local/lib/python3.5/site-packages')
from aip import AipFace
from aip import AipBodyAnalysis

class BodyCheck:
    def __init__(self):
        self.time = time.time()
        APP_ID = '18889374'
        API_KEY = 'pUNweNaSK4rWz57vGs9KpuW1'
        SECRET_KEY = 'ru5LqWM0lrcVYBh9cjd32fy951nagqcA'
        self.imageType = "BASE64"
        self.client_face = AipFace(APP_ID, API_KEY, SECRET_KEY)
        self.client_body = AipBodyAnalysis(APP_ID, API_KEY, SECRET_KEY)
        self.bridge = CvBridge()
        ##############人类数据
        self.filepath = "/home/dell/img/"
        self.option_face = {}
        self.option_body = {}
        self.option_face["face_field"] = "age,gender,glasses,race"
        self.option_face["max_face_num"] = 1
        self.option_body["type"] = "upper_wear,upper_color"


        ##############跟踪数据
        self.roi = None

        ##############话题名称
        # 接收器与发布器
        self.sub_image_name      = rospy.get_param('~image_raw_topic_name',          '/usb_cam/image_raw')
        # 发布器
        self.pub_pos_name        = rospy.get_param('~object_view_topic_name',        'roi')
        self.pub_img_name        = rospy.get_param('~image_test_topic_name',         '/image/test')
        self.pub_fet_name        = rospy.get_param('~feature_topic_name',            '/image/feature')
        ##############发布器
        self.img_pub = rospy.Publisher(self.pub_img_name, Image)
        self.roi_pub = rospy.Publisher(self.pub_pos_name, RegionOfInterest)
        self.fet_pub = rospy.Publisher(self.pub_fet_name, Description)

        self.img_sub = rospy.Subscriber(self.sub_image_name, Image, self.imgCallback)
        print("============================================================")

    # 人体数据转换
    def msgtobody(self, image_msg, file_name='image_body.png'):
        # 转为base64存储
        cv2.imwrite(self.filepath+file_name, image_msg)
        with open(self.filepath+file_name, 'rb') as fp:
            return fp.read()

    # 人脸数据转换
    def msgtoface(self,image_msg, file_name='image_faces.png'):
        cv2.imwrite(self.filepath+file_name, image_msg)
        with open(self.filepath+file_name, 'rb') as fp:
            data = base64.b64encode(fp.read())
            # python2.7
            data = str(data).encode('utf-8')
            return data

    # 挥手检测，返回一个挥手的人的方框xxyy数据
    def detectWave(self, image, gender=False):
        print("============================================================")
        print("CHECK")
        data = self.msgtobody(image, "image_body.png")
        # ----------挥手检测----------
        result = self.client_body.bodyAnalysis(data)
        wave = []
        loaction = []
        # 存在人
        if result['person_num'] > 0:
            id_ = -1
            # 对每个人进行检查
            for info in result['person_info']:
                id_+=1
                keypoint = info['body_parts']
                # 腕高
                if keypoint['right_elbow']['y'] > keypoint['right_wrist']['y']:
                    # 腕在外侧
                    if keypoint['right_wrist']['x'] < keypoint['right_shoulder']['x']:
                        wave.append(id_)
                        loc = []
                        loc.append(int(info['location']['left']))
                        loc.append(int(info['location']['left'] + info['location']['width']))
                        loc.append(int(info['location']['top']))
                        loc.append(int(info['location']['top']  + info['location']['height']))
                        loaction.append(copy.deepcopy(loc))
                # 腕高
                elif keypoint['left_elbow']['y'] > keypoint['left_wrist']['y']:
                    # 腕在外侧
                    if keypoint['left_wrist']['x'] > keypoint['left_shoulder']['x']:
                        wave.append(id_)
                        loc = []
                        loc.append(int(info['location']['left']))
                        loc.append(int(info['location']['left'] + info['location']['width']))
                        loc.append(int(info['location']['top']))
                        loc.append(int(info['location']['top']  + info['location']['height']))
                        loaction.append(copy.deepcopy(loc))
            if len(loaction) > 0:
                # 返回挥手的随机一个人的位置
                locate = loaction[0]
                loc = []
                loc.append(locate[0])
                loc.append(locate[1])
                loc.append(locate[2])
                loc.append(locate[3])
                return locate
        return None

    # 照片的回调函数，发布挥手人的位置
    def imgCallback(self, image):
        #try:
            #cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        #except CvBridgeError as e:
            #print(e)

        cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        position = self.detectWave(cv_image)
        if position is not None:
            msg = Description()
            msg.hair_style = "unknowm"
            msg.pose = "unknowm"
            # 特征检测——人脸和人体
            img_body = cv_image[position[2]:position[3],position[0]:position[1]]
            face = self.msgtoface(img_body)
            result1 = self.client_face.detect(face, self.imageType, self.option_face)
            print(result1["result"]["face_list"][0]["gender"]["type"]) 
            print(result1["result"]["face_list"][0]["glasses"]["type"]) 
            print(result1["result"]["face_list"][0]["race"]["type"]) 
            print(str(result1["result"]["face_list"][0]["age"]) )
            data = result1["result"]["face_list"][0]
            # 性别 + 眼睛 + 肤色 + 年龄
            msg.gender = data["gender"]["type"]
            msg.glasses = data["glasses"]["type"]
            msg.skin_color = data["race"]["type"]
            msg.age = str(data["age"])

            # 颜色 + 服装
            body = self.msgtobody(img_body)
            result2 = self.client_body.bodyAttr(body, self.option_body)
            print(result2["person_info"][0]["attributes"]["upper_wear"]["name"])
            data = result2["person_info"][0]["attributes"]
            # 红、橙、黄、绿、蓝、紫、粉、黑、白、灰、棕
            color =  data["upper_color"]["name"]
            if color == "红":
                msg.clothes_color = "red"
            elif color == "橙":
                msg.clothes_color = "orange"
            elif color == "黄":
                msg.clothes_color = "yellow"
            elif color == "绿":
                msg.clothes_color = "green"
            elif color == "蓝":
                msg.clothes_color = "blue"
            elif color == "紫":
                msg.clothes_color = "purple"
            elif color == "粉":
                msg.clothes_color = "pink"
            elif color == "黑":
                msg.clothes_color = "black"
            elif color == "白":
                msg.clothes_color = "white"
            elif color == "灰":
                msg.clothes_color = "gray"
            else:
                msg.clothes_color = "brown"
            
            type_ = data["upper_wear"]["name"]
            if type_ == "长袖":
                msg.clothes = "Coat"
            else:
                msg.clothes = "Short"
            self.fet_pub.publish(msg)
            
        # 如果存在人
        try:
            cv2.rectangle(cv_image, (position[1],position[3]),(position[0],position[2]),(0,0,255))
            roi = RegionOfInterest()
            roi.x_offset = position[0]
            roi.y_offset = position[2]
            roi.width = position[1] - position[0]
            roi.height = position[3] - position[2]
            self.roi = roi
            self.roi_pub.publish(roi)
            print("One Wave!")
        except:
            pass
        cv_image = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.img_pub.publish(cv_image)
    
if __name__ == "__main__":
    rospy.init_node('wave_detect', anonymous=True)
    body = BodyCheck()
    #img=cv2.imread("/home/dell/11.png")
    #body.detectWave(img)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("CLOSE TAKING PHOTO")
