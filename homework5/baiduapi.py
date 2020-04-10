#!/usr/bin/env python
# -*- coding: utf-8 -*-
# python3

#ros
import sys, os, rospy
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

#python
import numpy
ros_path = '/home/dell/.local/lib/python3.5/site-packages'
sys.path.append('/home/dell/.local/lib/python3.5/site-packages')
from aip import AipFace
from aip import AipBodyAnalysis
import matplotlib.pyplot as plt
import math
import base64
import cv2
import base64
from PIL import Image
from io import BytesIO


def frame2base64(frame):
    img = Image.fromarray(frame)  # 将每一帧转为Image
    output_buffer = BytesIO()  # 创建一个BytesIO
    img.save(output_buffer, format='JPEG')  # 写入output_buffer
    byte_data = output_buffer.getvalue()  # 在内存中读取
    base64_data = base64.b64encode(byte_data)  # 转为BASE64
    return base64_data  # 转码成功 返回base64编码
def generate():
    camera = cv2.VideoCapture(0)
    try:
        while True:
            ret, frame = camera.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.imshow("camera", frame)
            base64_data = frame2base64(frame)
            print(base64_data)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    except Exception as e:
        print(e)
    finally:
        # 释放资源
        camera.release()
        cv2.destroyAllWindows()
#APP_ID = '18721308'
#API_KEY = 'lNQGdBNazTPv8LpSP4x0GQlI'
#SECRET_KEY = 'nW8grONY777n4I2KvpOVuKGDNiY03omI'
APP_ID = '19361611'
API_KEY = 'oUxDwxsqvlRWMn7LNWtdBKC3'
SECRET_KEY = '6HHHfOZPyFP3eTycu6EGTbwd3G59A0UK'
# 初始化AipFace对象
aipFace = AipFace(APP_ID, API_KEY, SECRET_KEY)
client = AipBodyAnalysis(APP_ID, API_KEY, SECRET_KEY)
imageType = "BASE64"
# 定义参数变量
options = {}
options["face_field"] = "age"
options["max_face_num"] = 10
options["face_type"] = "LIVE"

img2=cv2.imread("/home/dell/img/11.png")

bodyoptions = {}
bodyoptions["type"] = '''gender,age,lower_wear,headwear,glasses,upper_color,lower_color,upper_wear_fg,
,upper_wear_texture,bag,smoke,vehicle,carrying_item,cellphone,umbrella'''


# 调用人脸属性检测接口
# 读取原图
cap = cv2.VideoCapture(0)
# while True:
num_pic=0
#cap = cv2.VideoCapture("/home/dell/img/man.mp4")#打开视频
while cap.isOpened():
    ret, img = cap.read()
    #x = img.tobytes()
    #img2 = cv2.imdecode(numpy.fromstring(x, numpy.uint8),1 )
    base64_data = frame2base64(img)
    image = str(base64_data)
    result = aipFace.detect(image, imageType, options)
    #result2 = client.bodyAttr(img2,bodyoptions)
    print(result)
    #print(result2)
    # print(type(result))
    # 解析位置信息
    face_num = result['result']['face_num']
    for num in range(0, int(face_num)):
        print(num)
        location = result['result']['face_list'][num - 1]['location']
        age=result['result']['face_list'][num - 1]['age']
        # print(location)
        # print(location['face_list'][0])
        Theta = location['rotation'] / 60  
        A = (int(location['left']), int(location['top']))
        B = (int(location['left']) ,
             int(location['top']) + int(location['height']))
        AC_Len = math.sqrt(location['width'] ** 2 + location['height'] ** 2)
        AC_Theta = math.atan(location['height'] / location['width']) + location['rotation'] / 60  ####或者是？？？
        C = (
            int(location['left']) + int(location['width']),
            int(location['top']) )
        D = (int(location['left']) + int(location['width']),
             int(location['top']) + int(location['height']))
        cv2.line(img, A, B, (0, 0, 255), 2)
        cv2.line(img, A, C, (0, 0, 255), 2)
        cv2.line(img, B, D, (0, 0, 255), 2)
        cv2.line(img, D, C, (0, 0, 255), 2)
        cv2.waitKey(260)
        cv2.putText(img, "age"+str(age), C, cv2.FONT_HERSHEY_COMPLEX, 1.0, (100, 200, 200), 1)
        cv2.putText(img, "people_num:"+str(face_num), (0,50), cv2.FONT_HERSHEY_COMPLEX, 1.0, (100, 200, 200), 2)
    cv2.imshow('img', img)
    
cv2.waitKey(1)



# import requests
# import base64
#
# '''
# 人体检测和属性识别
# '''
#
# request_url = "https://aip.baidubce.com/rest/2.0/image-classify/v1/body_attr"
#
# # 二进制方式打开图片文件
# f = open('[本地文件]', 'rb')
# img = base64.b64encode(f.read())
#
# params = {"image":img}
# access_token = '[调用鉴权接口获取的token]'
# request_url = request_url + "?access_token=" + access_token
# headers = {'content-type': 'application/x-www-form-urlencoded'}
# response = requests.post(request_url, data=params, headers=headers)
# if response:
#     print (response.json())

