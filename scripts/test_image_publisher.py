#!/usr/bin/python2
#-*- coding:utf-8 -*-
# 윗줄은 shebang line 이 프로그램이 실행될 파이썬 인터프리터를 가리킴
# env 세팅 혹은 venv을 이용하면 한기기 여러환경에서 여러 프로그램을 인터프리터 사용 가능할듯
# 파이썬 2 쓰도록 바꾸고 인코딩 utf-8로 변경

import rospy    # 파이썬 기반
import sys
from sensor_msgs.msg import Image

rospy.init_node('test_image_publisher')

pub = rospy.Publisher('test_image/raw'', Image)

rate = rospy.Rate(2)

count = 0

while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()

# 디렉터리 수정
