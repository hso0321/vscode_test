#!/usr/bin/python2
#-*- coding:utf-8 -*-
# 윗줄은 shebang line 이 프로그램이 실행될 파이썬 인터프리터를 가리킴
# env 세팅 혹은 venv을 이용하면 한기기 여러환경에서 여러 인터프리터를 사용 가능할듯
# 파이썬 2 쓰도록 바꾸고 인코딩 utf-8로 변경

import rospy    # 파이썬 기반
from std_msgs.msg import Float32MultiArray

rospy.init_node('test_topic_pub')

pub = rospy.Publisher('multiple array', Float32MultiArray)

rate = rospy.Rate(2)

count = [0, 1, 2, 3, 4]

while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()

# 디렉터리 수정
