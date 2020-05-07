# !/usr/bin/env python   
# 윗줄은 shebang line 이 프로그램이 실행될 파이썬 인터프리터를 가리킴
# env 세팅 혹은 venv을 이용하면 한기기 여러환경에서 여러 프로그램을 실행 가능할듯

import rospy    # 파이썬 기반
from std_msgs.msg import Int32

rospy.init_node('topic_publisher')

pub = rospy.Publisher('counter', Int32)

rate = rospy.Rate(2)

count = 0

while not rospy.is_shutdown():
    pub.publish(count)
    count += 1
    rate.sleep()

# 디렉터리 수정
