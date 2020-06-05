#!/usr/bin/python2

import rospy
from std_msgs.msg import Float32MultiArray
def callback(msg):
    print(msg.data)

rospy.init_node('test_topic_sub')

sub = rospy.Subscriber('multiple array', Float32MultiArray, callback)

rospy.spin()