#!/usr/bin/python2

import rospy                      # rospy
import numpy as np                # numpy
import cv2                        # OpenCV2
from sensor_msgs.msg import Image # ROS Image message
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter

#Instantiate CV Bridge
bridge = CvBridge()

def image_callback(msg):
    print("PyImageSubscriber node  Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError,e:
        print(e)
    else:
        # Display the converted image
        cv2.imshow("Image Display", cv2_img)
        # Wait 30 ms to allow image to be drawn.
        # Image won't display properly without this cv2.waitkey
        cv2.waitKey(30) 
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', cv2_img)

    
def image_listener():
    # Initiate the node
    rospy.init_node('py_image_listener')
    # Setupt the subscription, camera/rb/image_raw is used in turtlebot_gazebo example
    rospy.Subscriber("jetbot_camera/raw", Image, image_callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    cv2.destroyWindow("Image Display")

if __name__ == '__main__':
    image_listener()
