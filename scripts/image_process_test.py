#!/usr/bin/python2
#-*- coding:utf-8 -*-


import rospy                      # rospy
import numpy as np                # numpy
import cv2                        # OpenCV2
from sensor_msgs.msg import Image # ROS Image message
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
import math

#Instantiate CV Bridge
bridge = CvBridge()

Matrix = np.array([[991.65780078, 0., 451.37431919], [0., 940.49570204, 163.20240618], [0., 0., 1.]], dtype=np.float32)
Distortion = np.array([[-0.24267411, 0.04755508, 0.00706694, 0.01001887, -0.00422284]], dtype=np.float32)

vertices = ((222, 250), (124, 480), (587, 480), (480, 250))
src = np.float32(vertices)
dst = np.float32([[210, 0], [210, 480], [640 - 210, 480], [640 - 210, 0]])
mini_histo_roi = np.float32([[150, 400], [150, 480], [460, 480], [460, 400]])
warped_roi = np.float32([[0, 0], [150, 480], [460, 480], [640, 0]])


def distort(image):
    undist = cv2.undistort(image, Matrix, Distortion, None, Matrix)
    return undist


def resize(image, size):
    return cv2.resize(image, dsize=size)


def ready_process(image):
    distorted_img = distort(image)
    resized_img = resize(distorted_img, (320, 240))
    return resized_img


def warp(image, source, destination):
    # Compute and apply perpective transform
    img_size = (image.shape[1], image.shape[0])
    m = cv2.getPerspectiveTransform(source, destination)
    warped_img = cv2.warpPerspective(image, m, img_size, flags=cv2.INTER_NEAREST)  # keep same size as input image
    return warped_img


def bird_view(image):
    h, w = image.shape[:2]
    gap = 80
    bird_src = np.float32([[102, 105], [17, 240], [272, 240], [209, 107]])  # 새로 수정할거
    bird_dst = np.float32([[w/2 - gap, 0], [w/2 - gap, h], [w/2 + gap, h], [w/2 + gap, 0]])
    return cv2.bitwise_not(warp(cv2.bitwise_not(image), bird_src, bird_dst))


def mini_histo(gray, level):
    h, w = gray.shape[:2]
    result = []
    roi_length = 50
    last_level = 30
    for x in range(w):
        sum = 0
        for y in range(h - last_level- roi_length, h - last_level):
            if gray[y, x] < level:
                sum += 1
        result.append(sum)

    return result


def sobel_filter(warped_img):
    gray_warped = np.where(warped_img > 250, 160, warped_img)
    sobel_x = cv2.Sobel(gray_warped, cv2.CV_8U, 1, 0, ksize=3)
    return sobel_x


def hough_line(sobel_x):

    minLineLength = 1
    maxLineGap = 10
    _, threshold = cv2.threshold(sobel_x, 150, 255, cv2.THRESH_BINARY)

    lines = cv2.HoughLinesP(threshold, 1, np.pi / 360, 80, minLineLength, maxLineGap)
    return lines


def find_angle(gray):
    resized = ready_process(gray)
    warped = bird_view(resized)
    sobel_X = sobel_filter(warped)
    lines = hough_line(sobel_X)

    if lines is not None:
            angle_list = []
            for line in lines:
                [[x1, y1, x2, y2]] = line
                # cv2.line(warped, (x1, y1), (x2, y2), (0, 0, 255), 3)
                if (x1 - x2) is not 0:
                    angle = np.arctan((y2 - y1) / (x2 - x1))
                else:
                    angle = np.pi/2
                if angle > 0 and (np.pi - angle) > 0.1:
                    angle_list.append(angle)
                else:
                    pass
            sum = np.sum(angle_list, axis=0)
            print(angle_list)
            average_angle = sum / len(angle_list)
            # try:
            #     dx = math.cos(average_angle)
            #     try:
            #         dy = math.sin(average_angle)
            #         cv2.line(warped, (160, 240), (int(160 - 30 * dx), int(240 - 30 * dy)), (255, 0, 0))
            #     except:
            #         print('error in this angle to sin =', average_angle)
            # except:
            #     print('error in this angle to cos =', average_angle)
    return average_angle


def image_callback(msg):
    # print("PyImageSubscriber node  Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        b_img = cv2.cvtColor(cv2_img,cv2.COLOR_BGR2GRAY)
        
        # Display the converted image
        # cv2.imshow("Image Display", cv2_img)
        average_angle = find_angle(b_img)
        print("Average_angle =", average_angle)
        #cv2.imshow("Gray Image Display", b_img)

        # road_angle = find_angle(b_img)

        # Wait 30 ms to allow image to be drawn.
        # Image won't display properly without this cv2.waitkey
        cv2.waitKey(30) 
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', cv2_img)

    
def image_listener():
    # Initiate the node
    rospy.init_node('py_image_listener')
    # Setupt the subscription, camera/rb/image_raw is used in turtlebot_gazebo example
    #rospy.Subscriber("jetbot_camera/raw", Image, image_callback)
    rospy.Subscriber("/video_publisher/image_raw_bgr8", Image, image_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    cv2.destroyWindow("Image Display")

if __name__ == '__main__':
    image_listener()
