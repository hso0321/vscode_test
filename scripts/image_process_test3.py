#!/usr/bin/python2
#-*- coding:utf-8 -*-

import rospy                      # rospy
import numpy as np                # numpy
import cv2                        # OpenCV2
from sensor_msgs.msg import Image # ROS Image message
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError # ROS Image message -> OpenCV2 image converter
import math

#Instantiate CV Bridge
bridge = CvBridge()

vertices = [(0, 0), (0, 115), (72, 240), (268, 240), (320, 152), (320, 0)]
src = np.float32(((222, 250), (124, 480), (587, 480), (480, 250)))
dst = np.float32([[210, 0], [210, 480], [640 - 210, 480], [640 - 210, 0]])
mini_histo_roi = np.float32([[150, 400], [150, 480], [460, 480], [460, 400]])
warped_roi = np.float32([[0, 0], [150, 480], [460, 480], [640, 0]])

Matrix = np.array([[702.79573591, 0., 639.68721137], [0., 700.36468219, 351.49375006], [0., 0., 1.]], dtype=np.float32)
Distortion = np.array([[-0.24512458, -0.09374123, -0.01451455, -0.00190292, 0.22375519]], dtype=np.float32)

Blue = (255, 0, 0)
Green = (0, 255, 0)
Red = (0, 0, 255)


class Line:
    def __init__(self):
        # was the line detected in the last iteration?
        self.detected = False
        # Set the width of the windows +/- margin
        self.window_margin = 20
        # x values of the fitted line over the last n iterations
        self.prevx = []
        # polynomial coefficients for the most recent fit
        self.current_fit = [np.array([False])]
        #radius of curvature of the line in some units
        
        # starting x_value
        # 히스토그램을 만드냐 만들지 않느냐 판단기준
        self.startx = None
        # ending x_value
        self.endx = None
        # x values for detected line pixels
        self.allx = None
        # y values for detected line pixels
        self.ally = None
        # road information
        self.road_inf = None
        self.num_window = 9
        self.min_pixel = 50
        self.counter = 0

left_line = Line()
right_line = Line()

msg_data = [0, 0, 0, 0]


def distort(img):
    return cv2.undistort(img, Matrix, Distortion, None, Matrix)


def resize(image, size):
    return cv2.resize(image, dsize=size)


def ready_process(image):
    distorted_img = distort(image)
    resized_img = resize(distorted_img, (320, 240))
    return cv2.rotate(resized_img, cv2.ROTATE_180)


def warp(image, source, destination):
    # Compute and apply perpective transform
    img_size = (image.shape[1], image.shape[0])
    m = cv2.getPerspectiveTransform(source, destination)
    return cv2.warpPerspective(image, m, img_size, flags=cv2.INTER_NEAREST)  # keep same size as input image



# @logging_time
def bird_view(image):
    h, w = image.shape[:2]
    gap = 80
    bird_src = np.float32([[105, 105], [40, 240], [285, 240], [220, 105]])
    bird_dst = np.float32([[w/2 - gap, 0], [w/2 - gap, h], [w/2 + gap, h], [w/2 + gap, 0]])
    # return cv2.bitwise_not(warp(cv2.bitwise_not(image), bird_src, bird_dst))
    return warp(image, bird_src, bird_dst)

def lab_combine(img): #, th_h, th_l, th_s):
    #   convert to hls color space
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)

    L = lab[:, :, 0]
    A = lab[:, :, 1]
    B = lab[:, :, 2]

    filtered_51l = cv2.medianBlur(L, ksize=51)
    light = np.uint8(0.8*np.double(filtered_51l))

    inverse_l = cv2.bitwise_not(L)
    middle_process = cv2.add(inverse_l, filtered_51l)
    process_51l = cv2.bitwise_not(middle_process)

    blur_a = cv2.GaussianBlur(A, (3, 3), 0)
    _, masked_red_a = cv2.threshold(blur_a, 160, 255, cv2.THRESH_BINARY)
    process_red = np.uint8(0.5 * np.double(masked_red_a))
    single_line = cv2.add(process_51l, process_red)
    # _, single_line_thresh = cv2.threshold(single_line, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    _, single_line_thresh = cv2.threshold(single_line, 60, 255, cv2.THRESH_BINARY)
    # test = cv2.add(cv2.bitwise_not(cv2.add(inverse_l, light)), process_red)
    # _, test_thresh = cv2.threshold(test, 70, 255, cv2.THRESH_BINARY)
    # _, test_thresh2 = cv2.threshold(test, 0, 255, cv2.THRESH_BINARY+cv2.THRESH_OTSU)
    test_blur = cv2.GaussianBlur(single_line, (3, 3), 0)
    _, test_thresh = cv2.threshold(test_blur, 70, 255, cv2.THRESH_BINARY)


    return test_thresh


# .detected로 시작해서, startx를 구하고 currentx를 할당한뒤, .detected 갱신하고 .startx에 결과저장
def find_first(b_img, left_line, right_line):
    output = cv2.cvtColor(b_img, cv2.COLOR_GRAY2RGB)
    print('left detect =', left_line.detected, 'right detect =', right_line.detected)
    # 둘다 없을 때

    if (left_line.detected == False) and (right_line.detected == False):
        # Take a histogram of the bottom half of the image
        histogram = np.sum(b_img[184:225, 47:270], axis=0)      # 184~225 높이에 대한 (가로 47 ~ 269 측정값) 히스토그램 제작
        midpoint = np.int(159 - 47)
        cv2.rectangle(output, (47, 184), (270, 225), (0, 255, 255), 1)

        # in histogram, index = real proint - 47 (47 ~ 270 vs 0 ~ 320)
        start_leftX = np.argmax(histogram[:midpoint]) + 47
        start_rightX = np.argmax(histogram[midpoint:]) + midpoint + 47

        if histogram[start_leftX - 47] > 1000:
            current_leftX = start_leftX
        else:
            current_leftX = None
            # left_line.startx = None
            # detected는 여전히 false

        if histogram[start_rightX - 47] > 1000:
            current_rightX = start_rightX
        else:
            current_rightX = None
            # right_line.startx = None



    # 왼쪽만 없을 때
    elif (left_line.detected == False) and (right_line.detected == True):
        histogram_l = np.sum(b_img[184:225, 47:159], axis=0)      
        start_leftX = np.argmax(histogram_l) + 47
        cv2.rectangle(output, (47, 184), (159, 225), (0, 255, 255), 1)


        if histogram_l[start_leftX - 47] > 1000:
            current_leftX = start_leftX
        else:
            current_leftX = None
            left_line.startx = None
        current_rightX = right_line.startx  # 왼쪽 구했으니 오른쪽은 가져옴
        


    # 오른쪽만 없을 때
    elif (left_line.detected == True) and (right_line.detected == False):
        histogram_r = np.sum(b_img[184:225, 159:270], axis=0)      
        start_rightX = np.argmax(histogram_r) + 159
        cv2.rectangle(output, (159, 184), (270, 225), (0, 255, 255), 1)


        if histogram_r[start_rightX - 159] > 1000:
            current_rightX = start_rightX
        else:
            current_rightX = None
            right_line.startx = None
        current_leftX = left_line.startx 


    # 세가지 경우에 대해서, 히스토그램으로 구한 startx로 두가지 currentx 할당 -> None or x좌표



    # 160 근처에서 겹치는 시작점들을 분류
    if (current_leftX is not None) and (current_rightX is not None):
        # print('start lx =', start_leftX, 'start rx =', start_rightX)
        if abs(current_leftX - current_rightX) <= 25:
            histogram_m = np.sum(b_img[184:255, 134:184])       # 159를 기준으로 +-25픽셀

            # 비슷한것 중 히스토그램 값이 작은 것들을 삭제해서 정리
            try:
                if histogram_m[159 - current_leftX] >= histogram_m[current_rightX - 159 + 25]: 
                    current_rightX = None       # 왼쪽으로 갈거같으면, 오른쪽 삭제
                    right_line.startx = None

                else:       # histogram_m[159 - start_leftX] < histogram_m[start_rightX - 159 + 25]:    
                    current_leftX = None        # 오른쪽으로 갈거같으면, 왼쪽 삭제
                    left_line.startx = None
            except:
                print('error current_leftX =', current_leftX, 'current_rightX =', current_rightX)
    
    # 테스트용
    print('left line current  =', current_leftX, 'right line current =', current_rightX)


    # startx와 detected 할당
    if current_leftX is not None:
        left_line.startx = current_leftX
        left_line.detected = True
        cv2.circle(output, (int(current_leftX), 205), 10, Red, -1)                  # 빨간점
        # print('work')
        # print('left line detect =', left_line.detected)                 
    else:
        left_line.startx = None
        left_line.detected = False
        # print('sleep')

    if current_rightX is not None: 
        right_line.startx = current_rightX
        right_line.detected = True
        cv2.circle(output, (int(current_rightX), 205), 10, (0, 100, 255), -1)      # 주황점
        print('right line find!')
    else:
        right_line.startx = None
        right_line.detected = False


    # 결과 startx 를 구해서 startx를 잡아주고, currentx가 확실하면 .startx에 저장하고 .detected도 갱신
    return output


def drawline(output, left_fit, right_fit):
    ploty = np.linspace(0, output.shape[0] - 1, output.shape[0])

    if left_fit is not None:
        left_plotx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
        
    
    
    if right_fit is not None:
        right_plotx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]


    return output


def sliding_window(b_img, output, left_line, right_line, window_height):
    
    nonzero = b_img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    # Set minimum number of pixels found to recenter window
    min_num_pixel = left_line.min_pixel
    num_windows = left_line.num_window
    # Create empty lists to receive left and right lane pixel indices       # 한 프레임마다 저장 될 빈 리스트 생성
    win_left_lane = []
    win_right_lane = []

    left_weight_x, left_weight_y = [], []
    right_weight_x, right_weight_y = [], []
    window_margin = left_line.window_margin         # margin = 20

    current_leftX = left_line.startx
    current_rightX = right_line.startx

    # TODO 윈도우 맨위 맨아래 하나씩 줄여보기 
    # Step through the windows one by one
    for window in range(num_windows):           # 윈도우 마다 사각형 그리고, 안쪽 선으로 추정되는 픽셀 nonzero 인덱스 추출

        win_y_low = b_img.shape[0] - (window + 1) * window_height         # 위쪽이 더 작은 값임을 생각
        win_y_high = b_img.shape[0] - window * window_height
        new_high = int(0.7 * win_y_low + 0.3 * win_y_high)

        if current_leftX is not None:                                   # 시작점이 있으면 윈도우 정보(좌우 좌표값) 생성
            win_leftx_min = int(current_leftX - window_margin)
            win_leftx_max = int(current_leftX + window_margin)
            if win_leftx_min < 0:
                win_leftx_min = 0
                win_leftx_max = 0 + 2 * window_margin

            cv2.rectangle(output, (win_leftx_min, win_y_low), (win_leftx_max, win_y_high), (0, 255, 0), 2)              # 사각형 그림

            left_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= new_high) & (nonzerox >= win_leftx_min) & (
                        nonzerox <= win_leftx_max)).nonzero()[0]

            win_left_lane.append(left_window_inds)                          
            num_left_inds = len(left_window_inds)
        

        if current_rightX is not None:
            win_rightx_min = int(current_rightX - window_margin)
            win_rightx_max = int(current_rightX + window_margin)
            if win_rightx_max > b_img.shape[1]:
                win_rightx_min = b_img.shape[1] - 2 * window_margin
                win_rightx_max = b_img.shape[1]
            cv2.rectangle(output, (win_rightx_min, win_y_low), (win_rightx_max, win_y_high), (30, 200, 0), 2)
            
            # Identify the nonzero pixels in x and y within the window
            right_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= new_high) & (nonzerox >= win_rightx_min) & (
                nonzerox <= win_rightx_max)).nonzero()[0]

            # Append these indices to the lists
            # 한 프레임에서만 계속 보관하게되어있었다
            win_right_lane.append(right_window_inds)
            num_right_inds = len(right_window_inds)


        # currnet X 아직 안건듬

        
   
        # If you found > minpix pixels, recenter next window on their mean position
        # current x를 이동시키는 과정

        if current_leftX is not None:                                                               
            if num_left_inds > min_num_pixel:     # 선이랑 윈도우랑 좀 겹쳐있으면
                # TODO 여기서 win_y_high 에서의 x값의 평균을 구하고자 한다.

                current_leftX = np.int(np.mean(nonzerox[left_window_inds]))
                # 윈도우 평균을 계산 후 노란점
                # cv2.circle(output, (int(current_leftX), int((win_y_low + win_y_high) / 2)), 5, (0, 100, 100), -1)
                cv2.circle(output, (int(current_leftX), int((win_y_low + new_high) / 2)), 5, (0, 100, 100), -1)

                left_weight_x.append(int(current_leftX))
                # left_weight_y.append(int((win_y_low + win_y_high) / 2)) 
                left_weight_y.append(int((win_y_low + new_high) / 2)) 

                # 윈도우 하얀애들 평균을 current x에 두고 weight x,y에 저장
            # inds가 모자라면 그대로 유지
            else:
                print('num left idx under cutline')



        if current_rightX is not None:
            if num_right_inds > min_num_pixel:
                current_rightX = np.int(np.mean(nonzerox[right_window_inds]))
                # cv2.circle(output, (int(current_rightX), int((win_y_low + win_y_high) / 2)), 5, (255, 0, 0), -1)
                cv2.circle(output, (int(current_rightX), int((win_y_low + new_high) / 2)), 5, (255, 0, 0), -1)
                
                right_weight_x.append(int(current_rightX))
                # right_weight_y.append(int((win_y_low + win_y_high) / 2))
                right_weight_y.append(int((win_y_low + new_high) / 2))
            # 중간 사라지게 테스트
            else:
                # print('window =', window)
                print('num right idx under cutline')
                # print('num right inds =', num_right_inds)
                # print('win_right_lane', win_right_lane)

        # 첫번째 윈도우에서 첫 포인트 판별
        if window == 0:
            # 현재 점이 존재할 때 필요한가?
            if current_leftX is not None:
                # 선이 중앙을 넘어가는 경우
                if (current_leftX >= 160):
                    right_line.startx = current_leftX
                    left_line.startx = None
                    left_line.detected = False

                    print('swap l to r')
                else:
                    left_line.startx = current_leftX
                # 점선이나 선이 없을 때
                if num_left_inds < min_num_pixel: 
                    left_line.detected = False
                    left_line.startx = None
                    print('left num idx =', num_left_inds)
            else:
                if current_rightX is not None:
                    left_line.counter == 0


            if current_rightX is not None:
                if current_rightX <= 160:
                    left_line.startx = current_rightX
                    right_line.startx = None
                    right_line.detected = False

                    print('swap r to l')
                else:
                    right_line.startx = current_rightX

                if num_right_inds < min_num_pixel:
                    right_line.detected = False
                    right_line.startx = None
                    print('right num idx =', num_right_inds)

            if left_line.detected == False:
                if right_line.detected == False:            # 좌우 모두 없음
                    if left_line.counter >= 5:
                        left_line.counter = 0
                    else:
                        left_line.counter += 1
                    left_line.startx = None
                    right_line.startx = None
                    
                else:                                       # 좌측 없음
                    if left_line.counter >= 5:
                        left_line.counter = 0
                    else:
                        left_line.counter += 1
                    left_line.startx = None
            else:                   
                if right_line.detected == False:            # 우측 없음
                    if left_line.counter >= 5:
                        left_line.counter = 0
                    else:
                        left_line.counter += 1
                    right_line.startx = None

    # weight x,y를 이용해 polyfit 계산
    if len(left_weight_x) > 3:
        left_fit = np.polyfit(left_weight_y, left_weight_x, 2)
       
        # 테스트 출력
        print('left fit =', left_fit)
    else:
        left_fit = None
    if len(right_weight_x) > 3:
        right_fit = np.polyfit(right_weight_y, right_weight_x, 2)
        
        # 테스트 출력
        print('right fit =', right_fit)
    else:
        right_fit = None

    left_line.current_fit = left_fit
    right_line.current_fit = right_fit

    output = drawline(output, left_fit, right_fit)
    

    return output



def blind_search(b_img, left_line, right_line):

    output = cv2.cvtColor(b_img, cv2.COLOR_GRAY2RGB)        # 색으로 결과를 확인하기 위한 이미지

    num_windows = left_line.num_window
    window_height = np.int(b_img.shape[0] / num_windows)        # b_img.shape[0] / num_windows = 240 / 8 = 30
    
    nonzero = b_img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])
    if left_line.counter == 0:
        output = find_first(b_img, left_line, right_line)
        print('counter =', left_line.counter)
    else:
        print('counter =', left_line.counter)
    output = sliding_window(b_img, output, left_line, right_line, window_height)
    
    print('lane detect result =', left_line.detected, right_line.detected)

    return output


def prev_window_refer(b_img, left_line, right_line):        # 좌우 모두 detected = True 일때
    
    output = cv2.cvtColor(b_img, cv2.COLOR_GRAY2RGB)        # 색으로 결과를 확인하기 위한 이미지

    nonzero = b_img.nonzero()
    nonzeroy = np.array(nonzero[0])
    nonzerox = np.array(nonzero[1])

    num_windows = left_line.num_window
    window_height = np.int(b_img.shape[0] / num_windows)
    min_num_pixel = left_line.min_pixel
    new_min_pixel = int(0.3 * min_num_pixel)


    current_leftX = left_line.startx
    current_rightX = right_line.startx

    window_margin = int(1.5 * left_line.window_margin)         # margin = 20

    win_left_lane = []
    win_right_lane = []

    left_weight_x, left_weight_y = [], []
    right_weight_x, right_weight_y = [], []

    l_flag, r_flag = 0, 0

        

    # TODO 윈도우 맨위 맨아래 하나씩 줄여보기 
    # Step through the windows one by one
    for window in range(num_windows):           # 윈도우 마다 사각형 그리고, 안쪽 선으로 추정되는 픽셀 nonzero 인덱스 추출


        # Identify window boundaries in x and y (and right and left)
        win_y_low = b_img.shape[0] - (window + 1) * window_height         # 위쪽이 더 작은 값임을 생각
        win_y_high = b_img.shape[0] - window * window_height
        new_high = int(0.7 * win_y_low + 0.3 * win_y_high)

        if current_leftX is not None:                                   # 시작점이 있으면 윈도우 정보(좌우 좌표값) 생성
            win_leftx_min = int(current_leftX - window_margin)
            win_leftx_max = int(current_leftX + window_margin)
            if win_leftx_min < 0:
                win_leftx_min = 0
                win_leftx_max = 0 + 2 * window_margin

            cv2.rectangle(output, (win_leftx_min, win_y_low), (win_leftx_max, win_y_high), (0, 255, 0), 2)              # 사각형 그림

            left_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= new_high) & (nonzerox >= win_leftx_min) & (
                        nonzerox <= win_leftx_max)).nonzero()[0]

            win_left_lane.append(left_window_inds)                          
            num_left_inds = len(left_window_inds)
        

        if current_rightX is not None:
            win_rightx_min = int(current_rightX - window_margin)
            win_rightx_max = int(current_rightX + window_margin)
            if win_rightx_max > b_img.shape[1]:
                win_rightx_min = b_img.shape[1] - 2 * window_margin
                win_rightx_max = b_img.shape[1]
            cv2.rectangle(output, (win_rightx_min, win_y_low), (win_rightx_max, win_y_high), (30, 200, 0), 2)
            
            # Identify the nonzero pixels in x and y within the window
            right_window_inds = ((nonzeroy >= win_y_low) & (nonzeroy <= new_high) & (nonzerox >= win_rightx_min) & (
                nonzerox <= win_rightx_max)).nonzero()[0]

            # Append these indices to the lists
            # 한 프레임에서만 계속 보관하게되어있었다
            win_right_lane.append(right_window_inds)
            num_right_inds = len(right_window_inds)


        if current_leftX is not None:                                                               
            if num_left_inds > new_min_pixel:     # 선이랑 윈도우랑 좀 겹쳐있으면
            # TODO 여기서 win_y_high 에서의 x값의 평균을 구하고자 한다.
                if window == 0:         # 첫째 바닥값을 추가
                    left_weight_x.append(int(current_leftX))
                    left_weight_y.append(int(win_y_high)) 
                    cv2.circle(output, (int(current_leftX), int(win_y_high)), 5, (0, 0, 255), -1)

                current_leftX = np.int(np.mean(nonzerox[left_window_inds]))
                # 윈도우 평균을 계산 후 노란점
                cv2.circle(output, (int(current_leftX), int((win_y_low + new_high) / 2)), 5, (0, 100, 100), -1)
                left_weight_x.append(int(current_leftX))
                left_weight_y.append(int((win_y_low + new_high) / 2)) 
                # 윈도우 하얀애들 평균을 current x에 두고 weight x,y에 저장
                # inds가 모자라면 그대로 유지
                l_flag = 0

            else:
                if l_flag == 0:
                    l_flag += 1
                else:
                    current_rightX = None
                    l_flag = 0
        else:
            if l_flag == 0:
                l_flag += 1
            else:
                current_rightX = None
                l_flag =0



        if current_rightX is not None:
            if num_right_inds > new_min_pixel:
                if window == 0:
                    right_weight_x.append(int(current_rightX))
                    right_weight_y.append(int(win_y_high))
                    cv2.circle(output, (int(current_rightX), int(win_y_high)), 5, (0, 100, 255), -1)

                current_rightX = np.int(np.mean(nonzerox[right_window_inds]))
                cv2.circle(output, (int(current_rightX), int((win_y_low + new_high) / 2)), 5, (255, 0, 0), -1)
                right_weight_x.append(int(current_rightX))
                right_weight_y.append(int((win_y_low + new_high) / 2))
                r_flag = 0
            
            else:
                if r_flag == 0:
                    r_flag += 1
                else:
                    current_rightX = None
                    r_flag = 0
        else:
            if r_flag == 0:
                r_flag += 1
            else:
                current_rightX = None
                r_flag = 0  
        
        # 첫번째 윈도우에서 첫 포인트 판별
        if window == 0:
            # 현재 점이 존재할 때 필요한가?
            if current_leftX is not None:
                # 선이 중앙을 넘어가는 경우
                if (current_leftX >= 160):
                    right_line.startx = current_leftX
                    left_line.startx = None
                    left_line.detected = False

                    print('swap l to r')
                else:
                    left_line.startx = current_leftX
                # 점선이나 선이 없을 때
                if num_left_inds < min_num_pixel: 
                    left_line.detected = False
                    left_line.startx = None
                    print('left num idx =', num_left_inds)
            else:
                if current_rightX is not None:
                    left_line.counter == 0


            if current_rightX is not None:
                if current_rightX <= 160:
                    left_line.startx = current_rightX
                    right_line.startx = None
                    right_line.detected = False

                    print('swap r to l')
                else:
                    right_line.startx = current_rightX

                if num_right_inds < min_num_pixel:
                    right_line.detected = False
                    right_line.startx = None
                    print('right num idx =', num_right_inds)

    if left_line.detected == False:
        if right_line.detected == False:            # 좌우 모두 없음
            if left_line.counter >= 5:
                left_line.counter = 0
            else:
                left_line.counter += 1
            left_line.startx = None
            right_line.startx = None
            
        else:                                       # 좌측 없음
            if left_line.counter >= 5:
                left_line.counter = 0
            else:
                left_line.counter += 1
            left_line.startx = None
    else:                   
        if right_line.detected == False:            # 우측 없음
            if left_line.counter >= 5:
                left_line.counter = 0
            else:
                left_line.counter += 1
            right_line.startx = None

    # weight x,y를 이용해 polyfit 계산
    if len(left_weight_x) > 3:
        left_fit = np.polyfit(left_weight_y, left_weight_x, 2)
       
        # 테스트 출력
        print('left fit =', left_fit)
    else:
        left_fit = None
    if len(right_weight_x) > 3:
        right_fit = np.polyfit(right_weight_y, right_weight_x, 2)
        
        # 테스트 출력
        print('right fit =', right_fit)
    else:
        right_fit = None

    left_line.current_fit = left_fit
    right_line.current_fit = right_fit

    output = drawline(output, left_fit, right_fit)
    print('lane detect result =', left_line.detected, right_line.detected)
    return output


def find_LR_lines(binary_img, left_line, right_line):



    print('left line detected =', left_line.detected)
    print('right line detected =', right_line.detected)

    # if don't have lane lines info
    if (left_line.detected == False) or (right_line.detected == False):
        return blind_search(binary_img, left_line, right_line)
    # if have lane lines info
    else:
        return prev_window_refer(binary_img, left_line, right_line)


def make_center(binary_img):
    left_fit = left_line.current_fit
    right_fit = right_line.current_fit

    ploty = np.linspace(0, binary_img.shape[0] - 1, binary_img.shape[0])    

    if left_fit is not None:
        left_plotx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    if right_fit is not None:
        right_plotx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]

    if left_fit is not None:
        if right_fit is not None:
            centerx = np.mean([left_plotx, right_plotx], axis=0)
        else:
            centerx = np.add(left_plotx, 70)
    else:
        if right_fit is not None:
            centerx = np.subtract(right_plotx, 70)
        else:
            centerx = None

    if centerx is not None:
        cv2.polylines(binary_img, np.int_([np.array([np.transpose(np.vstack([centerx, ploty]))])], isClosed=False, color=(255, 0, 255), thickness=8))

        msg_center = Float64()
        msg_center.data = centerx.item(120)
        pub_lane.publish(msg_center)
    return binary_img


""" ros와 line detect 구문 분리용 주석"""


def image_callback(msg):
    # print("PyImageSubscriber node  Received an image!")
    try:
        # Convert your ROS Image message to OpenCV2
        cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError, e:
        print(e)
    else:
        resized = ready_process(cv2_img)

        lab = lab_combine(resized)

        # warp
        warped = bird_view(lab)

        # # 이미지, 라인 두개를 넣고, 처리된 사진을 가져옴
        searching_img = find_LR_lines(warped, left_line, right_line)
        
        result = make_center(searching_img)

        # if left_line.detected == True:
        #     if left_line.current_fit is not None:
        #         msg_data = [0, left_line.current_fit[0] if left_line.current_fit is not None else 0, left_line.current_fit[1], left_line.current_fit[2]]
        #     else:
        #         msg_data = [0, 0, 0, 1]
        # else:
        #     msg_data = [1, 2, 3, 4]
        # line_msg = Float32MultiArray()
        # line_msg.data = msg_data
        # pub.publish(line_msg)

        # Wait 30 ms to allow image to be drawn.
        # Image won't display properly without this cv2.waitkey
        # cv2.waitKey(30) 
        # Save your OpenCV2 image as a jpeg 
        # cv2.imwrite('camera_image.jpeg', cv2_img)
        cv2.imshow('result', result)

    
def image_listener():
    # Initiate the node
    rospy.init_node('py_image_listener')
    # Setupt the subscription, camera/rb/image_raw is used in turtlebot_gazebo example
    #rospy.Subscriber("jetbot_camera/raw", Image, image_callback)
    rospy.Subscriber("/video_publisher/image_raw_bgr8", Image, image_callback)
    pub_lane = rospy.Publisher("/detect/lane", Float64, queue_size=1)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    cv2.destroyWindow("Image Display")

if __name__ == '__main__':
    # line_msg = Float32MultiArray(data = msg_data)
    # pub = rospy.Publisher('line_data', Float32MultiArray, queue_size=1)
    image_listener()
