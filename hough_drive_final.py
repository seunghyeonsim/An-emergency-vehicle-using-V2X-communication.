#!/usr/bin/env python
# -*- coding: utf-8 -*-
####################################################################
# 프로그램명 : hough_drive_c1.py 일반차
# 작 성 자 : (주)자이트론
# 생 성 일 : 2020년 07월 23일
# 본 프로그램은 상업 라이센스에 의해 제공되므로 무단 배포 및 상업적 이용을 금합니다.
####################################################################

import rospy, rospkg, time
import numpy as np
import cv2, math
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from std_msgs.msg import Int32MultiArray
from math import *
import signal
import sys
import os
import time

isCross = False
ifCross2 = False

hdbd = 0


def onChange(x): 
    pass

cv2.namedWindow('trackBar',cv2.WINDOW_NORMAL) 
#cv2.createTrackbar('cannylow', 'trackBar',0,255, onChange)
#cv2.createTrackbar('cannyhigh', 'trackBar',0,255, onChange)

cv2.createTrackbar('hough1', 'trackBar',0,320, onChange)
cv2.createTrackbar('hough2', 'trackBar',0,320, onChange)
cv2.createTrackbar('hough3', 'trackBar',0,320, onChange)
cv2.createTrackbar('hough4', 'trackBar',0,320, onChange)
cv2.createTrackbar('hough5', 'trackBar',0,320, onChange)
cv2.createTrackbar('hough6', 'trackBar',0,320, onChange)
cv2.createTrackbar('hough7', 'trackBar',0,320, onChange)
cv2.createTrackbar('hough8', 'trackBar',0,320, onChange)


def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
Width = 320
Height = 240
Offset = 160
Gap = 40
ult_data = [0,0,0,0,0,0,0,0]
anglenow=0

cam = False
cam_debug = True

sub_f = 0

time_c = 0

def img_callback(data):
    global image   
    global sub_f 
    global time_c

    sub_f += 1
    if time.time() - time_c > 1:
        #print("pub fps :", sub_f)
        time_c = time.time()
        sub_f = 0

    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

# draw lines
def draw_lines(img, lines):
    global Offset
    for line in lines:
        x1, y1, x2, y2 = line[0]
        img = cv2.line(img, (x1, y1+Offset), (x2, y2+Offset), (0, 255, 0), 2)
    return img

# draw rectangle
def draw_rectangle(img, lpos, rpos, offset=0):
    center = (lpos + rpos) / 2

    cv2.rectangle(img, (lpos - 2, 7 + offset),
                       (lpos + 2, 12 + offset),
                       (255, 0, 0), 2)
    cv2.rectangle(img, (rpos - 2, 7 + offset),
                       (rpos + 2, 12 + offset),
                       (0, 255, 0), 2)
    cv2.rectangle(img, (center-2, 7 + offset),
                       (center+2, 12 + offset),
                       (0, 0, 255), 2)    
    cv2.rectangle(img, (157, 7 + offset),
                       (162, 12 + offset),
                       (255, 0, 255), 2)
    return img

# left lines, right lines
def divide_left_right(lines):
    global Width

    low_slope_threshold = 0.2
    high_slope_threshold = 10

    # calculate slope & filtering with threshold
    slopes = []
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if x2 - x1 == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if low_slope_threshold < abs(slope) < high_slope_threshold:
            slopes.append(slope)
            new_lines.append(line[0])

    # divide lines left to right
    left_lines = []
    right_lines = []
    th = 25

    for j in range(len(slopes)):
        Line = new_lines[j]
        slope = slopes[j]

        x1, y1, x2, y2 = Line

        if (slope < 0) and (x2 < Width/2 - th):
            left_lines.append([Line.tolist()])
        elif (slope > 0) and (x1 > Width/2 + th):
            right_lines.append([Line.tolist()])

    return left_lines, right_lines

# get average m, b of line, sum of x, y, mget lpos, rpos
def get_line_pos(img, lines, left=False, right=False):
    global Width, Height
    global Offset, Gap, cam_debug

    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    
    m = 0
    b = 0

    if size != 0:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)

        m = m_sum / size
        b = y_avg - m * x_avg
    
          
    

    if m == 0 and b == 0:
        if left:
            pos = 0
        elif right:
            pos = Width
    else:
        y = Gap / 2

        pos = (y - b) / m

        if cam_debug:
            b += Offset
            xs = (Height - b) / float(m)
            xe = ((Height/2) - b) / float(m)

            cv2.line(img, (int(xs), Height), (int(xe), (Height/2)), (255, 0,0), 3)
    #if size == 0:
        #pos = 5000
       
    return img, int(pos)

# show image and return lpos, rpos
def process_image(frame):
    global cam_processed 
    global Width
    global Offset, Gap
    global cam, cam_debug, img

    
    frame = cv2.resize(frame, dsize=(320, 240), interpolation=cv2.INTER_AREA)
    
    hough1 = cv2.getTrackbarPos('hough1', 'trackBar')
    hough2 = cv2.getTrackbarPos('hough2', 'trackBar')
    hough3 = cv2.getTrackbarPos('hough3', 'trackBar')
    hough4 = cv2.getTrackbarPos('hough4', 'trackBar')
    hough5 = cv2.getTrackbarPos('hough5', 'trackBar')
    hough6 = cv2.getTrackbarPos('hough6', 'trackBar')
    hough7 = cv2.getTrackbarPos('hough7', 'trackBar')
    hough8 = cv2.getTrackbarPos('hough8', 'trackBar')
    
    
    
    pts1 = np.float32([[57, 101], [275, 101], [0, 157], [320, 148]])
    pts2 = np.float32([[10, 0], [310, 0], [10, 240], [310, 240]])
    M = cv2.getPerspectiveTransform(pts1, pts2)
    birdeye = cv2.warpPerspective(frame, M, (320, 240))
    birdeye_roi = birdeye[90 :240, 0 : 320]
    cv2.imshow("birdeye_roi", birdeye_roi)
    
    
    img_gray = cv2.cvtColor(birdeye_roi, cv2.COLOR_BGR2GRAY)


    ret, img_binary = cv2.threshold(img_gray, 200, 255, cv2.THRESH_BINARY)
    #cv2.imshow("img_binary", img_binary)



    test = cv2.cvtColor(img_binary, cv2.COLOR_GRAY2BGR)
    reccnt = 0
    
    _, contours, hierarchy = cv2.findContours(img_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for cnt in contours:
        cv2.drawContours(img_gray, [cnt], 0, (255, 0, 0), 3)  # blue

    #cv2.imshow("result", img_gray)
    
    for cnt in contours:
        #size = len(cnt)
        # print(size)

        epsilon = 0.04 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)

        size = len(approx)
        #print('size %f' %size)

        cv2.line(birdeye_roi, tuple(approx[0][0]), tuple(approx[size - 1][0]), (0, 0, 255), 3)
        for k in range(size - 1):
            cv2.line(birdeye_roi, tuple(approx[k][0]), tuple(approx[k + 1][0]), (0, 255, 100 + k * 10), 3)
            
            
    
        

        if cv2.isContourConvex(approx):
            if size == 4:
                reccnt += 1
                
            else:
                pass
            
        
    #cv2.imshow("result", birdeye_roi)
    print('reccnt %f' %reccnt)
    
    global isCross
    global isCross2
    global hdbd
    
    
    if reccnt >= 5:
        isCross = True
        hdbd = hdbd +1
    print(bool(isCross))
    print('hdbd %f' %hdbd)
   
    
    
    
    
    # gray
    gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    #cv2.imshow('gray', gray)

    
    roi = gray[86 :137, 0 : Width]
    #roi = gray[hough1 :hough2, hough3 : hough4]
    
    
    cv2.imshow('roi', roi)
    #cannylow = cv2.getTrackbarPos('cannylow', 'trackBar')
    #cannyhigh = cv2.getTrackbarPos('cannyhigh', 'trackBar')
    # blur

    kernel_size = 5
    standard_deviation_x = 1.5     #Kernel standard deviation along X-axis
    blur_gray = cv2.GaussianBlur(roi, (kernel_size, kernel_size), standard_deviation_x)

    cv2.imshow('blur', blur_gray)
    # canny edge
    #cannylow = cv2.getTrackbarPos('cannylow', 'trackBar')
    #cannyhigh = cv2.getTrackbarPos('cannyhigh', 'trackBar')
    #low_threshold = 10+cannylow
    #high_threshold = 20+cannyhigh

    edge_img = cv2.Canny(np.uint8(blur_gray), 90, 160, kernel_size)
    cv2.imshow('c_e', edge_img)

    #hough1 = cv2.getTrackbarPos('hough1', 'trackBar')
    #hough2 = cv2.getTrackbarPos('hough2', 'trackBar')
    #hough3 = cv2.getTrackbarPos('hough3', 'trackBar')
    #hough4 = cv2.getTrackbarPos('hough4', 'trackBar')
    #HoughLinesP
    #cannylow = cv2.getTrackbarPos('cannylow', 'trackBar')
    #cannyhigh = cv2.getTrackbarPos('cannyhigh', 'trackBar')
    #cannylow = cv2.setTrackbarPos('cannyhigh', 'trackBar',11)
    #cannyhigh = cv2.setTrackbarPos('cannyhigh', 'trackBar',9)
    
    all_lines = cv2.HoughLinesP(edge_img, 1, math.pi/180,20,20,6)
    
    
    for line in all_lines:
        x1, y1, x2, y2 = line[0]
        hough_img = cv2.line(blur_gray, (x1, y1), (x2, y2), (255, 255, 255), 2)
    
    cv2.imshow('hough_img', hough_img)
    
    if cam:
        cv2.imshow('calibration', frame)
    # divide left, right lines
    if all_lines is None:
        return (Width)/2, (Width)/2, False
    
    left_lines, right_lines = divide_left_right(all_lines)

    # get center of lines
    frame, lpos = get_line_pos(frame, left_lines, left=True)
    frame, rpos = get_line_pos(frame, right_lines, right=True)

    if cam_debug:
        # draw lines
        frame = draw_lines(frame, left_lines)
        frame = draw_lines(frame, right_lines)
        frame = cv2.line(frame, (115, 117), (205, 117), (0,255,255), 2)

        # draw rectangle
        #if lpos==5000 :
            #lpos=40
        
        #if rpos==5000 :
            #rpos=Width-40

        
        frame = draw_rectangle(frame, lpos, rpos, offset=Offset)
        cv2.imshow('rectangle', frame)
        frame = cv2.rectangle(frame, (0, Offset), (Width, Offset+Gap), (0, 255, 0), 2)
        

    img = frame
    bridge = CvBridge()
    imgMsg =bridge.cv2_to_imgmsg(img,"bgr8")
    cam_processed.publish(imgMsg)

    return lpos, rpos, True

def draw_steer(steer_angle):
    global Width, Height, img
    #img = cv_image

    arrow = cv2.imread('/home/pi/xycar_ws/src/auto_drive/src/steer_arrow.png')

    origin_Height = arrow.shape[0]
    origin_Width = arrow.shape[1]
    steer_wheel_center = origin_Height * 0.74
    arrow_Height = Height/2
    arrow_Width = (arrow_Height * 462)/728

    matrix = cv2.getRotationMatrix2D((origin_Width/2, steer_wheel_center), (-steer_angle) * 1.5, 0.7)    
    arrow = cv2.warpAffine(arrow, matrix, (origin_Width+60, origin_Height))
    arrow = cv2.resize(arrow, dsize=(arrow_Width, arrow_Height), interpolation=cv2.INTER_AREA)

    gray_arrow = cv2.cvtColor(arrow, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray_arrow, 1, 255, cv2.THRESH_BINARY_INV)

    arrow_roi = img[arrow_Height: Height, (Width/2 - arrow_Width/2) : (Width/2 + arrow_Width/2)]
    arrow_roi = cv2.add(arrow, arrow_roi, mask=mask)
    res = cv2.add(arrow_roi, arrow)
    img[(Height - arrow_Height): Height, (Width/2 - arrow_Width/2): (Width/2 + arrow_Width/2)] = res
    
    
    cv2.imshow('steer', img)


def pure_pursuit(lpos,rpos):
    Gx = ((lpos + rpos) / 2)-160
       	
    if Gx ==0:
        angle = 0
    else:
        Gy = 45
            
        Ld = ((Gx)**2 + (Gy+24)**2)**0.5
            
        x = abs(Gx)
        y= Gy + 20
            
        r = (Ld**2)/(2*x)
        theta = math.atan(y/x)
        alpha = math.pi-(2*theta)
        L = r*math.tan(alpha/2)
        delta = math.atan(2*L*math.sin(alpha/2)/Ld)
        degrees = math.degrees(delta)
            
        if Gx<0:
            angle = (-1)*degrees*2
        else:
            angle = degrees*2
    drive(angle, 14)
    
def jy_method(lpos,rpos):
    llpos=lpos+20
    rrpos=rpos+300
    Gx = ((llpos + rrpos) / 2)-160
    
       	
    if Gx ==0:
        angle = 0
    else:
        Gy = 120
            
        
        theta = math.atan(Gy/Gx)
        
        
            
        
        angle = theta*20
    drive(angle, 14)

def callback_ult(msg):
    global ult_data
    ult_data = msg.data
    print ult_data
    

def start():
    global anglenow
    global cam_processed
    global motor
    global image
    global Width, Height
    global Is_Emergency_Mode_On
    Is_Emergency_Mode_On=False
    rospy.init_node('auto_drive')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    cam_processed = rospy.Publisher('cam_processed', Image, queue_size=10)
    

    
    rospy.Subscriber('xycar_ultrasonic', Int32MultiArray, callback_ult)

    image_sub = rospy.Subscriber("/usb_cam/image_raw/",Image,img_callback)
    print "---------- Xycar C1 HD v1.0 ----------"
    time.sleep(3)

    #sq = rospy.Rate(30)
    t_check = time.time()
    f_n = 0

    while True:
    #while not rospy.is_shutdown():

        #while True:

        #while not image.size == (Width*Height*3):
	#    continue            
        
        

        draw_img = image.copy()
        lpos, rpos, go = process_image(draw_img)
        
        pure_pursuit(lpos,rpos)
        
        if isCross == True and hdbd >=2:
            max_time_end = time.time() + (1.1)
            while True:
                
                drive(0,14)
                
                if time.time()>max_time_end:
                    break
            
            
            max_time_end = time.time() + (4.2) #2
            while True:
                drive(-50,17)
                if time.time()>max_time_end:
                    break
                
            max_time_end = time.time() + (5)
            while True:
                
                drive(0,0)
                
                if time.time()>max_time_end:
                    break
            max_time_end = time.time() + (2)
            while True:
                
                drive(0,14)
                
                if time.time()>max_time_end:
                    break
                
            global isCross
            isCross=False
            print(isCross)
            
            
        if isCross == True:
            
            max_time_end = time.time() + (5)
            while True:
                drive(0,0)
                    
                    
                if time.time()>max_time_end:
                    break 
            
            max_time_end = time.time() + (1.3) #1
            while True:
                drive(0,14)
                if time.time()>max_time_end:
                    break
            
            max_time_end = time.time() + (4.5) #2
            while True:
                drive(-50,17)
                if time.time()>max_time_end:
                    global isCross
                    isCross=False
                    break
                
            Gx = ((lpos + rpos) / 2)-160
       	
            if Gx ==0:
                angle = 0
            else:
                Gy = 45
            
                Ld = ((Gx)**2 + (Gy+24)**2)**0.5
            
                x = abs(Gx)
                y= Gy + 20
            
                r = (Ld**2)/(2*x)
                theta = math.atan(y/x)
                alpha = math.pi-(2*theta)
                L = r*math.tan(alpha/2)
                delta = math.atan(2*L*math.sin(alpha/2)/Ld)
                degrees = math.degrees(delta)
            
                if Gx<0:
                    angle = (-1)*degrees*2
                else:
                    angle = degrees*2
            drive(angle, 14) 
            
            
        
            
        #hdbd(draw_img)
        
        #if isCross = True:
           # drive(-50, 14)
            
            
        
        Gx = ((lpos + rpos) / 2)-160
       	
        if Gx ==0:
            angle = 0
        else:
            Gy = 120
            
            Ld = ((Gx)**2 + (Gy+24)**2)**0.5
            
            x = abs(Gx)
            y= Gy + 24
            
            r = (Ld**2)/(2*x)
            theta = math.atan(y/x)
            alpha = math.pi-(2*theta)
            L = r*math.tan(alpha/2)
            delta = math.atan(2*L*math.sin(alpha/2)/Ld)
            degrees = math.degrees(delta)
            
            if Gx<0:
                angle = (-1)*degrees*0.9
            else:
                angle = degrees*0.9
            
            
#             anglenow= anglenow-(anglenow*0.1)+angle
            
            
            
      
            
            
                

        steer_angle = anglenow 
        draw_steer(angle)
       
    
        #drive(angle, 14)
       
            
        cv2.waitKey(1)
        #sq.sleep()

if __name__ == '__main__':
    start()