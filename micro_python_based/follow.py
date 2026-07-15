#  -*- coding: UTF-8 -*-

# MindPlus
# Python
import cv2
import numpy as np
from pinpong.board import Board
from pinpong.board import NeoPixel
from pinpong.board import Board,Pin
from pinpong.extension.unihiker import *

def numberMap(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
def nm(x):
    return int(numberMap(x, 0, 255, 0, 1023))
def speed(x, y):
    if x>0:
        p_p5_out.write_digital(1)
    else:
        p_p5_out.write_digital(0)
    if y>0:
        p_p6_out.write_digital(1)
    else:
        p_p6_out.write_digital(0)
    x = nm(abs(x))
    y = nm(abs(y))
    p_p8_pwm.write_analog(x)
    p_p16_pwm.write_analog(y)

#获取线中心位置函数
def process_frame(img): 
    global lp1
    global lp2
    global lcs
    #转换为hsv色彩空间
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 定义红色的HSV范围
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    # 创建红色掩码
    mask = cv2.inRange(hsv, lower_red, upper_red)
    # 形态学操作：膨胀和腐蚀
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=4)
    mask = cv2.erode(mask, kernel, iterations=4)

    colorp1=mask[lp1]
    colorp2=mask[lp2]
    try:
        lineccp1 = np.sum(colorp1==lcs)
        lineccp2 = np.sum(colorp2==lcs)
        lineip1 = np.where(colorp1==lcs)
        lineip2 = np.where(colorp2==lcs)
        if lineccp1 == 0:
            lineccp1 = 1
        if lineccp2 == 0:
            lineccp2 = 1
        leftp1 = lineip1[0][lineccp1-1]
        rightp1 = lineip1[0][0]
        centerp1 = int((leftp1+rightp1)/2)
        leftp2 = lineip2[0][lineccp2-1]
        rightp2 = lineip2[0][0]
        centerp2 = int((leftp2+rightp2)/2)
        center = int((centerp1+centerp2)/2)
    except:
        center = None
    return mask,center

def pid(x):
    global kp,kd,old_err
    err = x-320
    offset = int(err*kp+(err-old_err)*kd)
    old_err = err
    return offset

Board().begin()
p_p5_out=Pin(Pin.P5, Pin.OUT)
p_p8_pwm=Pin(Pin.P8, Pin.PWM)
p_p6_out=Pin(Pin.P6, Pin.OUT)
p_p16_pwm=Pin(Pin.P16, Pin.PWM)
cv2.namedWindow("n", cv2.WINDOW_NORMAL)
cv2.moveWindow("n", 0, 0)
cv2.resizeWindow("n", 240, 320)
vd = cv2.VideoCapture()
vd.set(cv2.CAP_PROP_FRAME_WIDTH,240)
vd.set(cv2.CAP_PROP_FRAME_HEIGHT,320)
vd.open(0)

if vd.isOpened():
    lp1 = 320  #检测线1
    lp2 = 380  #检测线2
    lcs = 255  #检测点为白色=255 黑色=0
    bs = 60    #基础速度
    
    #PID参数
    kp = 0.010
    kd = 0.040
    old_err = 0   #用于求差分项
    
    while True:
        ret, cvi = vd.read() 
        # 获取图像 h = 480  w = 640
        h,w,c = cvi.shape
        res,center=process_frame(cvi)
        cv2.line(cvi, (0,lp1), (640,lp1), (0,255,0), 3, cv2.FILLED)
        cv2.drawMarker(cvi, (center, int(lp1+((lp2-lp1)/2))), (0,0,255), cv2.MARKER_CROSS, 20, 5, cv2.FILLED)
        cv2.line(cvi, (0,lp2), (640,lp2), (0,255,255), 3, cv2.FILLED)
        cv2.imshow("n", cvi)
        f = 0
        if center!= None:
            offset = pid(center)
            if center>320 :
                f = 1
            else:
                f = 2
            ls = bs + offset*10
            rs = bs - offset*10
        else:
            if f==1:
                for i in range(10):
                    ls,rs = bs,-1*bs 
                    time.sleep(0.1)
            elif f==2:
                for i in range(10):
                    ls,rs= -1*bs,bs
                    time.sleep(0.1)
            else:
                ls,rs= bs,bs
        speed(ls,rs)#设置两轮速度
        if cv2.waitKey(20) & 0xff== 97:
            speed(0,0)
            break
        print(ls,rs,center)
vd.release()
cv2.destroyAllWindows()