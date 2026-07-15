#  -*- coding: UTF-8 -*-

import cv2
import numpy as np
import time
from unihiker_k10 import pin

# =====================
# 定义引脚
# =====================
AIN1 = pin(2)   # 左轮方向1
AIN2 = pin(3)   # 左轮方向2
PWMA = pin(0)   # 左轮PWM

BIN1 = pin(4)   # 右轮方向1
BIN2 = pin(5)   # 右轮方向2
PWMB = pin(1)   # 右轮PWM

STBY = pin(6)   # TB6612使能

# =====================
# 初始化
# =====================
STBY.write_digital(1)

# =====================
# 电机控制函数
# =====================
def left_motor(speed, direction):
    if direction == 1:
        AIN1.write_digital(1)
        AIN2.write_digital(0)
    else:
        AIN1.write_digital(0)
        AIN2.write_digital(1)
    PWMA.write_analog(value=speed, freq=1000)

def right_motor(speed, direction):
    if direction == 1:
        BIN1.write_digital(1)
        BIN2.write_digital(0)
    else:
        BIN1.write_digital(0)
        BIN2.write_digital(1)
    PWMB.write_analog(value=speed, freq=1000)

def stop():
    PWMA.write_analog(value=0, freq=1000)
    PWMB.write_analog(value=0, freq=1000)

def speed(x, y):
    left_dir = 1 if x > 0 else 0
    right_dir = 1 if y > 0 else 0
    left_speed = int(abs(x))
    right_speed = int(abs(y))
    
    if left_speed > 1000:
        left_speed = 1000
    if right_speed > 1000:
        right_speed = 1000
    
    left_motor(left_speed, left_dir)
    right_motor(right_speed, right_dir)

#获取线中心位置函数
def process_frame(img): 
    global lp1
    global lp2
    global lcs
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
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

cv2.namedWindow("n", cv2.WINDOW_NORMAL)
cv2.moveWindow("n", 0, 0)
cv2.resizeWindow("n", 240, 320)
vd = cv2.VideoCapture()
vd.set(cv2.CAP_PROP_FRAME_WIDTH,240)
vd.set(cv2.CAP_PROP_FRAME_HEIGHT,320)
vd.open(0)

if vd.isOpened():
    lp1 = 320
    lp2 = 380
    lcs = 255
    bs = 300
    
    kp = 0.010
    kd = 0.040
    old_err = 0
    
    while True:
        ret, cvi = vd.read() 
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
        speed(ls,rs)
        if cv2.waitKey(20) & 0xff== 97:
            speed(0,0)
            break
        print(ls,rs,center)
vd.release()
cv2.destroyAllWindows()
stop()