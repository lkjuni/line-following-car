#  -*- coding: UTF-8 -*-

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

def move_forward(speed=500):
    left_motor(speed, 1)
    right_motor(speed, 1)

def move_backward(speed=500):
    left_motor(speed, 0)
    right_motor(speed, 0)

def turn_left(speed=500, turn_speed=200):
    left_motor(turn_speed, 1)
    right_motor(speed, 1)

def turn_right(speed=500, turn_speed=200):
    left_motor(speed, 1)
    right_motor(turn_speed, 1)

def spin_left(speed=400):
    left_motor(speed, 0)
    right_motor(speed, 1)

def spin_right(speed=400):
    left_motor(speed, 1)
    right_motor(speed, 0)

# =====================
# 摄像头循迹功能
# =====================
def process_frame(img, lp1, lp2, lcs):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_red = np.array([0, 100, 100])
    upper_red = np.array([10, 255, 255])
    mask = cv2.inRange(hsv, lower_red, upper_red)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.dilate(mask, kernel, iterations=4)
    mask = cv2.erode(mask, kernel, iterations=4)

    colorp1 = mask[lp1]
    colorp2 = mask[lp2]
    try:
        lineccp1 = np.sum(colorp1 == lcs)
        lineccp2 = np.sum(colorp2 == lcs)
        lineip1 = np.where(colorp1 == lcs)
        lineip2 = np.where(colorp2 == lcs)
        if lineccp1 == 0:
            lineccp1 = 1
        if lineccp2 == 0:
            lineccp2 = 1
        leftp1 = lineip1[0][lineccp1 - 1]
        rightp1 = lineip1[0][0]
        centerp1 = int((leftp1 + rightp1) / 2)
        leftp2 = lineip2[0][lineccp2 - 1]
        rightp2 = lineip2[0][0]
        centerp2 = int((leftp2 + rightp2) / 2)
        center = int((centerp1 + centerp2) / 2)
    except:
        center = None
    return mask, center

def pid_controller(x, kp, kd, old_err):
    err = x - 320
    offset = int(err * kp + (err - old_err) * kd)
    return offset, err

def tracking_mode():
    cv2.namedWindow("Tracking", cv2.WINDOW_NORMAL)
    cv2.moveWindow("Tracking", 0, 0)
    cv2.resizeWindow("Tracking", 240, 320)
    vd = cv2.VideoCapture()
    vd.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
    vd.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
    vd.open(0)

    if not vd.isOpened():
        print("摄像头打开失败")
        return

    lp1 = 320
    lp2 = 380
    lcs = 255
    bs = 300

    kp = 0.010
    kd = 0.040
    old_err = 0
    last_direction = 0

    print("循迹模式启动，按 'a' 键退出")
    try:
        while True:
            ret, cvi = vd.read()
            if not ret:
                break

            h, w, c = cvi.shape
            res, center = process_frame(cvi, lp1, lp2, lcs)

            cv2.line(cvi, (0, lp1), (640, lp1), (0, 255, 0), 3, cv2.FILLED)
            cv2.line(cvi, (0, lp2), (640, lp2), (0, 255, 255), 3, cv2.FILLED)
            if center is not None:
                cv2.drawMarker(cvi, (center, int(lp1 + ((lp2 - lp1) / 2))), (0, 0, 255), cv2.MARKER_CROSS, 20, 5, cv2.FILLED)
            cv2.imshow("Tracking", cvi)

            if center is not None:
                offset, old_err = pid_controller(center, kp, kd, old_err)
                if center > 320:
                    last_direction = 1
                else:
                    last_direction = 2

                ls = bs + offset * 10
                rs = bs - offset * 10
            else:
                if last_direction == 1:
                    ls, rs = bs, -bs
                elif last_direction == 2:
                    ls, rs = -bs, bs
                else:
                    ls, rs = bs, bs

            left_dir = 1 if ls > 0 else 0
            right_dir = 1 if rs > 0 else 0
            left_motor(int(abs(ls)), left_dir)
            right_motor(int(abs(rs)), right_dir)

            if cv2.waitKey(20) & 0xff == ord('a'):
                break
    finally:
        vd.release()
        cv2.destroyAllWindows()
        stop()
        print("循迹模式已退出")

# =====================
# 演示模式
# =====================
def demo_mode():
    print("演示模式启动")
    try:
        print("前进 2 秒")
        move_forward(500)
        time.sleep(2)

        print("停止 1 秒")
        stop()
        time.sleep(1)

        print("后退 2 秒")
        move_backward(500)
        time.sleep(2)

        print("停止 1 秒")
        stop()
        time.sleep(1)

        print("左转 1.5 秒")
        turn_left(500, 200)
        time.sleep(1.5)

        print("停止 1 秒")
        stop()
        time.sleep(1)

        print("右转 1.5 秒")
        turn_right(500, 200)
        time.sleep(1.5)

        print("停止 1 秒")
        stop()
        time.sleep(1)

        print("原地左转 1 秒")
        spin_left(400)
        time.sleep(1)

        print("停止 1 秒")
        stop()
        time.sleep(1)

        print("原地右转 1 秒")
        spin_right(400)
        time.sleep(1)

        stop()
        print("演示模式结束")
    except KeyboardInterrupt:
        stop()
        print("演示模式被中断")

# =====================
# 键盘控制模式
# =====================
def keyboard_mode():
    print("键盘控制模式启动")
    print("方向键: 上下左右控制方向")
    print("W/S: 增加/减少速度")
    print("空格: 停止")
    print("Q: 退出")

    speed = 400
    try:
        while True:
            import sys
            if sys.platform == 'win32':
                import msvcrt
                if msvcrt.kbhit():
                    key = msvcrt.getch()
                    if key == b'w' or key == b'W':
                        speed = min(speed + 50, 1000)
                        print(f"速度增加: {speed}")
                    elif key == b's' or key == b'S':
                        speed = max(speed - 50, 0)
                        print(f"速度减少: {speed}")
                    elif key == b' ':
                        stop()
                        print("停止")
                    elif key == b'q' or key == b'Q':
                        break
                    elif key == b'H':
                        move_forward(speed)
                        print(f"前进: {speed}")
                    elif key == b'P':
                        move_backward(speed)
                        print(f"后退: {speed}")
                    elif key == b'K':
                        turn_left(speed, speed // 3)
                        print(f"左转: {speed}")
                    elif key == b'M':
                        turn_right(speed, speed // 3)
                        print(f"右转: {speed}")
            else:
                import select
                import tty
                import termios
                fd = sys.stdin.fileno()
                old_settings = termios.tcgetattr(fd)
                try:
                    tty.setraw(sys.stdin.fileno())
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key == 'w':
                            speed = min(speed + 50, 1000)
                            print(f"速度增加: {speed}")
                        elif key == 's':
                            speed = max(speed - 50, 0)
                            print(f"速度减少: {speed}")
                        elif key == ' ':
                            stop()
                            print("停止")
                        elif key == 'q':
                            break
                        elif key == 'A':
                            move_forward(speed)
                            print(f"前进: {speed}")
                        elif key == 'B':
                            move_backward(speed)
                            print(f"后退: {speed}")
                        elif key == 'D':
                            turn_left(speed, speed // 3)
                            print(f"左转: {speed}")
                        elif key == 'C':
                            turn_right(speed, speed // 3)
                            print(f"右转: {speed}")
                finally:
                    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop()
        print("键盘控制模式已退出")

# =====================
# 主菜单
# =====================
def main():
    print("=" * 40)
    print("    循迹小车控制系统")
    print("=" * 40)
    print("1. 演示模式")
    print("2. 键盘控制模式")
    print("3. 摄像头循迹模式")
    print("4. 退出")
    print("=" * 40)

    while True:
        try:
            choice = input("请输入选择 (1-4): ")
            if choice == '1':
                demo_mode()
            elif choice == '2':
                keyboard_mode()
            elif choice == '3':
                tracking_mode()
            elif choice == '4':
                print("系统退出")
                break
            else:
                print("无效输入，请输入 1-4")
        except KeyboardInterrupt:
            print("\n系统退出")
            break
        except Exception as e:
            print(f"发生错误: {e}")
            stop()

if __name__ == "__main__":
    try:
        main()
    finally:
        stop()