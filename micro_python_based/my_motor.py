from unihiker_k10 import pin
import time


# =====================
# 定义引脚
# =====================
AIN1 = pin(2)   # 左轮方向1
AIN2 = pin(3)   # 左轮方向2
PWMA = pin(0)   # 左轮PWM

BIN1 = pin(4)   # 右轮方向1
BIN2 = pin(8)   # 右轮方向2
PWMB = pin(1)   # 右轮PWM

STBY = pin(6)   # TB6612使能


# =====================
# 初始化
# =====================

# 开启TB6612
STBY.write_digital(1)


# =====================
# 电机控制函数
# =====================

def left_motor(speed, direction):
    """
    speed: 0~1000 PWM占空比
    direction:
        1 前进
        0 后退
    """

    if direction == 1:
        AIN1.write_digital(1)
        AIN2.write_digital(0)
    else:
        AIN1.write_digital(0)
        AIN2.write_digital(1)

    PWMA.write_analog(value=speed, freq=1000)



def right_motor(speed, direction):
    """
    右轮控制
    """

    if direction == 1:
        BIN1.write_digital(1)
        BIN2.write_digital(0)
    else:
        BIN1.write_digital(0)
        BIN2.write_digital(1)

    PWMB.write_analog(value=speed, freq=1000)



def stop():
    """
    停止
    """

    PWMA.write_analog(value=0, freq=1000)
    PWMB.write_analog(value=0, freq=1000)

def common_move(speed, direction,speed_gap):
    """
    直行（前进or后退）
    
    speed: 0~1000 PWM占空比
    direction:
        1 前进
        0 后退
    speed_gap: 0-1000
        正数 左拐
        负数 右拐
    """
    
    left_motor(speed, direction)
    right_motor(speed+speed_gap, direction)




    


# =====================
# 测试动作
# =====================
if (__name__ == "__main__"):
    
    while True:
        print("前进")
        left_motor(600, 1)
        right_motor(600, 1)
        time.sleep(3)


        print("停止")

        stop()

        time.sleep(1)



        print("后退")

        left_motor(600, 0)
        right_motor(600, 0)

        time.sleep(3)



        print("停止")

        stop()

        time.sleep(1)



    print("左转")

    # 左轮慢，右轮快
    left_motor(200, 1)
    right_motor(700, 1)

    time.sleep(2)



    print("右转")

    left_motor(700, 1)
    right_motor(200, 1)

    time.sleep(2)


