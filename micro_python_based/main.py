# -*- coding: UTF-8 -*-

import cv2
import numpy as np
import time
#import my_motor as mt
from unihiker_k10 import pin


# ==================================================
# 一、TB6612 电机引脚定义
# ==================================================

AIN1 = pin(2)   # 左轮方向1
AIN2 = pin(3)   # 左轮方向2
PWMA = pin(0)   # 左轮PWM

BIN1 = pin(4)   # 右轮方向1
BIN2 = pin(5)   # 右轮方向2
PWMB = pin(1)   # 右轮PWM

STBY = pin(6)   # TB6612使能


# 开启TB6612
STBY.write_digital(1)


# ==================================================
# 二、电机控制函数
# ==================================================

def limit_pwm(value):
    """
    将PWM限制在0~1000。
    """
    return max(0, min(1000, int(value)))


def left_motor(speed, direction):
    """
    控制左轮。

    speed:
        0~1000

    direction:
        1：前进
        0：后退
    """

    speed = limit_pwm(speed)

    if direction == 1:
        AIN1.write_digital(1)
        AIN2.write_digital(0)
    else:
        AIN1.write_digital(0)
        AIN2.write_digital(1)

    PWMA.write_analog(
        value=speed,
        freq=1000
    )


def right_motor(speed, direction):
    """
    控制右轮。

    speed:
        0~1000

    direction:
        1：前进
        0：后退
    """

    speed = limit_pwm(speed)

    if direction == 1:
        BIN1.write_digital(1)
        BIN2.write_digital(0)
    else:
        BIN1.write_digital(0)
        BIN2.write_digital(1)

    PWMB.write_analog(
        value=speed,
        freq=1000
    )


def stop():
    """
    停止左右电机。
    """

    PWMA.write_analog(
        value=0,
        freq=1000
    )

    PWMB.write_analog(
        value=0,
        freq=1000
    )


def common_move(speed, direction, speed_gap):
    """
    控制小车直行或差速转弯。

    speed:
        左轮速度，范围0~1000。

    direction:
        1：前进
        0：后退

    speed_gap:
        右轮相对于左轮的速度差。

        speed_gap > 0：
            右轮比左轮快，小车向左转。

        speed_gap < 0：
            右轮比左轮慢，小车向右转。

        speed_gap = 0：
            左右轮速度相同，小车直行。
    """

    left_speed = limit_pwm(speed)
    right_speed = limit_pwm(speed + speed_gap)

    left_motor(left_speed, direction)
    right_motor(right_speed, direction)


# ==================================================
# 三、寻找一条检测线上的红色区域中心
# ==================================================

def find_line_center(row):
    """
    row是一行二值图像。

    红色区域在mask中为255。
    没有检测到红色时返回None。
    """

    positions = np.where(row == 255)[0]

    if len(positions) == 0:
        return None

    left_position = positions[0]
    right_position = positions[-1]

    center = int(
        (left_position + right_position) / 2
    )

    return center


# ==================================================
# 四、图像处理：检测红色轨迹中心
# ==================================================

def process_frame(img, line_y1, line_y2):
    """
    检测两条水平线上的红色轨迹。

    返回：
        mask：
            红色二值掩码。

        center：
            两条检测线红色中心的平均值。
            未检测到时为None。

        center1：
            第一条检测线上的红色中心。

        center2：
            第二条检测线上的红色中心。
    """

    # BGR转换到HSV
    hsv = cv2.cvtColor(
        img,
        cv2.COLOR_BGR2HSV
    )

    # 红色在HSV色环两端，因此使用两个范围
    lower_red1 = np.array([0, 100, 100])
    upper_red1 = np.array([10, 255, 255])

    lower_red2 = np.array([170, 100, 100])
    upper_red2 = np.array([180, 255, 255])

    mask1 = cv2.inRange(
        hsv,
        lower_red1,
        upper_red1
    )

    mask2 = cv2.inRange(
        hsv,
        lower_red2,
        upper_red2
    )

    mask = cv2.bitwise_or(mask1, mask2)

    # 形态学处理，消除噪点、连接断开的红线
    kernel = np.ones(
        (5, 5),
        np.uint8
    )

    # 先膨胀再腐蚀，填补轨迹内部的小缺口
    mask = cv2.dilate(
        mask,
        kernel,
        iterations=2
    )

    mask = cv2.erode(
        mask,
        kernel,
        iterations=2
    )

    height, width = mask.shape

    # 防止检测线超出图像范围
    if line_y1 < 0 or line_y1 >= height:
        return mask, None, None, None

    if line_y2 < 0 or line_y2 >= height:
        return mask, None, None, None

    center1 = find_line_center(mask[line_y1])
    center2 = find_line_center(mask[line_y2])

    valid_centers = []

    if center1 is not None:
        valid_centers.append(center1)

    if center2 is not None:
        valid_centers.append(center2)

    if len(valid_centers) == 0:
        center = None
    else:
        center = int(
            sum(valid_centers) / len(valid_centers)
        )

    return mask, center, center1, center2


# ==================================================
# 五、PD控制器
# ==================================================

kp = 1.2
kd = 2.0

old_error = 0


def pid(line_center, image_center):
    """
    根据红线中心和图像中心计算speed_gap。

    line_center > image_center：
        红线在右边，小车需要右转。
        右轮应减速，因此speed_gap为负数。

    line_center < image_center：
        红线在左边，小车需要左转。
        右轮应加速，因此speed_gap为正数。
    """

    global old_error

    error = line_center - image_center

    differential = error - old_error

    correction = (
        error * kp
        + differential * kd
    )

    old_error = error

    # 取负号，使转向方向符合common_move的语义
    speed_gap = -int(correction)

    # 限制最大差速，避免转向过于剧烈
    speed_gap = max(
        -450,
        min(450, speed_gap)
    )

    return speed_gap


# ==================================================
# 六、主程序
# ==================================================

def main():

    global old_error

    # --------------------------
    # 运行参数
    # --------------------------

    # 正常循迹时左轮的基础速度
    base_speed = 450

    # 丢线时单轮寻找轨迹的速度
    search_speed = 350

    # 连续丢线多少帧后开始转弯寻找
    search_after_frames = 3

    # 记录连续丢线帧数
    lost_frames = 0

    # 记录最后一次看见红线的位置
    #
    # 1：红线最后在右边
    # -1：红线最后在左边
    # 0：还没有检测到过红线
    last_line_side = 0

    # 摄像头
    camera = cv2.VideoCapture(0)

    # 请求摄像头输出640×480
    camera.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        640
    )

    camera.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        480
    )

    if not camera.isOpened():
        print("摄像头打开失败")
        stop()
        return

    cv2.namedWindow(
        "camera",
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        "camera",
        640,
        480
    )

    cv2.namedWindow(
        "mask",
        cv2.WINDOW_NORMAL
    )

    cv2.resizeWindow(
        "mask",
        640,
        480
    )

    try:

        while True:

            ret, frame = camera.read()

            if not ret or frame is None:
                print("摄像头读取失败")
                stop()
                time.sleep(0.1)
                continue

            height, width = frame.shape[:2]

            # 检测线放在图像下方
            # 使用比例而不是写死320、380，
            # 这样摄像头分辨率变化时不会越界。
            line_y1 = int(height * 0.67)
            line_y2 = int(height * 0.80)

            image_center = width // 2

            mask, center, center1, center2 = process_frame(
                frame,
                line_y1,
                line_y2
            )

            # --------------------------
            # 绘制辅助信息
            # --------------------------

            # 图像中心线
            cv2.line(
                frame,
                (image_center, 0),
                (image_center, height - 1),
                (255, 0, 0),
                2
            )

            # 第一条检测线
            cv2.line(
                frame,
                (0, line_y1),
                (width - 1, line_y1),
                (0, 255, 0),
                2
            )

            # 第二条检测线
            cv2.line(
                frame,
                (0, line_y2),
                (width - 1, line_y2),
                (0, 255, 255),
                2
            )

            # 绘制第一条检测线上的中心点
            if center1 is not None:
                cv2.circle(
                    frame,
                    (center1, line_y1),
                    6,
                    (0, 0, 255),
                    -1
                )

            # 绘制第二条检测线上的中心点
            if center2 is not None:
                cv2.circle(
                    frame,
                    (center2, line_y2),
                    6,
                    (255, 0, 255),
                    -1
                )

            # --------------------------
            # 检测到红线
            # --------------------------

            if center is not None:

                lost_frames = 0

                # 检测到线后正常使用PD控制
                speed_gap = pid(
                    center,
                    image_center
                )

                # PID结果直接作为common_move的speed_gap
                common_move(
                    speed=base_speed,
                    direction=1,
                    speed_gap=speed_gap
                )

                # 记录红线最后所在方向
                if center > image_center:
                    last_line_side = 1

                elif center < image_center:
                    last_line_side = -1

                # 绘制最终使用的轨迹中心
                marker_y = int(
                    (line_y1 + line_y2) / 2
                )

                cv2.drawMarker(
                    frame,
                    (center, marker_y),
                    (0, 0, 255),
                    cv2.MARKER_CROSS,
                    20,
                    3
                )

                left_speed = limit_pwm(base_speed)
                right_speed = limit_pwm(
                    base_speed + speed_gap
                )

                status_text = (
                    f"center={center} "
                    f"gap={speed_gap} "
                    f"L={left_speed} "
                    f"R={right_speed}"
                )

                print(
                    "检测到红线：",
                    "center =", center,
                    "speed_gap =", speed_gap,
                    "left =", left_speed,
                    "right =", right_speed
                )

            # --------------------------
            # 没有检测到红线
            # --------------------------

            else:

                lost_frames += 1

                # 刚刚丢线，先短暂保持直行
                # 防止单帧识别失败造成小车突然转向
                if lost_frames <= search_after_frames:

                    common_move(
                        speed=base_speed,
                        direction=1,
                        speed_gap=0
                    )

                    status_text = "line temporarily lost"

                else:

                    # 最后一次看到红线在右边
                    if last_line_side == 1:

                        # 左轮前进，右轮停止
                        # 小车向右转动寻找红线
                        common_move(
                            speed=search_speed,
                            direction=1,
                            speed_gap=-search_speed
                        )

                        status_text = "searching right"

                    # 最后一次看到红线在左边
                    elif last_line_side == -1:

                        # 左轮停止，右轮前进
                        # common_move(0, 1, search_speed)：
                        #
                        # 左轮速度 = 0
                        # 右轮速度 = 0 + search_speed
                        common_move(
                            speed=0,
                            direction=1,
                            speed_gap=search_speed
                        )

                        status_text = "searching left"

                    # 启动后从未识别到红线
                    else:

                        # 低速向前寻找
                        common_move(
                            speed=250,
                            direction=1,
                            speed_gap=0
                        )

                        status_text = "searching forward"

                print(
                    "未检测到红线：",
                    "lost_frames =", lost_frames,
                    "last_side =", last_line_side
                )

            # --------------------------
            # 显示状态
            # --------------------------

            cv2.putText(
                frame,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 0),
                2
            )

            cv2.imshow(
                "camera",
                frame
            )

            cv2.imshow(
                "mask",
                mask
            )

            key = cv2.waitKey(1) & 0xFF

            # 按a或者q结束
            if key == ord("a") or key == ord("q"):
                break

    except KeyboardInterrupt:
        print("程序被手动停止")

    finally:
        stop()
        camera.release()
        cv2.destroyAllWindows()

        print("电机已经停止")
        print("摄像头已经释放")


if __name__ == "__main__":
    main()