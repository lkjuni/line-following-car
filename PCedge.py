"""
PC 端视觉巡线程序。

- 从 K10 的 /stream 读取 MJPEG 视频流
- 用 HSV 阈值提取红线并计算中心位置
- 用 PD 控制器计算转向差速
- 通过 HTTP 将控制量发送给 K10，不使用 MQTT
"""

from __future__ import annotations

import threading
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import requests


# ==================== 网络配置 ====================

K10_IP = "192.168.238.14"

STREAM_URL = f"http://{K10_IP}/stream"
CONTROL_BASE_URL = f"http://{K10_IP}:81"

# 控制请求最多发送 20 次/秒
CONTROL_INTERVAL = 0.05
CONTROL_CONNECT_TIMEOUT = 0.15
CONTROL_READ_TIMEOUT = 0.25

# 视频流连接及读取超时
STREAM_CONNECT_TIMEOUT = 3.0
STREAM_READ_TIMEOUT = 5.0


# ==================== 视觉参数 ====================

LP1 = 160
LP2 = 200
LCS = 255

# 红色在 HSV 色环首尾各占一段
LOWER_RED_1 = np.array([0, 100, 100], dtype=np.uint8)
UPPER_RED_1 = np.array([10, 255, 255], dtype=np.uint8)

LOWER_RED_2 = np.array([160, 100, 100], dtype=np.uint8)
UPPER_RED_2 = np.array([180, 255, 255], dtype=np.uint8)


# ==================== 控制参数 ====================

KP = 0.5
KD = 1.0

BASE_SPEED = 300
MAX_SPEED_GAP = 400

old_err = 0


class K10HttpController:
    """
    K10 HTTP 控制器。

    控制请求在后台线程发送，避免网络延迟阻塞视频处理。
    发送槽中只保留最新指令，旧指令不会堆积。
    """

    def __init__(self, base_url: str) -> None:
        self.base_url = base_url.rstrip("/")

        self._condition = threading.Condition()
        self._pending: Optional[Tuple[str, int, int]] = None
        self._closed = False
        self._connected = False
        self._last_error = "尚未连接"
        self._last_error_print = 0.0

        self._session = requests.Session()

        # K10 是局域网设备，不经过系统代理
        self._session.trust_env = False

        self._thread = threading.Thread(
            target=self._worker,
            name="k10-http-control",
            daemon=True,
        )
        self._thread.start()

    @property
    def connected(self) -> bool:
        with self._condition:
            return self._connected

    @property
    def last_error(self) -> str:
        with self._condition:
            return self._last_error

    def submit_drive(self, base_speed: int, speed_gap: int) -> None:
        """提交行驶命令。"""
        command = ("drive", int(base_speed), int(speed_gap))

        with self._condition:
            if self._closed:
                return

            self._pending = command
            self._condition.notify()

    def submit_stop(self) -> None:
        """提交停车命令。"""
        with self._condition:
            if self._closed:
                return

            self._pending = ("stop", 0, 0)
            self._condition.notify()

    def _worker(self) -> None:
        while True:
            with self._condition:
                while self._pending is None and not self._closed:
                    self._condition.wait()

                if self._closed:
                    return

                command = self._pending
                self._pending = None

            assert command is not None

            action, base_speed, speed_gap = command

            try:
                if action == "drive":
                    response = self._session.get(
                        f"{self.base_url}/control",
                        params={
                            "base": base_speed,
                            "gap": speed_gap,
                        },
                        timeout=(
                            CONTROL_CONNECT_TIMEOUT,
                            CONTROL_READ_TIMEOUT,
                        ),
                    )
                else:
                    response = self._session.get(
                        f"{self.base_url}/stop",
                        timeout=(
                            CONTROL_CONNECT_TIMEOUT,
                            CONTROL_READ_TIMEOUT,
                        ),
                    )

                response.raise_for_status()
                self._record_success()

            except requests.RequestException as exc:
                self._record_failure(str(exc))

    def _record_success(self) -> None:
        with self._condition:
            recovered = not self._connected
            self._connected = True
            self._last_error = ""

        if recovered:
            print(f"[HTTP] 已连接 K10 控制服务: {self.base_url}")

    def _record_failure(self, message: str) -> None:
        now = time.monotonic()

        with self._condition:
            self._connected = False
            self._last_error = message

            should_print = now - self._last_error_print >= 1.0

            if should_print:
                self._last_error_print = now

        if should_print:
            print(f"[HTTP] 控制请求失败: {message}")

    def close(self) -> None:
        """停止后台线程并尝试发送最后一条停车命令。"""
        with self._condition:
            if self._closed:
                return

            self._closed = True
            self._pending = None
            self._condition.notify_all()

        self._thread.join(timeout=0.5)

        try:
            response = self._session.get(
                f"{self.base_url}/stop",
                timeout=(
                    CONTROL_CONNECT_TIMEOUT,
                    CONTROL_READ_TIMEOUT,
                ),
            )
            response.raise_for_status()
            print("[HTTP] 已发送停车命令")

        except requests.RequestException:
            # 即使请求失败，K10 的看门狗也会在超时后停车
            print("[HTTP] 停车请求未送达，等待 K10 看门狗停车")

        finally:
            self._session.close()


def reset_pid() -> None:
    global old_err
    old_err = 0


def pid(x: int, img_w: int) -> int:
    """根据目标中心相对画面中心的偏移计算差速。"""
    global old_err

    mid = img_w // 2
    err = x - mid

    offset = err * KP + (err - old_err) * KD
    old_err = err

    return -1*int(round(offset))


def process_frame(
    img: np.ndarray,
) -> Tuple[np.ndarray, Optional[int]]:
    """
    提取红色掩码。

    当两条检测线均检测到红线时，返回两条检测线中心的平均值。
    """
    h, _ = img.shape[:2]

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, LOWER_RED_1, UPPER_RED_1)
    mask2 = cv2.inRange(hsv, LOWER_RED_2, UPPER_RED_2)

    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), dtype=np.uint8)

    mask = cv2.dilate(
        mask,
        kernel,
        iterations=4,
    )

    mask = cv2.erode(
        mask,
        kernel,
        iterations=4,
    )

    if not (0 <= LP1 < h and 0 <= LP2 < h):
        return mask, None

    centers = []

    for y in (LP1, LP2):
        indices = np.flatnonzero(mask[y, :] == LCS)

        if indices.size == 0:
            return mask, None

        left = int(indices[0])
        right = int(indices[-1])

        centers.append((left + right) // 2)

    center = int(sum(centers) / len(centers))

    return mask, center


def iter_mjpeg_frames(response: requests.Response):
    """从 MJPEG 字节流中持续提取 JPEG 图像。"""
    buffer = bytearray()
    max_buffer_size = 2 * 1024 * 1024

    for chunk in response.iter_content(chunk_size=4096):
        if not chunk:
            continue

        buffer.extend(chunk)

        while True:
            start = buffer.find(b"\xff\xd8")

            if start < 0:
                if len(buffer) > max_buffer_size:
                    del buffer[:-2]
                break

            end = buffer.find(b"\xff\xd9", start + 2)

            if end < 0:
                if start > 0:
                    del buffer[:start]
                break

            jpg_data = bytes(buffer[start:end + 2])
            del buffer[:end + 2]

            yield jpg_data


def main() -> None:
    controller = K10HttpController(CONTROL_BASE_URL)

    # 程序启动时先请求停车
    controller.submit_stop()

    cv2.namedWindow(
        "live transmission",
        cv2.WINDOW_AUTOSIZE,
    )

    cv2.namedWindow(
        "mask",
        cv2.WINDOW_AUTOSIZE,
    )

    last_control_time = 0.0
    line_was_visible = False

    try:
        with requests.get(
            STREAM_URL,
            stream=True,
            timeout=(
                STREAM_CONNECT_TIMEOUT,
                STREAM_READ_TIMEOUT,
            ),
        ) as response:
            response.raise_for_status()

            print(f"[视频] 已连接: {STREAM_URL}")

            for jpg_data in iter_mjpeg_frames(response):
                img_np = np.frombuffer(
                    jpg_data,
                    dtype=np.uint8,
                )

                img = cv2.imdecode(
                    img_np,
                    cv2.IMREAD_COLOR,
                )

                if img is None:
                    continue

                h, w = img.shape[:2]

                mask, center = process_frame(img)

                if center is not None:
                    pid_offset = pid(center, w)

                    pid_offset = max(
                        -MAX_SPEED_GAP,
                        min(MAX_SPEED_GAP, pid_offset),
                    )

                    now = time.monotonic()

                    if now - last_control_time >= CONTROL_INTERVAL:
                        controller.submit_drive(
                            BASE_SPEED,
                            pid_offset,
                        )

                        last_control_time = now

                    line_was_visible = True

                else:
                    # 丢线立即停车
                    pid_offset = 0
                    reset_pid()

                    if line_was_visible:
                        controller.submit_stop()

                    line_was_visible = False
                    last_control_time = 0.0

                # 绘制两条检测线
                cv2.line(
                    img,
                    (0, LP1),
                    (w - 1, LP1),
                    (0, 255, 0),
                    3,
                )

                cv2.line(
                    img,
                    (0, LP2),
                    (w - 1, LP2),
                    (0, 255, 255),
                    3,
                )

                # 绘制画面中心线
                cv2.line(
                    img,
                    (w // 2, 0),
                    (w // 2, h - 1),
                    (255, 0, 0),
                    2,
                )

                # 绘制红线中心
                if center is not None:
                    mid_y = (LP1 + LP2) // 2

                    cv2.drawMarker(
                        img,
                        (center, mid_y),
                        (0, 0, 255),
                        cv2.MARKER_CROSS,
                        20,
                        5,
                    )

                http_ok = controller.connected

                cv2.putText(
                    img,
                    f"Center: {center}",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    img,
                    f"PID gap: {pid_offset:+d}",
                    (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2,
                )

                cv2.putText(
                    img,
                    f"HTTP: {'OK' if http_ok else 'OFF'}",
                    (10, 75),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (
                        (0, 255, 0)
                        if http_ok
                        else (0, 0, 255)
                    ),
                    2,
                )

                cv2.imshow("mask", mask)
                cv2.imshow("live transmission", img)

                status = (
                    "OK"
                    if center is not None
                    else "LOST"
                )

                print(
                    f"[{status}] "
                    f"center={center}, "
                    f"speed_gap={pid_offset:+d}"
                )

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except requests.RequestException as exc:
        print(f"[错误] K10 视频流连接失败: {exc}")

    except KeyboardInterrupt:
        print("\n用户中断")

    finally:
        controller.close()
        cv2.destroyAllWindows()
        print("程序结束")


if __name__ == "__main__":
    main()