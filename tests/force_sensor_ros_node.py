#!/usr/bin/env python3
"""
中科米点 MIOS-M-Y60-H26.5 六维力传感器 → ROS1 WrenchStamped 发布节点
================================================================
- 协议：以厂商官方参考代码 force_sensor.py 为准（手册有出入）
- 帧头 0xAA 0x55  /  帧尾 0x0D 0x0A
- 模式：1kHz 推送（命令 0x02）+ 数据源切换为矩阵滤波（命令 0x33）
- 单位：传感器返回 kg / kg·m，本节点默认换算为 N / N·m 发布

发布话题：
    geometry_msgs/WrenchStamped   /force_sensor/wrench

用法：
    # 直接跑（默认参数在文件顶部 USER PARAMS 区改）
    rosrun your_pkg force_sensor_ros_node.py

    # 或者用 rosparam 覆盖（优先级高于 USER PARAMS）
    rosrun your_pkg force_sensor_ros_node.py _port:=/dev/ttyUSB0 _topic:=/my_wrench

依赖：
    pip install pyserial numpy
    apt install ros-noetic-geometry-msgs
"""

import struct
import time
from collections import deque
from typing import Optional, Tuple

import numpy as np
import rospy
import serial
from geometry_msgs.msg import WrenchStamped

# =============================================================================
# USER PARAMS（默认值，可被 rosparam ~param_name 覆盖）
# =============================================================================

# ---- 串口 ----
SERIAL_PORT     = "/dev/ttyUSB0"          # Linux: "/dev/ttyUSB0"；Windows: "COM6"
BAUD_RATE       = 460800
SERIAL_TIMEOUT  = 0.5

# ---- 数据流 ----
# auto: 优先 0x02 1kHz 推送，启动失败后自动降级到 0x33/0x03 单帧轮询。
# stream: 只使用 0x02 推送；poll: 只使用 DATA_SOURCE_CMD 单帧轮询。
READ_MODE       = "auto"          # "auto" / "stream" / "poll"
DATA_SOURCE_CMD = 0x33            # 0x33 矩阵滤波 / 0x34 原始 / 0x03 单帧（不推荐用作数据源）
POLL_HZ         = 200.0            # 单帧轮询发布频率，实际频率还受串口响应时间影响
STREAM_START_FAILS = 6             # auto 模式启动阶段连续失败 N 次后降级到 poll

# ---- 启动行为 ----
TARE_ON_START   = True           # 启动时发清零命令 0x30
WARMUP_S        = 0.0             # 启动后等待秒数（手册建议正式实验预热 1 小时）

# ---- 输出 ----
OUTPUT_UNITS    = "N"             # "N" → 牛顿/牛·米；"kgf" → 原始 kg
GRAVITY         = 9.80665

# ---- Host 侧滤波（默认关闭，因为 0x33 已是传感器内部矩阵滤波）----
FILTER_ENABLE     = False
FILTER_TYPE       = "ema"         # "ema" 或 "mean"
FILTER_EMA_ALPHA  = 0.05          # 1kHz 下 α=0.05 ≈ 8Hz 截止
FILTER_WIN_SIZE   = 20

# ---- ROS ----
ROS_TOPIC       = "/force_sensor/wrench"
FRAME_ID        = "force_sensor"  # tf frame_id；下游 tf 监听器要对齐
PUB_QUEUE_SIZE  = 100             # 1kHz 推送下不能太小，否则订阅慢会丢消息

# ---- 调试 ----
PRINT_RAW_BYTES = False           # 谨慎开启：1kHz 下会刷爆终端
LOG_EVERY_N     = 1000            # 每 N 帧 loginfo 一次；1kHz 下 1000 ≈ 1s
TIMEOUT_REPORT  = 200             # 累计 N 次失败才 warn 一次
RX_CHUNK_SIZE   = 256             # 每次最多从串口搬运的字节数
RX_BUFFER_LIMIT = 4096            # 防止长期错帧时缓冲区无限增长

# =============================================================================
# 协议常量
# =============================================================================

FRAME_HEAD   = b"\xAA\x55"
FRAME_TAIL   = b"\x0D\x0A"
FRAME_LEN_RX = 2 + 1 + 24 + 2     # 头 + cmd + 数据 + 尾 = 29

CMD_STOP            = 0x01
CMD_STREAM_1KHZ     = 0x02
CMD_SINGLE_SHOT     = 0x03
CMD_AUTO_START      = 0x06
CMD_TARE            = 0x30
CMD_MATRIX_FILTERED = 0x33
CMD_RAW_DATA        = 0x34

DATA_CMDS = frozenset({CMD_STREAM_1KHZ, CMD_SINGLE_SHOT, CMD_AUTO_START,
                       CMD_MATRIX_FILTERED, CMD_RAW_DATA})


# =============================================================================
# 协议工具
# =============================================================================

def build_frame(cmd: int, payload: bytes = b"") -> bytes:
    return FRAME_HEAD + bytes([cmd]) + payload + FRAME_TAIL


class FrameReadResult:
    """read_frame 的轻量返回对象，避免异常路径在 1kHz 下反复分配大对象。"""
    __slots__ = ("cmd", "data", "reason", "raw")

    def __init__(
        self,
        cmd: Optional[int] = None,
        data: Optional[bytes] = None,
        reason: str = "",
        raw: bytes = b"",
    ):
        self.cmd = cmd
        self.data = data
        self.reason = reason
        self.raw = raw


class SerialFrameReader:
    """
    串口流帧解析器。

    这里按 diag.py 验证过的方式工作：先收原始字节，再在缓冲区里搜索
    AA 55 帧头，凑满 29 字节后检查 0D 0A 帧尾。相比逐字段阻塞读取，
    这种滑动同步在 1kHz 推送中遇到错位、残留应答或偶发丢字节时更容易恢复。
    """

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buf = bytearray()

    def _read_available(self) -> bool:
        n_waiting = getattr(self.ser, "in_waiting", 0)
        n_read = max(1, min(RX_CHUNK_SIZE, n_waiting or FRAME_LEN_RX))
        chunk = self.ser.read(n_read)
        if not chunk:
            return False
        self.buf.extend(chunk)
        if len(self.buf) > RX_BUFFER_LIMIT:
            del self.buf[:-RX_BUFFER_LIMIT]
        return True

    def _next_from_buffer(self) -> Optional[FrameReadResult]:
        while True:
            head_idx = self.buf.find(FRAME_HEAD)
            if head_idx < 0:
                # 保留最后 1 字节，避免单独的 0xAA 被切在两次读取之间。
                if len(self.buf) > 1:
                    del self.buf[:-1]
                return None
            if head_idx > 0:
                del self.buf[:head_idx]
            if len(self.buf) < FRAME_LEN_RX:
                return None

            raw = bytes(self.buf[:FRAME_LEN_RX])
            if raw[-2:] != FRAME_TAIL:
                # 当前 AA 55 不是完整有效帧头，丢掉第一个 AA 后继续找下一帧。
                del self.buf[0]
                return FrameReadResult(reason="bad_tail", raw=raw)

            del self.buf[:FRAME_LEN_RX]
            cmd = raw[2]
            if cmd not in DATA_CMDS:
                return FrameReadResult(cmd=cmd, reason="non_data_cmd", raw=raw)
            return FrameReadResult(cmd=cmd, data=raw[3:27], raw=raw)

    def read_frame(self) -> FrameReadResult:
        deadline = time.time() + SERIAL_TIMEOUT
        last_error: Optional[FrameReadResult] = None

        while time.time() < deadline:
            result = self._next_from_buffer()
            if result is not None:
                if result.data is not None:
                    return result
                last_error = result
                continue
            if not self._read_available():
                continue
            result = self._next_from_buffer()
            if result is not None:
                if result.data is not None:
                    return result
                last_error = result

        if last_error is not None:
            return last_error
        return FrameReadResult(reason="timeout")


def parse_data(data: bytes) -> np.ndarray:
    """24 字节 → 6 个 float (Fx Fy Fz Mx My Mz)，按 OUTPUT_UNITS 换算单位。"""
    if len(data) != 24:
        raise ValueError(f"数据域长度异常：{len(data)}")
    vals = np.array(struct.unpack("<6f", data), dtype=float)
    if OUTPUT_UNITS == "N":
        vals = vals * GRAVITY    # kg→N 与 kg·m→N·m 用同一系数
    return vals


# =============================================================================
# Host 侧滤波（可选）
# =============================================================================

class EMAFilter:
    def __init__(self, alpha: float = 0.05):
        if not (0.0 < alpha <= 1.0):
            raise ValueError(f"alpha 不合法: {alpha}")
        self.alpha = alpha
        self._y: Optional[np.ndarray] = None

    def __call__(self, x: np.ndarray) -> np.ndarray:
        if self._y is None:
            self._y = x.copy()
        else:
            self._y = self.alpha * x + (1.0 - self.alpha) * self._y
        return self._y.copy()


class MeanFilter:
    def __init__(self, window: int = 20):
        if window < 1:
            raise ValueError(f"window 必须 >= 1，收到 {window}")
        self.buf: deque = deque(maxlen=window)

    def __call__(self, x: np.ndarray) -> np.ndarray:
        self.buf.append(x.copy())
        return np.mean(np.stack(self.buf, axis=0), axis=0)


def make_filter():
    if not FILTER_ENABLE:
        return None
    if FILTER_TYPE == "ema":
        return EMAFilter(alpha=FILTER_EMA_ALPHA)
    if FILTER_TYPE == "mean":
        return MeanFilter(window=FILTER_WIN_SIZE)
    raise ValueError(f"未知的 FILTER_TYPE: {FILTER_TYPE}")


def filter_label() -> str:
    if not FILTER_ENABLE:
        return "off"
    if FILTER_TYPE == "ema":
        return f"ema(α={FILTER_EMA_ALPHA})"
    if FILTER_TYPE == "mean":
        return f"mean(N={FILTER_WIN_SIZE})"
    return FILTER_TYPE


def datasource_label() -> str:
    return {
        CMD_SINGLE_SHOT:     "0x03 单帧",
        CMD_MATRIX_FILTERED: "0x33 矩阵滤波",
        CMD_RAW_DATA:        "0x34 原始",
    }.get(DATA_SOURCE_CMD, f"0x{DATA_SOURCE_CMD:02X}")


# =============================================================================
# rosparam 覆盖
# =============================================================================

def load_rosparams():
    global SERIAL_PORT, BAUD_RATE, SERIAL_TIMEOUT
    global READ_MODE, DATA_SOURCE_CMD, POLL_HZ, STREAM_START_FAILS
    global TARE_ON_START, WARMUP_S, OUTPUT_UNITS
    global FILTER_ENABLE, FILTER_TYPE, FILTER_EMA_ALPHA, FILTER_WIN_SIZE
    global ROS_TOPIC, FRAME_ID, PUB_QUEUE_SIZE
    global PRINT_RAW_BYTES, LOG_EVERY_N, TIMEOUT_REPORT
    global RX_CHUNK_SIZE, RX_BUFFER_LIMIT

    SERIAL_PORT      = rospy.get_param("~port", SERIAL_PORT)
    BAUD_RATE        = int(rospy.get_param("~baudrate", BAUD_RATE))
    SERIAL_TIMEOUT   = float(rospy.get_param("~timeout", SERIAL_TIMEOUT))

    READ_MODE        = str(rospy.get_param("~read_mode", READ_MODE)).lower()
    if READ_MODE not in ("auto", "stream", "poll"):
        raise ValueError(f"未知的 READ_MODE: {READ_MODE}")
    ds = rospy.get_param("~data_source_cmd", DATA_SOURCE_CMD)
    DATA_SOURCE_CMD  = int(ds, 0) if isinstance(ds, str) else int(ds)
    POLL_HZ          = float(rospy.get_param("~poll_hz", POLL_HZ))
    STREAM_START_FAILS = int(rospy.get_param("~stream_start_fails", STREAM_START_FAILS))

    TARE_ON_START    = bool(rospy.get_param("~tare_on_start", TARE_ON_START))
    WARMUP_S         = float(rospy.get_param("~warmup_s", WARMUP_S))
    OUTPUT_UNITS     = rospy.get_param("~output_units", OUTPUT_UNITS)

    FILTER_ENABLE    = bool(rospy.get_param("~filter_enable", FILTER_ENABLE))
    FILTER_TYPE      = rospy.get_param("~filter_type", FILTER_TYPE)
    FILTER_EMA_ALPHA = float(rospy.get_param("~filter_ema_alpha", FILTER_EMA_ALPHA))
    FILTER_WIN_SIZE  = int(rospy.get_param("~filter_win_size", FILTER_WIN_SIZE))

    ROS_TOPIC        = rospy.get_param("~topic", ROS_TOPIC)
    FRAME_ID         = rospy.get_param("~frame_id", FRAME_ID)
    PUB_QUEUE_SIZE   = int(rospy.get_param("~pub_queue_size", PUB_QUEUE_SIZE))

    PRINT_RAW_BYTES  = bool(rospy.get_param("~print_raw_bytes", PRINT_RAW_BYTES))
    LOG_EVERY_N      = int(rospy.get_param("~log_every_n", LOG_EVERY_N))
    TIMEOUT_REPORT   = int(rospy.get_param("~timeout_report", TIMEOUT_REPORT))
    RX_CHUNK_SIZE    = int(rospy.get_param("~rx_chunk_size", RX_CHUNK_SIZE))
    RX_BUFFER_LIMIT  = int(rospy.get_param("~rx_buffer_limit", RX_BUFFER_LIMIT))


# =============================================================================
# 串口生命周期
# =============================================================================

def open_sensor() -> serial.Serial:
    ser = serial.Serial(
        port=SERIAL_PORT, baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=SERIAL_TIMEOUT,
    )
    ser.reset_input_buffer(); ser.reset_output_buffer()
    return ser


def stop_stream(ser: serial.Serial) -> None:
    ser.write(build_frame(CMD_STOP))
    time.sleep(0.05)
    ser.reset_input_buffer()


def tare(ser: serial.Serial) -> None:
    rospy.loginfo("发送清零命令 (0x30) ...")
    ser.write(build_frame(CMD_TARE))
    time.sleep(0.2)
    ser.reset_input_buffer()


def start_stream(ser: serial.Serial) -> None:
    """
    启动 1kHz 推送：
      1) 先发 DATA_SOURCE_CMD（0x33 矩阵滤波 / 0x34 原始）切数据源
      2) 短暂等待传感器响应、清掉缓冲
      3) 再发 0x02 启动 1kHz 推送
    顺序很关键——反了等于推送默认数据。
    """
    if DATA_SOURCE_CMD in (CMD_MATRIX_FILTERED, CMD_RAW_DATA):
        rospy.loginfo(f"切换数据源 → {datasource_label()}")
        ser.write(build_frame(DATA_SOURCE_CMD))
        time.sleep(0.1)
        ser.reset_input_buffer()
    rospy.loginfo("启动 1kHz 推送 (0x02)")
    ser.write(build_frame(CMD_STREAM_1KHZ))


def poll_command() -> int:
    if DATA_SOURCE_CMD in (CMD_MATRIX_FILTERED, CMD_RAW_DATA, CMD_SINGLE_SHOT):
        return DATA_SOURCE_CMD
    return CMD_SINGLE_SHOT


# =============================================================================
# 主循环
# =============================================================================

def make_wrench_msg(vals: np.ndarray, stamp) -> WrenchStamped:
    msg = WrenchStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = FRAME_ID
    msg.wrench.force.x  = float(vals[0])
    msg.wrench.force.y  = float(vals[1])
    msg.wrench.force.z  = float(vals[2])
    msg.wrench.torque.x = float(vals[3])
    msg.wrench.torque.y = float(vals[4])
    msg.wrench.torque.z = float(vals[5])
    return msg


def publish_frame(data: bytes, filt, pub: rospy.Publisher) -> np.ndarray:
    vals_raw = parse_data(data)
    vals_pub = filt(vals_raw) if filt is not None else vals_raw
    pub.publish(make_wrench_msg(vals_pub, rospy.Time.now()))
    return vals_pub


def run_streaming(ser: serial.Serial, pub: rospy.Publisher) -> bool:
    """1kHz 流模式：传感器主动推送，主循环只读+发布，不用 rospy.Rate 限流。"""
    filt = make_filter()
    start_stream(ser)
    reader = SerialFrameReader(ser)

    frame_idx = 0
    err_count = 0
    err_by_reason = {}
    t_log = time.time()

    while not rospy.is_shutdown():
        result = reader.read_frame()
        if result.data is None:
            err_count += 1
            reason = result.reason or "unknown"
            err_by_reason[reason] = err_by_reason.get(reason, 0) + 1
            if err_count % TIMEOUT_REPORT == 1:
                detail = f" raw={result.raw.hex(' ').upper()}" if result.raw and PRINT_RAW_BYTES else ""
                rospy.logwarn(
                    f"帧读取超时/校验失败（累计 {err_count}, 原因 {err_by_reason}）{detail}"
                )
            if READ_MODE == "auto" and frame_idx == 0 and err_count >= STREAM_START_FAILS:
                rospy.logwarn(
                    f"0x02 流模式启动后连续 {err_count} 次未读到有效帧，降级为单帧轮询"
                )
                return False
            continue

        data = result.data
        if PRINT_RAW_BYTES:
            rospy.loginfo(f"[RAW] {result.raw.hex(' ').upper()}")

        vals_pub = publish_frame(data, filt, pub)

        frame_idx += 1
        if LOG_EVERY_N > 0 and frame_idx % LOG_EVERY_N == 0:
            now = time.time()
            actual_hz = LOG_EVERY_N / (now - t_log) if now > t_log else 0.0
            t_log = now
            rospy.loginfo(
                f"[{frame_idx:7d}] @{actual_hz:6.1f}Hz "
                f"Fx={vals_pub[0]:+7.3f} Fy={vals_pub[1]:+7.3f} Fz={vals_pub[2]:+7.3f} "
                f"Mx={vals_pub[3]:+7.4f} My={vals_pub[4]:+7.4f} Mz={vals_pub[5]:+7.4f}"
            )
    return True


def run_polling(ser: serial.Serial, pub: rospy.Publisher) -> None:
    """单帧轮询模式：每次发送 0x33/0x03，按 diag.py 的方式等一帧响应。"""
    filt = make_filter()
    reader = SerialFrameReader(ser)
    cmd = poll_command()
    rate = rospy.Rate(POLL_HZ) if POLL_HZ > 0 else None

    frame_idx = 0
    err_count = 0
    err_by_reason = {}
    t_log = time.time()

    rospy.loginfo(f"启动单帧轮询 ({datasource_label()}) @ 目标 {POLL_HZ:g}Hz")
    while not rospy.is_shutdown():
        ser.write(build_frame(cmd))
        result = reader.read_frame()
        if result.data is None:
            err_count += 1
            reason = result.reason or "unknown"
            err_by_reason[reason] = err_by_reason.get(reason, 0) + 1
            if err_count % TIMEOUT_REPORT == 1:
                detail = f" raw={result.raw.hex(' ').upper()}" if result.raw and PRINT_RAW_BYTES else ""
                rospy.logwarn(
                    f"轮询帧读取失败（累计 {err_count}, 原因 {err_by_reason}）{detail}"
                )
            if rate is not None:
                rate.sleep()
            continue

        if PRINT_RAW_BYTES:
            rospy.loginfo(f"[RAW] {result.raw.hex(' ').upper()}")

        vals_pub = publish_frame(result.data, filt, pub)

        frame_idx += 1
        if LOG_EVERY_N > 0 and frame_idx % LOG_EVERY_N == 0:
            now = time.time()
            actual_hz = LOG_EVERY_N / (now - t_log) if now > t_log else 0.0
            t_log = now
            rospy.loginfo(
                f"[{frame_idx:7d}] @{actual_hz:6.1f}Hz "
                f"Fx={vals_pub[0]:+7.3f} Fy={vals_pub[1]:+7.3f} Fz={vals_pub[2]:+7.3f} "
                f"Mx={vals_pub[3]:+7.4f} My={vals_pub[4]:+7.4f} Mz={vals_pub[5]:+7.4f}"
            )
        if rate is not None:
            rate.sleep()


# =============================================================================
# 入口
# =============================================================================

def main() -> None:
    rospy.init_node("mios_force_sensor", anonymous=False)
    load_rosparams()

    rospy.loginfo(f"打开串口 {SERIAL_PORT} @ {BAUD_RATE} bps")
    ser = open_sensor()
    pub = rospy.Publisher(ROS_TOPIC, WrenchStamped, queue_size=PUB_QUEUE_SIZE)

    try:
        # 进来先发停止命令，确保传感器不在某个残留的推送状态
        stop_stream(ser)

        if TARE_ON_START:
            tare(ser)
        if WARMUP_S > 0:
            rospy.loginfo(f"预热 {WARMUP_S} 秒 ...")
            time.sleep(WARMUP_S)

        rospy.loginfo(
            f"模式: {READ_MODE}   数据源: {datasource_label()}   "
            f"host 滤波: {filter_label()}   单位: {OUTPUT_UNITS}   "
            f"话题: {ROS_TOPIC}   frame_id: {FRAME_ID}"
        )

        if READ_MODE == "poll":
            run_polling(ser, pub)
        else:
            stream_ok = run_streaming(ser, pub)
            if not stream_ok and not rospy.is_shutdown():
                stop_stream(ser)
                run_polling(ser, pub)

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"运行异常: {e}")
    finally:
        try:
            stop_stream(ser)
        except Exception:
            pass
        ser.close()
        rospy.loginfo("串口已关闭")


if __name__ == "__main__":
    main()
