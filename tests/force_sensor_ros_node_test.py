#!/usr/bin/env python3
"""
中科米点 MIOS-M-Y60-H26.5 六维力传感器 → ROS1 WrenchStamped 发布节点 (v5.3)
================================================================
所有参数在文件顶部 USER PARAMS 区修改，直接运行，不需要 rosparam。

v5.3 关键发现:
- 本传感器实际帧格式 = 28 字节: AA + cmd + 24 字节数据 + 0D 0A
- 这是手册 (27 字节, FE 尾) 和厂商代码 (29 字节, AA 55 头) 之外的第三种格式
- 现在三种格式都支持，自动检测

启动方法:
    python3 force_sensor_ros_node.py

支持的三种帧格式 (运行时自动检测):
  - binary_28b    : AA + cmd + 24data + 0D 0A    (28字节, 本传感器实际格式) ★
  - binary_manual : AA + cmd + 24data + FE       (27字节, 手册标称格式)
  - binary_vendor : AA 55 + cmd + 24data + 0D 0A (29字节, 厂商代码格式)
  - ascii         : 'channels: x,y,z,a,b,c' 或纯 CSV (debug 模式)

手册命令清单 (4.3.3):
    0x01 停止 / 0x02 1kHz流 / 0x03 单帧 / 0x04 改波特率 / 0x05 查询信息 / 0x06 自动启动
    0x30 清零 / 0x31 退出debug / 0x32 进入debug / 0x33 矩阵滤波 / 0x34 原始数据
"""

import re
import struct
import time
from collections import Counter, deque
from typing import Optional, Tuple

import numpy as np
import rospy
import serial
from geometry_msgs.msg import WrenchStamped

# =============================================================================
# USER PARAMS （这里就是全部配置，改完直接运行）
# =============================================================================

# ---- 串口 ----
SERIAL_PORT     = "/dev/ttyUSB0"
BAUD_RATE       = 460800
SERIAL_TIMEOUT  = 0.5

# ---- 协议格式 ----
COMMAND_FORMAT  = "both"          # "manual" | "vendor" | "both"

# ---- 启动序列 ----
EXIT_DEBUG_ON_START = True
STOP_ON_START       = True
DATA_SOURCE_CMD     = 0x33        # 0x33 矩阵滤波 / 0x34 原始数据
USE_STREAMING       = True
POLL_HZ             = 100.0

# ---- 兼容回退 ----
ALLOW_ASCII_FALLBACK = True

# ---- 其他启动行为 ----
TARE_ON_START   = False
WARMUP_S        = 0.0

# ---- 输出 ----
OUTPUT_UNITS    = "N"             # "N" → 牛顿；"kgf" → 原始 kg
GRAVITY         = 9.80665

# ---- Host 侧滤波 ----
FILTER_ENABLE     = False
FILTER_TYPE       = "ema"
FILTER_EMA_ALPHA  = 0.05
FILTER_WIN_SIZE   = 20

# ---- ROS ----
ROS_TOPIC       = "/force_sensor/wrench"
FRAME_ID        = "force_sensor"
PUB_QUEUE_SIZE  = 100

# ---- 调试 ----
PRINT_RAW        = False
LOG_EVERY_N      = 200            # 1kHz 下 200 帧 ≈ 5Hz 日志
TIMEOUT_REPORT   = 200
DUMP_ON_UNKNOWN  = True
DUMP_BYTES       = 256

# =============================================================================
# 协议常量
# =============================================================================

# 命令发送格式
MANUAL_HEAD    = b"\xAA"
MANUAL_TAIL_TX = b"\x0B\x0C"

VENDOR_HEAD    = b"\xAA\x55"
VENDOR_TAIL    = b"\x0D\x0A"

# 数据帧格式（三种，本传感器实际使用 28b）
MANUAL_TAIL_RX        = b"\xFE"
MANUAL_FRAME_LEN_RX   = 1 + 1 + 24 + 1        # 27
BINARY_28B_FRAME_LEN  = 1 + 1 + 24 + 2        # 28 ← 本传感器实际格式
VENDOR_FRAME_LEN_RX   = 2 + 1 + 24 + 2        # 29

CMD_STOP            = 0x01
CMD_STREAM_1KHZ     = 0x02
CMD_SINGLE_SHOT     = 0x03
CMD_SET_BAUDRATE    = 0x04
CMD_QUERY_INFO      = 0x05
CMD_AUTO_START      = 0x06
CMD_TARE            = 0x30
CMD_DEBUG_EXIT      = 0x31
CMD_DEBUG_ENTER     = 0x32
CMD_MATRIX_FILTERED = 0x33
CMD_RAW_DATA        = 0x34

DATA_CMDS = frozenset({CMD_STREAM_1KHZ, CMD_SINGLE_SHOT, CMD_AUTO_START,
                       CMD_MATRIX_FILTERED, CMD_RAW_DATA})

CHANNELS_RE = re.compile(r"channels:\s*([^\r\n]+)")


# =============================================================================
# 帧构造
# =============================================================================

def build_manual(cmd: int, payload: bytes = b"") -> bytes:
    return MANUAL_HEAD + bytes([cmd]) + payload + MANUAL_TAIL_TX


def build_vendor(cmd: int, payload: bytes = b"") -> bytes:
    return VENDOR_HEAD + bytes([cmd]) + payload + VENDOR_TAIL


def send_command(ser: serial.Serial, cmd: int, payload: bytes = b"",
                 inter_delay_s: float = 0.03) -> None:
    if COMMAND_FORMAT == "manual":
        ser.write(build_manual(cmd, payload))
    elif COMMAND_FORMAT == "vendor":
        ser.write(build_vendor(cmd, payload))
    else:
        ser.write(build_manual(cmd, payload))
        time.sleep(inter_delay_s)
        ser.write(build_vendor(cmd, payload))


# =============================================================================
# 高层命令包装
# =============================================================================

def cmd_stop(ser):           send_command(ser, CMD_STOP);            time.sleep(0.1)
def cmd_stream_1khz(ser):    send_command(ser, CMD_STREAM_1KHZ);     time.sleep(0.05)
def cmd_single_shot(ser):    send_command(ser, CMD_SINGLE_SHOT)
def cmd_query_info(ser):     send_command(ser, CMD_QUERY_INFO);      time.sleep(0.05)
def cmd_auto_start(ser):     send_command(ser, CMD_AUTO_START);      time.sleep(0.05)
def cmd_tare(ser):           send_command(ser, CMD_TARE);            time.sleep(0.2)
def cmd_debug_exit(ser):     send_command(ser, CMD_DEBUG_EXIT);      time.sleep(0.1)
def cmd_debug_enter(ser):    send_command(ser, CMD_DEBUG_ENTER);     time.sleep(0.1)
def cmd_data_matrix(ser):    send_command(ser, CMD_MATRIX_FILTERED); time.sleep(0.05)
def cmd_data_raw(ser):       send_command(ser, CMD_RAW_DATA);        time.sleep(0.05)


def cmd_set_baudrate(ser, code: int) -> None:
    if code not in (0x01, 0x02, 0x03):
        raise ValueError(f"波特率编号必须是 0x01/0x02/0x03，收到 {code}")
    cmd_stop(ser)
    send_command(ser, CMD_SET_BAUDRATE, bytes([code]))
    time.sleep(0.2)


# =============================================================================
# 解析器
# =============================================================================

def _unpack_floats(data24: bytes) -> Optional[np.ndarray]:
    if len(data24) != 24:
        return None
    try:
        return np.array(struct.unpack("<6f", data24), dtype=float)
    except struct.error:
        return None


def parse_ascii_line(line_bytes: bytes) -> Optional[np.ndarray]:
    """支持 'channels: x,y,z,a,b,c' 或纯 'x,y,z,a,b,c'"""
    try:
        text = line_bytes.decode("ascii", errors="ignore").strip()
    except Exception:
        return None
    if not text:
        return None
    m = CHANNELS_RE.search(text)
    parts = m.group(1).split(",") if m else text.split(",")
    if len(parts) < 6:
        return None
    try:
        return np.array([float(p.strip()) for p in parts[:6]], dtype=float)
    except ValueError:
        return None


def read_binary_28b(ser: serial.Serial) -> Optional[Tuple[int, np.ndarray]]:
    """同步到 AA + 数据 cmd，读 28 字节帧，校验 0D 0A 帧尾。
    这是本传感器实测使用的格式。"""
    deadline = time.time() + SERIAL_TIMEOUT
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != 0xAA:
            continue
        # 读 cmd 字节，必须是数据响应码
        cmd_byte = ser.read(1)
        if len(cmd_byte) != 1:
            return None
        if cmd_byte[0] not in DATA_CMDS:
            continue  # 不是数据帧，可能是 AA 55 命令格式的开头，继续同步
        # 找到帧头，读剩下 26 字节 (24 data + 2 tail)
        rest = ser.read(BINARY_28B_FRAME_LEN - 2)
        if len(rest) != BINARY_28B_FRAME_LEN - 2:
            return None
        data = rest[0:24]
        tail = rest[24:26]
        if tail != b"\x0D\x0A":
            continue  # 帧尾不对，可能是误同步，继续找下一个 AA
        vals = _unpack_floats(data)
        return (cmd_byte[0], vals) if vals is not None else None
    return None


def read_binary_manual(ser: serial.Serial) -> Optional[Tuple[int, np.ndarray]]:
    deadline = time.time() + SERIAL_TIMEOUT
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        if b[0] != 0xAA:
            continue
        cmd_byte = ser.read(1)
        if len(cmd_byte) != 1 or cmd_byte[0] not in DATA_CMDS:
            continue
        rest = ser.read(MANUAL_FRAME_LEN_RX - 2)
        if len(rest) != MANUAL_FRAME_LEN_RX - 2:
            return None
        data = rest[0:24]
        tail = rest[24]
        if tail != MANUAL_TAIL_RX[0]:
            continue
        vals = _unpack_floats(data)
        return (cmd_byte[0], vals) if vals is not None else None
    return None


def read_binary_vendor(ser: serial.Serial) -> Optional[Tuple[int, np.ndarray]]:
    deadline = time.time() + SERIAL_TIMEOUT
    while time.time() < deadline:
        b1 = ser.read(1)
        if not b1 or b1 != b"\xAA":
            continue
        b2 = ser.read(1)
        if b2 != b"\x55":
            continue
        rest = ser.read(VENDOR_FRAME_LEN_RX - 2)
        if len(rest) != VENDOR_FRAME_LEN_RX - 2:
            return None
        cmd = rest[0]
        if cmd not in DATA_CMDS:
            continue
        data = rest[1:25]
        tail = rest[25:27]
        if tail != VENDOR_TAIL:
            continue
        vals = _unpack_floats(data)
        return (cmd, vals) if vals is not None else None
    return None


READER_FOR_FORMAT = {
    "binary_28b":     read_binary_28b,
    "binary_manual":  read_binary_manual,
    "binary_vendor":  read_binary_vendor,
}


# =============================================================================
# 格式检测（按出现概率从高到低排序）
# =============================================================================

def _periodic_pattern(positions: list, period: int, min_hits: int = 2) -> bool:
    """positions 中是否有至少 min_hits 个相邻间距为 period 的元素。"""
    if len(positions) < min_hits + 1:
        return False
    diffs = [positions[i+1] - positions[i] for i in range(len(positions)-1)]
    return Counter(diffs).get(period, 0) >= min_hits


def detect_format(ser: serial.Serial, duration_s: float = 0.8) -> Tuple[str, bytes]:
    deadline = time.time() + duration_s
    buf = bytearray()
    while time.time() < deadline and len(buf) < 2000:
        b = ser.read(128)
        if b:
            buf.extend(b)
    captured = bytes(buf)
    if not captured:
        return "none", captured

    # 1) ASCII (channels: 或纯 CSV)
    text = captured.decode("ascii", errors="ignore")
    if "channels:" in text:
        return "ascii", captured
    csv_lines = [l for l in text.split("\n")
                 if l.count(",") >= 5 and any(c.isdigit() for c in l)]
    if len(csv_lines) >= 3:
        return "ascii", captured

    # 2) 二进制 28b: AA + DATA_CMD + 24 + 0D 0A，间距 28 ← 本传感器实际格式
    pos_28b = [
        i for i in range(len(captured) - BINARY_28B_FRAME_LEN + 1)
        if captured[i] == 0xAA and captured[i+1] in DATA_CMDS
        and captured[i + BINARY_28B_FRAME_LEN - 2 : i + BINARY_28B_FRAME_LEN] == b"\x0D\x0A"
    ]
    if _periodic_pattern(pos_28b, BINARY_28B_FRAME_LEN):
        return "binary_28b", captured

    # 3) 二进制厂商: AA 55 + cmd + 24 + 0D 0A, 间距 29
    pos_vendor = [
        i for i in range(len(captured) - VENDOR_FRAME_LEN_RX + 1)
        if captured[i] == 0xAA and captured[i+1] == 0x55 and captured[i+2] in DATA_CMDS
        and captured[i + VENDOR_FRAME_LEN_RX - 2 : i + VENDOR_FRAME_LEN_RX] == b"\x0D\x0A"
    ]
    if _periodic_pattern(pos_vendor, VENDOR_FRAME_LEN_RX):
        return "binary_vendor", captured

    # 4) 二进制手册: AA + cmd + 24 + FE, 间距 27
    pos_manual = [
        i for i in range(len(captured) - MANUAL_FRAME_LEN_RX + 1)
        if captured[i] == 0xAA and captured[i+1] in DATA_CMDS
        and captured[i + MANUAL_FRAME_LEN_RX - 1] == MANUAL_TAIL_RX[0]
    ]
    if _periodic_pattern(pos_manual, MANUAL_FRAME_LEN_RX):
        return "binary_manual", captured

    return "unknown", captured


# =============================================================================
# Hex/ASCII dump
# =============================================================================

def dump_bytes(buf: bytes, max_bytes: int = 256) -> None:
    rospy.logwarn("─" * 75)
    rospy.logwarn(f"原始字节 dump (前 {min(max_bytes, len(buf))} / {len(buf)} 字节):")
    chunk = buf[:max_bytes]
    for offset in range(0, len(chunk), 16):
        row = chunk[offset:offset+16]
        hex_part = " ".join(f"{b:02X}" for b in row).ljust(48)
        ascii_part = "".join(chr(b) if 32 <= b < 127 else "." for b in row)
        rospy.logwarn(f"  {offset:04X}: {hex_part}  |{ascii_part}|")
    rospy.logwarn("─" * 75)
    n_aa = chunk.count(0xAA)
    n_aa55 = sum(1 for i in range(len(chunk)-1) if chunk[i] == 0xAA and chunk[i+1] == 0x55)
    n_fe = chunk.count(0xFE)
    n_0a = chunk.count(0x0A)
    n_0d0a = sum(1 for i in range(len(chunk)-1) if chunk[i] == 0x0D and chunk[i+1] == 0x0A)
    n_printable = sum(1 for b in chunk if 32 <= b < 127)
    rospy.logwarn(f"字节统计: AA={n_aa}  AA55={n_aa55}  FE={n_fe}  "
                  f"\\n={n_0a}  \\r\\n={n_0d0a}  "
                  f"ASCII={n_printable}/{len(chunk)} ({100*n_printable//max(1,len(chunk))}%)")
    rospy.logwarn("─" * 75)


# =============================================================================
# Host 侧滤波
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


def convert_units(vals_kg: np.ndarray) -> np.ndarray:
    return vals_kg * GRAVITY if OUTPUT_UNITS == "N" else vals_kg


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


def open_sensor() -> serial.Serial:
    return serial.Serial(
        port=SERIAL_PORT, baudrate=BAUD_RATE,
        bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE, timeout=SERIAL_TIMEOUT,
    )


# =============================================================================
# 启动序列
# =============================================================================

def startup_sequence(ser: serial.Serial) -> None:
    if STOP_ON_START:
        rospy.loginfo("发送 0x01 停止任何残留数据流")
        cmd_stop(ser)
        ser.reset_input_buffer()

    if EXIT_DEBUG_ON_START:
        rospy.loginfo("发送 0x31 退出 debug 模式（双格式兼容）")
        cmd_debug_exit(ser)
        time.sleep(0.2)
        ser.reset_input_buffer()

    if TARE_ON_START:
        rospy.loginfo("发送 0x30 清零")
        cmd_tare(ser)

    if WARMUP_S > 0:
        rospy.loginfo(f"预热 {WARMUP_S} 秒")
        time.sleep(WARMUP_S)


def start_data_stream(ser: serial.Serial) -> None:
    if DATA_SOURCE_CMD in (CMD_MATRIX_FILTERED, CMD_RAW_DATA):
        ds_name = "0x33 矩阵滤波" if DATA_SOURCE_CMD == CMD_MATRIX_FILTERED else "0x34 原始"
        rospy.loginfo(f"切换数据源 → {ds_name}")
        send_command(ser, DATA_SOURCE_CMD)
        time.sleep(0.1)
        ser.reset_input_buffer()

    if USE_STREAMING:
        rospy.loginfo("启动 1kHz 流推送 (0x02)")
        cmd_stream_1khz(ser)


# =============================================================================
# 发布循环
# =============================================================================

def loop_ascii(ser: serial.Serial, pub: rospy.Publisher) -> None:
    rospy.loginfo("使用 ASCII 解析器")
    filt = make_filter()
    frame_idx = 0
    err_count = 0
    t_log = time.time()

    while not rospy.is_shutdown():
        line = ser.readline()
        if not line:
            err_count += 1
            if err_count % TIMEOUT_REPORT == 1:
                rospy.logwarn(f"读取超时（累计 {err_count}）")
            continue

        if PRINT_RAW:
            rospy.loginfo(f"[RAW] {line!r}")

        vals_kg = parse_ascii_line(line)
        if vals_kg is None:
            err_count += 1
            if err_count % TIMEOUT_REPORT == 1:
                rospy.logwarn(f"行解析失败（累计 {err_count}）: {line[:60]!r}")
            continue

        vals = convert_units(vals_kg)
        vals_pub = filt(vals) if filt is not None else vals
        pub.publish(make_wrench_msg(vals_pub, rospy.Time.now()))

        frame_idx += 1
        if LOG_EVERY_N > 0 and frame_idx % LOG_EVERY_N == 0:
            now = time.time()
            actual_hz = LOG_EVERY_N / (now - t_log) if now > t_log else 0.0
            t_log = now
            rospy.loginfo(
                f"[ascii {frame_idx:7d}] @{actual_hz:6.1f}Hz "
                f"Fx={vals_pub[0]:+7.3f} Fy={vals_pub[1]:+7.3f} Fz={vals_pub[2]:+7.3f} "
                f"Mx={vals_pub[3]:+7.4f} My={vals_pub[4]:+7.4f} Mz={vals_pub[5]:+7.4f}"
            )


def loop_binary(ser: serial.Serial, pub: rospy.Publisher, fmt: str) -> None:
    reader = READER_FOR_FORMAT[fmt]
    rospy.loginfo(f"使用二进制解析器 ({fmt})")

    filt = make_filter()
    rate = rospy.Rate(POLL_HZ) if not USE_STREAMING else None
    frame_idx = 0
    err_count = 0
    t_log = time.time()

    while not rospy.is_shutdown():
        if not USE_STREAMING:
            ser.reset_input_buffer()
            cmd_single_shot(ser)

        result = reader(ser)
        if result is None:
            err_count += 1
            if err_count % TIMEOUT_REPORT == 1:
                rospy.logwarn(f"二进制帧读取失败（累计 {err_count}）")
            if rate is not None:
                rate.sleep()
            continue

        cmd, vals_kg = result
        if PRINT_RAW:
            rospy.loginfo(f"[RAW] cmd=0x{cmd:02X} vals={vals_kg}")

        vals = convert_units(vals_kg)
        vals_pub = filt(vals) if filt is not None else vals
        pub.publish(make_wrench_msg(vals_pub, rospy.Time.now()))

        frame_idx += 1
        if LOG_EVERY_N > 0 and frame_idx % LOG_EVERY_N == 0:
            now = time.time()
            actual_hz = LOG_EVERY_N / (now - t_log) if now > t_log else 0.0
            t_log = now
            label = fmt.replace("binary_", "")
            rospy.loginfo(
                f"[{label:6s} {frame_idx:7d}] @{actual_hz:6.1f}Hz "
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

    rospy.loginfo("=" * 60)
    rospy.loginfo("MIOS 力传感器 ROS 节点 v5.3")
    rospy.loginfo(f"  串口:       {SERIAL_PORT} @ {BAUD_RATE} bps")
    rospy.loginfo(f"  命令格式:   {COMMAND_FORMAT}")
    rospy.loginfo(f"  退出debug:  {EXIT_DEBUG_ON_START}")
    rospy.loginfo(f"  数据源:     0x{DATA_SOURCE_CMD:02X}")
    rospy.loginfo(f"  流模式:     {USE_STREAMING}")
    rospy.loginfo(f"  ASCII 回退: {ALLOW_ASCII_FALLBACK}")
    rospy.loginfo(f"  Host 滤波:  {filter_label()}")
    rospy.loginfo(f"  输出单位:   {OUTPUT_UNITS}")
    rospy.loginfo(f"  ROS 话题:   {ROS_TOPIC}")
    rospy.loginfo(f"  Frame ID:   {FRAME_ID}")
    rospy.loginfo("=" * 60)

    ser = open_sensor()
    pub = rospy.Publisher(ROS_TOPIC, WrenchStamped, queue_size=PUB_QUEUE_SIZE)

    try:
        startup_sequence(ser)
        start_data_stream(ser)

        time.sleep(0.2)
        rospy.loginfo("检测输出格式...")
        fmt, sample = detect_format(ser, duration_s=0.8)
        rospy.loginfo(f"检测结果: {fmt} (捕获 {len(sample)} 字节)")

        if fmt == "ascii":
            loop_ascii(ser, pub)

        elif fmt in READER_FOR_FORMAT:
            ser.reset_input_buffer()
            if USE_STREAMING:
                cmd_stream_1khz(ser)
            loop_binary(ser, pub, fmt)

        elif fmt == "unknown":
            if DUMP_ON_UNKNOWN:
                dump_bytes(sample, DUMP_BYTES)
            if ALLOW_ASCII_FALLBACK:
                rospy.logwarn("ALLOW_ASCII_FALLBACK=True，尝试 ASCII 解析")
                loop_ascii(ser, pub)
            else:
                rospy.logerr("无法确定输出格式，退出。")

        else:  # "none"
            rospy.logerr(
                "未收到任何数据。检查: (1) 传感器供电, (2) RS485 A/B 接线, "
                "(3) 公共 GND, (4) 波特率")

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"运行异常: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            cmd_stop(ser)
        except Exception:
            pass
        ser.close()
        rospy.loginfo("串口已关闭")


if __name__ == "__main__":
    main()