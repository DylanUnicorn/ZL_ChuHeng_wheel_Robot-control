#!/usr/bin/env python3
"""Serial protocol tester for hardware_driver.

This script is intended for host-side testing on Windows before running the
Jetson ROS node. It can:

1. Verify CRC and frame packing logic offline.
2. Open a serial port such as COM15.
3. Monitor incoming MCU frames and verify CRC on every frame.
4. Send chassis control commands to the MCU.
5. Run a simple connectivity smoke test.

Examples:

  python serial_protocol_tester.py self-check
  python serial_protocol_tester.py list-ports
  python serial_protocol_tester.py connectivity --port COM15 --duration 3
  python serial_protocol_tester.py monitor --port COM15 --duration 10 --show-hex
  python serial_protocol_tester.py interactive --port COM15
  python serial_protocol_tester.py send-control --port COM15 --vx 0.2 --wz 0.1 \
      --duration 2 --repeat-hz 10 --watch 3
"""

from __future__ import annotations

import argparse
import struct
import sys
import time
from collections import Counter
from dataclasses import dataclass
from typing import Optional

try:
  import serial
  from serial.tools import list_ports
except ImportError:
  serial = None
  list_ports = None


FRAME_HEAD = 0x5A
FRAME_TAIL = 0xA5
MIN_FRAME_SIZE = 6

CHASSIS_CONTROL = 0x10
CHASSIS_STATUS = 0x11
BATTERY_INFO = 0x12
LEG_MOTOR_STATUS = 0x13
HIP_MOTOR_STATUS = 0x14
CHASSIS_IMU = 0x15

MESSAGE_NAMES = {
    CHASSIS_CONTROL: "CHASSIS_CONTROL",
    CHASSIS_STATUS: "CHASSIS_STATUS",
    BATTERY_INFO: "BATTERY_INFO",
    LEG_MOTOR_STATUS: "LEG_MOTOR_STATUS",
    HIP_MOTOR_STATUS: "HIP_MOTOR_STATUS",
    CHASSIS_IMU: "CHASSIS_IMU",
    0x20: "ARM_CONTROL",
    0x21: "ARM_STATUS",
    0x22: "HEAD_CONTROL",
    0x23: "HEAD_STATUS",
    0x30: "MICROPHONE_DATA",
    0x40: "FULLBODY_CONTROL",
    0x41: "FULLBODY_STATUS",
    0x42: "CONTROL_MODE_SET",
    0x43: "EFFECTOR_FEEDBACK",
    0x44: "HEARTBEAT",
    0x45: "WATCHDOG_STATUS",
    0x46: "GO_HOME",
    0x47: "HOME_STATUS",
    0x50: "SINGLE_MOTOR_TEST",
    0x52: "READ_RAW_POS",
}


CRC_TABLE = (
    0x0000, 0x1189, 0x2312, 0x329B, 0x4624, 0x57AD, 0x6536, 0x74BF,
    0x8C48, 0x9DC1, 0xAF5A, 0xBED3, 0xCA6C, 0xDBE5, 0xE97E, 0xF8F7,
    0x1081, 0x0108, 0x3393, 0x221A, 0x56A5, 0x472C, 0x75B7, 0x643E,
    0x9CC9, 0x8D40, 0xBFDB, 0xAE52, 0xDAED, 0xCB64, 0xF9FF, 0xE876,
    0x2102, 0x308B, 0x0210, 0x1399, 0x6726, 0x76AF, 0x4434, 0x55BD,
    0xAD4A, 0xBCC3, 0x8E58, 0x9FD1, 0xEB6E, 0xFAE7, 0xC87C, 0xD9F5,
    0x3183, 0x200A, 0x1291, 0x0318, 0x77A7, 0x662E, 0x54B5, 0x453C,
    0xBDCB, 0xAC42, 0x9ED9, 0x8F50, 0xFBEF, 0xEA66, 0xD8FD, 0xC974,
    0x4204, 0x538D, 0x6116, 0x709F, 0x0420, 0x15A9, 0x2732, 0x36BB,
    0xCE4C, 0xDFC5, 0xED5E, 0xFCD7, 0x8868, 0x99E1, 0xAB7A, 0xBAF3,
    0x5285, 0x430C, 0x7197, 0x601E, 0x14A1, 0x0528, 0x37B3, 0x263A,
    0xDECD, 0xCF44, 0xFDDF, 0xEC56, 0x98E9, 0x8960, 0xBBFB, 0xAA72,
    0x6306, 0x728F, 0x4014, 0x519D, 0x2522, 0x34AB, 0x0630, 0x17B9,
    0xEF4E, 0xFEC7, 0xCC5C, 0xDDD5, 0xA96A, 0xB8E3, 0x8A78, 0x9BF1,
    0x7387, 0x620E, 0x5095, 0x411C, 0x35A3, 0x242A, 0x16B1, 0x0738,
    0xFFCF, 0xEE46, 0xDCDD, 0xCD54, 0xB9EB, 0xA862, 0x9AF9, 0x8B70,
    0x8408, 0x9581, 0xA71A, 0xB693, 0xC22C, 0xD3A5, 0xE13E, 0xF0B7,
    0x0840, 0x19C9, 0x2B52, 0x3ADB, 0x4E64, 0x5FED, 0x6D76, 0x7CFF,
    0x9489, 0x8500, 0xB79B, 0xA612, 0xD2AD, 0xC324, 0xF1BF, 0xE036,
    0x18C1, 0x0948, 0x3BD3, 0x2A5A, 0x5EE5, 0x4F6C, 0x7DF7, 0x6C7E,
    0xA50A, 0xB483, 0x8618, 0x9791, 0xE32E, 0xF2A7, 0xC03C, 0xD1B5,
    0x2942, 0x38CB, 0x0A50, 0x1BD9, 0x6F66, 0x7EEF, 0x4C74, 0x5DFD,
    0xB58B, 0xA402, 0x9699, 0x8710, 0xF3AF, 0xE226, 0xD0BD, 0xC134,
    0x39C3, 0x284A, 0x1AD1, 0x0B58, 0x7FE7, 0x6E6E, 0x5CF5, 0x4D7C,
    0xC60C, 0xD785, 0xE51E, 0xF497, 0x8028, 0x91A1, 0xA33A, 0xB2B3,
    0x4A44, 0x5BCD, 0x6956, 0x78DF, 0x0C60, 0x1DE9, 0x2F72, 0x3EFB,
    0xD68D, 0xC704, 0xF59F, 0xE416, 0x90A9, 0x8120, 0xB3BB, 0xA232,
    0x5AC5, 0x4B4C, 0x79D7, 0x685E, 0x1CE1, 0x0D68, 0x3FF3, 0x2E7A,
    0xE70E, 0xF687, 0xC41C, 0xD595, 0xA12A, 0xB0A3, 0x8238, 0x93B1,
    0x6B46, 0x7ACF, 0x4854, 0x59DD, 0x2D62, 0x3CEB, 0x0E70, 0x1FF9,
    0xF78F, 0xE606, 0xD49D, 0xC514, 0xB1AB, 0xA022, 0x92B9, 0x8330,
    0x7BC7, 0x6A4E, 0x58D5, 0x495C, 0x3DE3, 0x2C6A, 0x1EF1, 0x0F78,
)


@dataclass(frozen=True)
class Frame:
  type_id: int
  payload: bytes
  crc: Optional[int] = None

  def encode(self) -> bytes:
    if len(self.payload) > 255:
      raise ValueError("payload length must fit in one byte")
    body = bytes((FRAME_HEAD, self.type_id & 0xFF, len(self.payload)))
    body += self.payload
    crc = crc16_table(body)
    return body + struct.pack("<HB", crc, FRAME_TAIL)


@dataclass
class ParserStats:
  discarded_bytes: int = 0
  crc_failures: int = 0
  tail_failures: int = 0


class FrameParser:
  def __init__(self) -> None:
    self._buffer = bytearray()
    self.stats = ParserStats()

  def feed(self, data: bytes) -> list[Frame]:
    frames: list[Frame] = []
    if not data:
      return frames

    self._buffer.extend(data)
    while len(self._buffer) >= MIN_FRAME_SIZE:
      if self._buffer[0] != FRAME_HEAD:
        del self._buffer[0]
        self.stats.discarded_bytes += 1
        continue

      payload_length = self._buffer[2]
      frame_size = 3 + payload_length + 3
      if len(self._buffer) < frame_size:
        break

      raw_frame = bytes(self._buffer[:frame_size])
      if raw_frame[-1] != FRAME_TAIL:
        del self._buffer[0]
        self.stats.tail_failures += 1
        continue

      calculated_crc = crc16_table(raw_frame[:-3])
      received_crc = struct.unpack("<H", raw_frame[-3:-1])[0]
      if calculated_crc != received_crc:
        del self._buffer[0]
        self.stats.crc_failures += 1
        continue

      frames.append(
          Frame(
              type_id=raw_frame[1],
              payload=raw_frame[3:-3],
              crc=received_crc,
          ))
      del self._buffer[:frame_size]

    return frames


def crc16_table(data: bytes) -> int:
  crc = 0xFFFF
  for byte in data:
    crc = (crc >> 8) ^ CRC_TABLE[(crc ^ byte) & 0xFF]
  return crc & 0xFFFF


def crc16_bitwise(data: bytes) -> int:
  crc = 0xFFFF
  for byte in data:
    crc ^= byte
    for _ in range(8):
      if crc & 0x0001:
        crc = (crc >> 1) ^ 0x8408
      else:
        crc >>= 1
  return crc & 0xFFFF


def timestamp() -> str:
  return time.strftime("%H:%M:%S")


def log(message: str) -> None:
  print(f"[{timestamp()}] {message}")


def prompt(message: str) -> str:
  return input(f"[{timestamp()}] {message}").strip()


def format_hex(data: bytes) -> str:
  return " ".join(f"{byte:02X}" for byte in data)


def message_name(type_id: int) -> str:
  return MESSAGE_NAMES.get(type_id, f"UNKNOWN_0x{type_id:02X}")


def build_chassis_control(
    control_type: int,
    vx: float,
    vy: float,
    wz: float,
) -> Frame:
  payload = struct.pack("<Bfff", control_type, vx, vy, wz)
  return Frame(CHASSIS_CONTROL, payload)


def decode_frame(frame: Frame) -> str:
  payload = frame.payload
  type_id = frame.type_id

  if type_id == CHASSIS_CONTROL and len(payload) == 13:
    control_type, vx, vy, wz = struct.unpack("<Bfff", payload)
    return (
        f"control_type={control_type}, vx={vx:.3f} m/s, "
        f"vy={vy:.3f} m/s, wz={wz:.3f} rad/s"
    )

  if type_id == CHASSIS_STATUS and len(payload) == 24:
    vx, vy, wz, pos_x, pos_y, theta = struct.unpack("<ffffff", payload)
    return (
        f"vx={vx:.3f} m/s, vy={vy:.3f} m/s, wz={wz:.3f} rad/s, "
        f"x={pos_x:.3f} m, y={pos_y:.3f} m, theta={theta:.3f} rad"
    )

  if type_id == BATTERY_INFO and len(payload) >= 13:
    voltage, current, percentage, status = struct.unpack("<fffB", payload[:13])
    suffix = ""
    if len(payload) > 13:
      suffix = f", extra={len(payload) - 13} byte(s)"
    return (
        f"voltage={voltage:.3f} V, current={current:.3f} A, "
        f"percentage={percentage:.1f} %, status={status}{suffix}"
    )

  if type_id == CHASSIS_IMU and len(payload) == 36:
    values = struct.unpack("<fffffffff", payload)
    return (
        "roll={:.3f}, pitch={:.3f}, yaw={:.3f}, gx={:.3f}, gy={:.3f}, "
        "gz={:.3f}, ax={:.3f}, ay={:.3f}, az={:.3f}"
    ).format(*values)

  return f"payload={len(payload)} byte(s)"


def ensure_pyserial() -> None:
  if serial is None:
    raise SystemExit(
        "pyserial is not installed. Run: pip install pyserial"
    )


def open_serial_port(port: str, baud: int, timeout: float):
  ensure_pyserial()
  try:
    return serial.Serial(port=port, baudrate=baud, timeout=timeout)
  except Exception as exc:
    available = []
    if list_ports is not None:
      available = [p.device for p in list_ports.comports()]
    message = f"failed to open {port}: {exc}"
    if available:
      message += f"\navailable ports: {', '.join(available)}"
    raise SystemExit(message) from exc


def summarize_counts(counts: Counter, prefix: str) -> None:
  if counts:
    summary = ", ".join(
        f"{message_name(type_id)}={count}"
        for type_id, count in sorted(counts.items())
    )
    log(f"{prefix}: {summary}")
  else:
    log(f"{prefix}: no valid frame received")


def monitor_stream(
    ser,
    duration: float,
    show_hex: bool,
    stop_on_first_frame: bool = False,
) -> Counter:
  parser = FrameParser()
  counts: Counter = Counter()
  start_time = time.monotonic()

  while duration <= 0 or (time.monotonic() - start_time) < duration:
    chunk = ser.read(max(1, ser.in_waiting or 1))
    if not chunk:
      continue

    for frame in parser.feed(chunk):
      counts[frame.type_id] += 1
      text = (
          f"RX {message_name(frame.type_id)}(0x{frame.type_id:02X}) "
          f"len={len(frame.payload)} crc=0x{frame.crc:04X}: "
          f"{decode_frame(frame)}"
      )
      if show_hex:
        text += f" | {format_hex(frame.encode())}"
      log(text)
      if stop_on_first_frame:
        return counts

  if parser.stats.crc_failures or parser.stats.tail_failures:
    log(
        "parser warnings: "
        f"discarded={parser.stats.discarded_bytes}, "
        f"tail_failures={parser.stats.tail_failures}, "
        f"crc_failures={parser.stats.crc_failures}"
    )
  return counts


def send_frame(
    ser,
    frame: Frame,
    show_hex: bool,
) -> None:
  raw = frame.encode()
  ser.write(raw)
  ser.flush()
  text = (
      f"TX {message_name(frame.type_id)}(0x{frame.type_id:02X}) "
      f"len={len(frame.payload)}: {decode_frame(frame)}"
  )
  if show_hex:
    text += f" | {format_hex(raw)}"
  log(text)


def send_chassis_command(
    ser,
    control_type: int,
    vx: float,
    vy: float,
    wz: float,
    duration: float,
    repeat_hz: float,
    show_hex: bool,
    auto_stop: bool = False,
) -> None:
  frame = build_chassis_control(
      control_type=control_type,
      vx=vx,
      vy=vy,
      wz=wz,
  )

  if duration > 0 and repeat_hz > 0:
    deadline = time.monotonic() + duration
    period = 1.0 / repeat_hz
    while time.monotonic() < deadline:
      send_frame(ser, frame, show_hex)
      time.sleep(period)
  else:
    send_frame(ser, frame, show_hex)

  if auto_stop and (vx != 0.0 or vy != 0.0 or wz != 0.0):
    stop_frame = build_chassis_control(control_type=control_type, vx=0.0, vy=0.0,
                                       wz=0.0)
    send_frame(ser, stop_frame, show_hex)


def print_interactive_menu(args: argparse.Namespace) -> None:
  print()
  print("================ 串口交互菜单 ================")
  print(f"串口: {args.port}  波特率: {args.baud}  show_hex: {args.show_hex}")
  print(f"脉冲时长: {args.pulse_seconds:.2f}s  发送频率: {args.repeat_hz:.1f} Hz  回看: {args.watch:.1f}s")
  print("1. 停止底盘")
  print(f"2. 前进  vx=+{args.move_speed:.3f}")
  print(f"3. 后退  vx=-{args.move_speed:.3f}")
  print(f"4. 左移  vy=+{args.strafe_speed:.3f}")
  print(f"5. 右移  vy=-{args.strafe_speed:.3f}")
  print(f"6. 左转  wz=+{args.turn_speed:.3f}")
  print(f"7. 右转  wz=-{args.turn_speed:.3f}")
  print("8. 连通性测试")
  print("9. 监听回包")
  print("c. 自定义 vx vy wz")
  print("h. 重新打印菜单")
  print("q. 退出")
  print("============================================")
  print()


def run_connectivity_session(ser, duration: float, repeat_hz: float,
                             show_hex: bool) -> Counter:
  frame = build_chassis_control(control_type=1, vx=0.0, vy=0.0, wz=0.0)
  deadline = time.monotonic() + duration
  period = 1.0 / repeat_hz
  next_send = time.monotonic()
  counts: Counter = Counter()
  parser = FrameParser()

  while time.monotonic() < deadline:
    now = time.monotonic()
    if now >= next_send:
      send_frame(ser, frame, show_hex)
      next_send = now + period

    chunk = ser.read(max(1, ser.in_waiting or 1))
    if not chunk:
      continue

    for parsed in parser.feed(chunk):
      counts[parsed.type_id] += 1
      text = (
          f"RX {message_name(parsed.type_id)}(0x{parsed.type_id:02X}) "
          f"len={len(parsed.payload)} crc=0x{parsed.crc:04X}: "
          f"{decode_frame(parsed)}"
      )
      if show_hex:
        text += f" | {format_hex(parsed.encode())}"
      log(text)

  return counts


def watch_feedback(ser, watch: float, show_hex: bool, prefix: str) -> None:
  if watch <= 0:
    return
  log(f"watching MCU feedback for {watch:.1f}s")
  counts = monitor_stream(
      ser,
      duration=watch,
      show_hex=show_hex,
      stop_on_first_frame=False,
  )
  summarize_counts(counts, prefix)


def run_interactive(args: argparse.Namespace) -> int:
  with open_serial_port(args.port, args.baud, args.timeout) as ser:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    log(f"opened {args.port} @ {args.baud}")
    print_interactive_menu(args)

    while True:
      try:
        choice = prompt("请选择指令 > ").lower()
      except EOFError:
        print()
        log("input stream closed, leaving interactive mode")
        return 0

      if not choice:
        continue

      if choice == "q":
        log("interactive mode closed")
        return 0

      if choice == "h":
        print_interactive_menu(args)
        continue

      if choice == "1":
        send_chassis_command(
            ser,
            control_type=1,
            vx=0.0,
            vy=0.0,
            wz=0.0,
            duration=0.0,
            repeat_hz=args.repeat_hz,
            show_hex=args.show_hex,
            auto_stop=False,
        )
        watch_feedback(ser, args.watch, args.show_hex, "feedback summary")
        continue

      if choice in {"2", "3", "4", "5", "6", "7"}:
        vx = 0.0
        vy = 0.0
        wz = 0.0
        if choice == "2":
          vx = args.move_speed
        elif choice == "3":
          vx = -args.move_speed
        elif choice == "4":
          vy = args.strafe_speed
        elif choice == "5":
          vy = -args.strafe_speed
        elif choice == "6":
          wz = args.turn_speed
        elif choice == "7":
          wz = -args.turn_speed

        send_chassis_command(
            ser,
            control_type=1,
            vx=vx,
            vy=vy,
            wz=wz,
            duration=args.pulse_seconds,
            repeat_hz=args.repeat_hz,
            show_hex=args.show_hex,
            auto_stop=True,
        )
        watch_feedback(ser, args.watch, args.show_hex, "feedback summary")
        continue

      if choice == "8":
        counts = run_connectivity_session(
            ser,
            duration=args.connectivity_duration,
            repeat_hz=args.repeat_hz,
            show_hex=args.show_hex,
        )
        summarize_counts(counts, "connectivity summary")
        continue

      if choice == "9":
        counts = monitor_stream(
            ser,
            duration=args.monitor_duration,
            show_hex=args.show_hex,
            stop_on_first_frame=False,
        )
        summarize_counts(counts, "monitor summary")
        continue

      if choice == "c":
        raw_values = prompt(
            "输入 vx vy wz，例如 0.2 0 0.1 > "
        )
        parts = raw_values.replace(",", " ").split()
        if len(parts) != 3:
          log("invalid input, expected exactly 3 numbers")
          continue
        try:
          vx, vy, wz = (float(parts[0]), float(parts[1]), float(parts[2]))
        except ValueError:
          log("invalid input, numbers required")
          continue

        send_chassis_command(
            ser,
            control_type=1,
            vx=vx,
            vy=vy,
            wz=wz,
            duration=args.pulse_seconds,
            repeat_hz=args.repeat_hz,
            show_hex=args.show_hex,
            auto_stop=True,
        )
        watch_feedback(ser, args.watch, args.show_hex, "feedback summary")
        continue

      log("unknown command, press h to show the menu again")



def run_self_check(_: argparse.Namespace) -> int:
  sample_payload = struct.pack("<Bfff", 1, 0.5, 0.0, 0.3)
  sample_body = bytes((FRAME_HEAD, CHASSIS_CONTROL, len(sample_payload)))
  sample_body += sample_payload

  crc_table_value = crc16_table(sample_body)
  crc_bitwise_value = crc16_bitwise(sample_body)
  if crc_table_value != crc_bitwise_value:
    log(
        "self-check failed: table CRC 0x"
        f"{crc_table_value:04X} != bitwise CRC 0x{crc_bitwise_value:04X}"
    )
    return 1

  frame = build_chassis_control(control_type=1, vx=0.5, vy=0.0, wz=0.3)
  encoded = frame.encode()
  parser = FrameParser()
  parsed = parser.feed(encoded)
  if len(parsed) != 1:
    log("self-check failed: encoded frame could not be parsed back")
    return 1

  if parsed[0].payload != frame.payload or parsed[0].type_id != frame.type_id:
    log("self-check failed: round-trip payload mismatch")
    return 1

  log("self-check passed")
  log(f"sample frame: {format_hex(encoded)}")
  log(f"sample crc: 0x{crc_table_value:04X}")
  return 0


def run_list_ports(_: argparse.Namespace) -> int:
  ensure_pyserial()
  ports = list(list_ports.comports())
  if not ports:
    log("no serial ports found")
    return 0

  for port in ports:
    description = port.description or ""
    hwid = port.hwid or ""
    print(f"{port.device}: {description} {hwid}".strip())
  return 0


def run_connectivity(args: argparse.Namespace) -> int:
  with open_serial_port(args.port, args.baud, args.timeout) as ser:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    log(f"opened {args.port} @ {args.baud}")

    counts = run_connectivity_session(
        ser,
        duration=args.duration,
        repeat_hz=args.repeat_hz,
        show_hex=args.show_hex,
    )

    if counts:
      summarize_counts(counts, "connectivity check passed")
      return 0

    log(
        "connectivity check did not receive any valid frame. "
        "Check COM port, baud rate, and whether the MCU is already running."
    )
    return 1


def run_monitor(args: argparse.Namespace) -> int:
  with open_serial_port(args.port, args.baud, args.timeout) as ser:
    log(f"opened {args.port} @ {args.baud}")
    counts = monitor_stream(
        ser,
        duration=args.duration,
        show_hex=args.show_hex,
        stop_on_first_frame=False,
    )

  summarize_counts(counts, "monitor summary")
  return 0


def run_send_control(args: argparse.Namespace) -> int:
  with open_serial_port(args.port, args.baud, args.timeout) as ser:
    log(f"opened {args.port} @ {args.baud}")
    send_chassis_command(
        ser,
        control_type=args.control_type,
        vx=args.vx,
        vy=args.vy,
        wz=args.wz,
        duration=args.duration,
        repeat_hz=args.repeat_hz,
        show_hex=args.show_hex,
        auto_stop=False,
    )

    watch_feedback(ser, args.watch, args.show_hex, "feedback summary")
  return 0


def parse_payload_hex(payload_hex: str) -> bytes:
  cleaned = payload_hex.replace(" ", "").replace("_", "")
  if cleaned.startswith("0x"):
    cleaned = cleaned[2:]
  if len(cleaned) % 2 != 0:
    raise ValueError("payload hex must contain an even number of digits")
  return bytes.fromhex(cleaned)


def run_send_raw(args: argparse.Namespace) -> int:
  try:
    payload = parse_payload_hex(args.payload_hex)
  except ValueError as exc:
    raise SystemExit(str(exc)) from exc

  frame = Frame(args.type_id, payload)
  with open_serial_port(args.port, args.baud, args.timeout) as ser:
    log(f"opened {args.port} @ {args.baud}")
    send_frame(ser, frame, args.show_hex)
    if args.watch > 0:
      log(f"watching MCU feedback for {args.watch:.1f}s")
      monitor_stream(
          ser,
          duration=args.watch,
          show_hex=args.show_hex,
          stop_on_first_frame=False,
      )
  return 0


def add_serial_arguments(parser: argparse.ArgumentParser) -> None:
  parser.add_argument("--port", default="COM15", help="serial port, default COM15")
  parser.add_argument("--baud", type=int, default=115200, help="baud rate")
  parser.add_argument(
      "--timeout",
      type=float,
      default=0.1,
      help="serial read timeout in seconds",
  )
  parser.add_argument(
      "--show-hex",
      action="store_true",
      help="print full frame hex for TX and RX",
  )


def build_arg_parser() -> argparse.ArgumentParser:
  parser = argparse.ArgumentParser(
      description="Windows host-side serial protocol tester for hardware_driver"
  )
  subparsers = parser.add_subparsers(dest="command", required=True)

  subparsers.add_parser(
      "self-check",
      help="verify CRC and frame packing without any hardware",
  )
  subparsers.add_parser("list-ports", help="list available serial ports")

  connectivity = subparsers.add_parser(
      "connectivity",
      help="send zero-velocity chassis control and wait for valid MCU frames",
  )
  add_serial_arguments(connectivity)
  connectivity.add_argument(
      "--duration",
      type=float,
      default=3.0,
      help="test duration in seconds",
  )
  connectivity.add_argument(
      "--repeat-hz",
      type=float,
      default=5.0,
      help="how often to resend the idle control frame during the test",
  )

  monitor = subparsers.add_parser(
      "monitor",
      help="monitor incoming frames and verify CRC",
  )
  add_serial_arguments(monitor)
  monitor.add_argument(
      "--duration",
      type=float,
      default=10.0,
      help="monitor duration in seconds, <=0 means keep running",
  )

  interactive = subparsers.add_parser(
      "interactive",
      help="open the serial port once and drive the tester from a keyboard menu",
  )
  add_serial_arguments(interactive)
  interactive.add_argument(
      "--move-speed",
      type=float,
      default=0.20,
      help="forward/backward pulse speed in m/s",
  )
  interactive.add_argument(
      "--strafe-speed",
      type=float,
      default=0.20,
      help="left/right pulse speed in m/s",
  )
  interactive.add_argument(
      "--turn-speed",
      type=float,
      default=0.30,
      help="turn pulse speed in rad/s",
  )
  interactive.add_argument(
      "--pulse-seconds",
      type=float,
      default=0.50,
      help="how long each motion pulse is sent before auto-stop",
  )
  interactive.add_argument(
      "--repeat-hz",
      type=float,
      default=10.0,
      help="send rate for motion pulses and connectivity checks",
  )
  interactive.add_argument(
      "--watch",
      type=float,
      default=1.5,
      help="feedback watch window after each motion command",
  )
  interactive.add_argument(
      "--monitor-duration",
      type=float,
      default=5.0,
      help="duration used by menu item 9",
  )
  interactive.add_argument(
      "--connectivity-duration",
      type=float,
      default=3.0,
      help="duration used by menu item 8",
  )

  send_control = subparsers.add_parser(
      "send-control",
      help="send CHASSIS_CONTROL(0x10) to the MCU",
  )
  add_serial_arguments(send_control)
  send_control.add_argument(
      "--control-type",
      type=int,
      default=1,
      help="1=velocity, 2=follow, 3=swing, 4=spin",
  )
  send_control.add_argument("--vx", type=float, default=0.0, help="forward speed")
  send_control.add_argument("--vy", type=float, default=0.0, help="lateral speed")
  send_control.add_argument(
      "--wz",
      type=float,
      default=0.0,
      help="yaw rate in rad/s",
  )
  send_control.add_argument(
      "--duration",
      type=float,
      default=0.0,
      help="resend command for this long; 0 means send once",
  )
  send_control.add_argument(
      "--repeat-hz",
      type=float,
      default=10.0,
      help="resend rate when duration > 0",
  )
  send_control.add_argument(
      "--watch",
      type=float,
      default=2.0,
      help="watch MCU feedback after sending, in seconds",
  )

  send_raw = subparsers.add_parser(
      "send-raw",
      help="send a custom type id and raw payload bytes",
  )
  add_serial_arguments(send_raw)
  send_raw.add_argument(
      "type_id",
      type=lambda value: int(value, 0),
      help="message type, for example 0x10",
  )
  send_raw.add_argument(
      "payload_hex",
      help="payload bytes in hex, for example 010000803F0000000000000000",
  )
  send_raw.add_argument(
      "--watch",
      type=float,
      default=2.0,
      help="watch MCU feedback after sending, in seconds",
  )

  return parser


def main() -> int:
  parser = build_arg_parser()
  args = parser.parse_args()

  if args.command == "self-check":
    return run_self_check(args)
  if args.command == "list-ports":
    return run_list_ports(args)
  if args.command == "connectivity":
    return run_connectivity(args)
  if args.command == "monitor":
    return run_monitor(args)
  if args.command == "interactive":
    return run_interactive(args)
  if args.command == "send-control":
    return run_send_control(args)
  if args.command == "send-raw":
    return run_send_raw(args)

  parser.print_help()
  return 1


if __name__ == "__main__":
  try:
    sys.exit(main())
  except KeyboardInterrupt:
    log("interrupted by user")
    sys.exit(130)