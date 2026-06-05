#!/usr/bin/env python3
"""
BMS Serial Monitor / Logger for the Teensy 4.1 LTC6813 BMS project.

What it does:
  - Reads the Teensy serial stream as fast as possible.
  - Logs EVERY received line to a timestamped CSV file.
  - Creates logs under: BMS_Serial_Laptop_Recieved/
  - Uses file names like: BMS_Serial_2026-06-04_19-22-10.csv
  - Shows a human-scale dashboard instead of flooding the terminal.
  - Parses the current BMS serial print format in setup, charge, standby, drive, and debug modes.

Install dependency:
  python -m pip install pyserial

Examples:
  python tools/bms_serial_monitor.py --list
  python tools/bms_serial_monitor.py --port COM7 --baud 9600
  python tools/bms_serial_monitor.py --auto --baud 9600
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import shutil
import sys
import time
import threading
from collections import defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing dependency: pyserial")
    print("Install it with:")
    print("  python -m pip install pyserial")
    sys.exit(2)


# CAN IDs used by the current BMS project.
CHARGER_CMD_ID = "1806E5F4"      # BMS -> charger, extended
CHARGER_STATUS_ID = "18FF50E5"   # charger -> BMS, extended
BMS_TX_ID = "7"                  # current TX_CAN() prints/sends standard ID 0x00000007


# Full debug-sketch style CAN lines:
TX_RE = re.compile(
    r"TX ID=0x([0-9A-Fa-f]+)\s+EXT=(\d+)\s+DATA=([0-9A-Fa-f ]+)\s+write=(\d+)"
)
RX_RE = re.compile(
    r"RX ID=0x([0-9A-Fa-f]+)\s+EXT=(\d+)\s+LEN=(\d+)\s+DATA=([0-9A-Fa-f ]+)"
)

# Current BMS code RX_CAN() format:
#   ID: 18FF50E5 Data:
#   0 1 0 0 0 E 0 3D
BMS_RX_ID_RE = re.compile(r"^ID:\s*([0-9A-Fa-f]+)\s+Data:\s*$")
HEX_BYTE_LINE_RE = re.compile(r"^\s*([0-9A-Fa-f]{1,2}\s*){1,8}\s*$")

# Current BMS code charger_enable() format:
#   Charger CAN Message: 16 F8 0 14 1 0 0 0
CHARGER_CAN_MESSAGE_RE = re.compile(r"^Charger CAN Message:\s*([0-9A-Fa-f ]+)")

# Generic label/value prints, such as:
#   Pack Voltage: 582.4
#   Max cell voltage: 4.1000
#   SOC: 99.5
LABEL_VALUE_RE = re.compile(r"^([A-Za-z][A-Za-z0-9 _/().%-]*?):\s*([-+]?\d+(?:\.\d+)?)\s*$")
LABEL_ONLY_RE = re.compile(r"^([A-Za-z][A-Za-z0-9 _/().%-]*?):\s*$")
NUMBER_ONLY_RE = re.compile(r"^\s*([-+]?\d+(?:\.\d+)?)\s*$")

# Debug sketch decoded charger line, supported too:
DECODED_CHARGER_RE = re.compile(
    r"Decoded:\s*Vout=([-+0-9.]+)\s*V,\s*Iout=([-+0-9.]+)\s*A,\s*Status=0x([0-9A-Fa-f]+)"
)


KEY_ALIASES = {
    "time (minutes)": ("charge_time_min", "min"),
    "charge fault status": ("charger_fault_status_raw", "raw"),
    "current": ("current_a", "A"),
    "pack voltage": ("pack_voltage_v", "V"),
    "charger voltage": ("charger_voltage_v", "V"),
    "charger current": ("charger_current_a", "A"),
    "max cell voltage": ("max_cell_voltage_v", "V"),
    "max_cell_voltage": ("max_cell_voltage_v", "V"),
    "max cell_voltage": ("max_cell_voltage_v", "V"),
    "min cell voltage": ("min_cell_voltage_v", "V"),
    "min_cell_voltage": ("min_cell_voltage_v", "V"),
    "min cell_voltage": ("min_cell_voltage_v", "V"),

    # Temperature labels seen across the BMS code/log output.
    # These all map to the dashboard's Max_temp / Min_temp fields.
    "max_temp": ("max_cell_temp_c", "C"),
    "max temp": ("max_cell_temp_c", "C"),
    "max cell temp": ("max_cell_temp_c", "C"),
    "max_cell_temp": ("max_cell_temp_c", "C"),
    "max cell_temp": ("max_cell_temp_c", "C"),
    "min_temp": ("min_cell_temp_c", "C"),
    "min temp": ("min_cell_temp_c", "C"),
    "min cell temp": ("min_cell_temp_c", "C"),
    "min_cell_temp": ("min_cell_temp_c", "C"),
    "min cell_temp": ("min_cell_temp_c", "C"),

    "max die temp": ("max_die_temp_c", "C"),
    "max_die_temp": ("max_die_temp_c", "C"),
    "min die temp": ("min_die_temp_c", "C"),
    "min_die_temp": ("min_die_temp_c", "C"),
    "soc": ("soc_percent", "%"),
    "power limit": ("power_limit_kw", "kW"),
    "memory usage": ("sd_memory_usage_percent", "%"),
    "adc_a": ("adc_a_raw", "raw"),
    "adc_b": ("adc_b_raw", "raw"),
    "a voltage": ("adc_a_voltage_v", "V"),
    "b voltage": ("adc_b_voltage_v", "V"),
    "hall effect adc voltage": ("hall_adc_voltage_v", "V"),
}


MODE_LINES = {
    "startup": "startup",
    "Setup": "setup",
    "Charge Mode Entered": "charge",
    "Standby Mode Entered": "standby",
    "Drive Mode Entered": "drive",
    "Debug Mode Entered": "debug",
}


FAULT_KEYWORDS = [
    "fault",
    "error",
    "invalid",
    "open voltage sense lead",
    "open fusible link",
    "watchdog",
    "e-stop",
    "failed",
]


# These lines are still parsed, counted, decoded, and written to the CSV log,
# but they should not consume the fixed RECENT EVENTS slots on the dashboard.
# The dashboard already has dedicated CAN / SERIAL and CHARGER sections.
SUPPRESS_RECENT_EVENT_TYPES = {
    "can_tx",
    "can_rx",
    "can_rx_id_pending",
    "charger_can_command",
    "charger_status_decoded",
}


CSV_COLUMNS = [
    "pc_timestamp_iso",
    "pc_epoch_s",
    "elapsed_s",
    "line_number",
    "mode",
    "raw_line",
    "parsed_type",
    "key",
    "value",
    "unit",
    "can_direction",
    "can_id",
    "can_ext",
    "can_len",
    "can_data_hex",
    "charger_cmd_voltage_v",
    "charger_cmd_current_a",
    "charger_cmd_control",
    "charger_cmd_text",
    "charger_status_voltage_v",
    "charger_status_current_a",
    "charger_status_byte_hex",
    "charger_status_faults",
]


@dataclass
class MonitorState:
    start_time: float = field(default_factory=time.time)
    line_count: int = 0
    raw_line_rate_hz: float = 0.0
    mode: str = "unknown"
    last_line: str = ""
    last_tx_time: Optional[float] = None
    last_rx_time: Optional[float] = None
    tx_count: int = 0
    rx_count: int = 0
    id_counts: dict[str, int] = field(default_factory=lambda: defaultdict(int))
    latest_values: dict[str, tuple[Any, str]] = field(default_factory=dict)
    charger_cmd: dict[str, Any] = field(default_factory=dict)
    charger_status: dict[str, Any] = field(default_factory=dict)
    recent_events: deque[str] = field(default_factory=lambda: deque(maxlen=20))
    pending_label: Optional[str] = None
    pending_rx_id: Optional[str] = None


def clean_can_id(can_id: str) -> str:
    can_id = can_id.upper().lstrip("0")
    return can_id if can_id else "0"


def parse_hex_bytes(data_string: str) -> list[int]:
    out: list[int] = []
    for part in data_string.strip().split():
        try:
            out.append(int(part, 16) & 0xFF)
        except ValueError:
            pass
    return out


def hex_bytes(data: list[int]) -> str:
    return " ".join(f"{b:02X}" for b in data)


def charger_status_faults(status: int) -> list[str]:
    faults: list[str] = []
    if status & 0x01:
        faults.append("hardware failure")
    if status & 0x02:
        faults.append("over-temperature")
    if status & 0x04:
        faults.append("input voltage problem")
    if status & 0x08:
        faults.append("battery/output connection problem")
    if status & 0x10:
        faults.append("communication receive timeout")
    return faults


def decode_charger_command(data: list[int]) -> Optional[dict[str, Any]]:
    if len(data) < 5:
        return None
    voltage_raw = (data[0] << 8) | data[1]
    current_raw = (data[2] << 8) | data[3]
    control = data[4]
    return {
        "voltage_v": voltage_raw / 10.0,
        "current_a": current_raw / 10.0,
        "control": control,
        "text": "STOP / DISABLE" if control == 1 else "START / ENABLE" if control == 0 else f"UNKNOWN 0x{control:02X}",
    }


def decode_charger_status(data: list[int]) -> Optional[dict[str, Any]]:
    if len(data) < 5:
        return None
    voltage_raw = (data[0] << 8) | data[1]
    current_raw = (data[2] << 8) | data[3]
    status = data[4]
    return {
        "voltage_v": voltage_raw / 10.0,
        "current_a": current_raw / 10.0,
        "status": status,
        "faults": charger_status_faults(status),
    }


def normalize_label(label: str) -> tuple[str, str]:
    lookup = label.strip().lower()
    return KEY_ALIASES.get(lookup, (lookup.replace(" ", "_"), ""))


def guess_mode_from_line(line: str, old_mode: str) -> str:
    stripped = line.strip()
    if stripped in MODE_LINES:
        return MODE_LINES[stripped]
    # standby loop prints the variable "standby" repeatedly.
    if stripped in {"charge", "standby", "drive", "debug"}:
        return stripped
    return old_mode


def row_base(now: float, line_number: int, mode: str, raw_line: str) -> dict[str, Any]:
    return {
        "pc_timestamp_iso": datetime.now().isoformat(timespec="milliseconds"),
        "pc_epoch_s": f"{now:.3f}",
        "elapsed_s": f"{now - START_TIME:.3f}",
        "line_number": line_number,
        "mode": mode,
        "raw_line": raw_line,
        "parsed_type": "raw",
        "key": "",
        "value": "",
        "unit": "",
        "can_direction": "",
        "can_id": "",
        "can_ext": "",
        "can_len": "",
        "can_data_hex": "",
        "charger_cmd_voltage_v": "",
        "charger_cmd_current_a": "",
        "charger_cmd_control": "",
        "charger_cmd_text": "",
        "charger_status_voltage_v": "",
        "charger_status_current_a": "",
        "charger_status_byte_hex": "",
        "charger_status_faults": "",
    }


# Set by main() right before the serial loop starts. Used by row_base().
START_TIME = time.time()


def process_line(line: str, now: float, state: MonitorState) -> dict[str, Any]:
    state.line_count += 1
    state.last_line = line
    state.mode = guess_mode_from_line(line, state.mode)

    row = row_base(now, state.line_count, state.mode, line)
    important = False

    stripped = line.strip()
    lower = stripped.lower()

    if stripped in MODE_LINES or stripped in {"charge", "standby", "drive", "debug"}:
        row["parsed_type"] = "mode"
        row["key"] = "mode"
        row["value"] = state.mode
        important = True

    for word in FAULT_KEYWORDS:
        if word in lower:
            important = True
            if row["parsed_type"] == "raw":
                row["parsed_type"] = "event"
                row["key"] = "event"
                row["value"] = stripped
            break

    # Full debug-sketch TX line.
    m = TX_RE.search(stripped)
    if m:
        can_id = clean_can_id(m.group(1))
        ext = m.group(2)
        data = parse_hex_bytes(m.group(3))
        row.update({
            "parsed_type": "can_tx",
            "can_direction": "TX",
            "can_id": f"0x{can_id}",
            "can_ext": ext,
            "can_len": len(data),
            "can_data_hex": hex_bytes(data),
            "key": "can_tx",
            "value": f"0x{can_id}",
        })
        state.tx_count += 1
        state.last_tx_time = now
        state.id_counts[f"TX 0x{can_id}"] += 1
        if can_id == CHARGER_CMD_ID:
            cmd = decode_charger_command(data)
            if cmd:
                state.charger_cmd = cmd
                row.update({
                    "charger_cmd_voltage_v": f"{cmd['voltage_v']:.1f}",
                    "charger_cmd_current_a": f"{cmd['current_a']:.1f}",
                    "charger_cmd_control": f"0x{cmd['control']:02X}",
                    "charger_cmd_text": cmd["text"],
                })
                important = True

    # Full debug-sketch RX line.
    m = RX_RE.search(stripped)
    if m:
        can_id = clean_can_id(m.group(1))
        ext = m.group(2)
        length = int(m.group(3))
        data = parse_hex_bytes(m.group(4))
        row.update({
            "parsed_type": "can_rx",
            "can_direction": "RX",
            "can_id": f"0x{can_id}",
            "can_ext": ext,
            "can_len": length,
            "can_data_hex": hex_bytes(data),
            "key": "can_rx",
            "value": f"0x{can_id}",
        })
        state.rx_count += 1
        state.last_rx_time = now
        state.id_counts[f"RX 0x{can_id}"] += 1
        if can_id == CHARGER_STATUS_ID:
            status = decode_charger_status(data)
            if status:
                state.charger_status = status
                row.update({
                    "charger_status_voltage_v": f"{status['voltage_v']:.2f}",
                    "charger_status_current_a": f"{status['current_a']:.2f}",
                    "charger_status_byte_hex": f"0x{status['status']:02X}",
                    "charger_status_faults": "; ".join(status["faults"]),
                })
                important = True

    # Current BMS RX_CAN() first line.
    m = BMS_RX_ID_RE.match(stripped)
    if m:
        can_id = clean_can_id(m.group(1))
        state.pending_rx_id = can_id
        row.update({
            "parsed_type": "can_rx_id_pending",
            "can_direction": "RX",
            "can_id": f"0x{can_id}",
            "key": "can_rx_id_pending",
            "value": f"0x{can_id}",
        })
        important = True

    # Current BMS RX_CAN() second line with data bytes.
    elif state.pending_rx_id and HEX_BYTE_LINE_RE.match(stripped):
        can_id = state.pending_rx_id
        data = parse_hex_bytes(stripped)
        state.pending_rx_id = None
        row.update({
            "parsed_type": "can_rx",
            "can_direction": "RX",
            "can_id": f"0x{can_id}",
            "can_ext": "unknown",
            "can_len": len(data),
            "can_data_hex": hex_bytes(data),
            "key": "can_rx",
            "value": f"0x{can_id}",
        })
        state.rx_count += 1
        state.last_rx_time = now
        state.id_counts[f"RX 0x{can_id}"] += 1
        if can_id == CHARGER_STATUS_ID:
            status = decode_charger_status(data)
            if status:
                state.charger_status = status
                row.update({
                    "charger_status_voltage_v": f"{status['voltage_v']:.2f}",
                    "charger_status_current_a": f"{status['current_a']:.2f}",
                    "charger_status_byte_hex": f"0x{status['status']:02X}",
                    "charger_status_faults": "; ".join(status["faults"]),
                })
                important = True

    # Current BMS charger_enable() print line.
    m = CHARGER_CAN_MESSAGE_RE.match(stripped)
    if m:
        data = parse_hex_bytes(m.group(1))
        cmd = decode_charger_command(data)
        row.update({
            "parsed_type": "charger_can_command",
            "can_direction": "TX",
            "can_id": f"0x{CHARGER_CMD_ID}",
            "can_ext": "1",
            "can_len": len(data),
            "can_data_hex": hex_bytes(data),
            "key": "charger_can_command",
        })
        state.tx_count += 1
        state.last_tx_time = now
        state.id_counts[f"TX 0x{CHARGER_CMD_ID}"] += 1
        if cmd:
            state.charger_cmd = cmd
            row.update({
                "value": cmd["text"],
                "charger_cmd_voltage_v": f"{cmd['voltage_v']:.1f}",
                "charger_cmd_current_a": f"{cmd['current_a']:.1f}",
                "charger_cmd_control": f"0x{cmd['control']:02X}",
                "charger_cmd_text": cmd["text"],
            })
        important = True

    # Debug-sketch decoded charger line.
    m = DECODED_CHARGER_RE.search(stripped)
    if m:
        status_byte = int(m.group(3), 16)
        status = {
            "voltage_v": float(m.group(1)),
            "current_a": float(m.group(2)),
            "status": status_byte,
            "faults": charger_status_faults(status_byte),
        }
        state.charger_status = status
        row.update({
            "parsed_type": "charger_status_decoded",
            "key": "charger_status",
            "value": f"0x{status_byte:02X}",
            "charger_status_voltage_v": f"{status['voltage_v']:.2f}",
            "charger_status_current_a": f"{status['current_a']:.2f}",
            "charger_status_byte_hex": f"0x{status['status']:02X}",
            "charger_status_faults": "; ".join(status["faults"]),
        })
        important = True

    # Generic label-value line.
    m = LABEL_VALUE_RE.match(stripped)
    if m:
        label, raw_value = m.group(1), m.group(2)
        key, unit = normalize_label(label)
        try:
            value: Any = float(raw_value) if "." in raw_value else int(raw_value)
        except ValueError:
            value = raw_value
        state.latest_values[key] = (value, unit)
        row.update({
            "parsed_type": "value",
            "key": key,
            "value": value,
            "unit": unit,
        })
        important = key in {
            "charge_time_min",
            "charger_fault_status_raw",
            "current_a",
            "pack_voltage_v",
            "charger_voltage_v",
            "charger_current_a",
            "max_cell_voltage_v",
            "min_cell_voltage_v",
            "max_cell_temp_c",
            "min_cell_temp_c",
            "max_die_temp_c",
            "min_die_temp_c",
            "soc_percent",
            "power_limit_kw",
        }
        state.pending_label = None

    # Generic label only; next numeric-only line gets associated with this label.
    elif LABEL_ONLY_RE.match(stripped):
        label = LABEL_ONLY_RE.match(stripped).group(1)
        state.pending_label = label
        if row["parsed_type"] == "raw":
            row.update({
                "parsed_type": "label_pending",
                "key": normalize_label(label)[0],
            })

    # Numeric-only line after a label-only line.
    elif state.pending_label and NUMBER_ONLY_RE.match(stripped):
        key, unit = normalize_label(state.pending_label)
        raw_value = NUMBER_ONLY_RE.match(stripped).group(1)
        try:
            value = float(raw_value) if "." in raw_value else int(raw_value)
        except ValueError:
            value = raw_value
        state.latest_values[key] = (value, unit)
        row.update({
            "parsed_type": "value",
            "key": key,
            "value": value,
            "unit": unit,
        })
        state.pending_label = None
        important = True

    # Some lines should become visible in the dashboard event list.
    # CAN traffic is still logged and decoded, but it should not push BMS values
    # or fault messages out of the fixed-size RECENT EVENTS panel.
    if important and row["parsed_type"] not in SUPPRESS_RECENT_EVENT_TYPES:
        state.recent_events.append(stripped)

    return row


def list_serial_ports() -> list[Any]:
    ports = list(list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return []

    print("Available serial ports:")
    for p in ports:
        print(f"  {p.device:14s}  {p.description}")
    return ports


def auto_select_port() -> Optional[str]:
    ports = list(list_ports.comports())
    if not ports:
        return None

    preferred_words = ["teensy", "usb serial", "arduino", "serial"]
    scored: list[tuple[int, str, str]] = []
    for p in ports:
        desc = f"{p.description} {getattr(p, 'manufacturer', '')}".lower()
        score = 0
        for i, word in enumerate(preferred_words):
            if word in desc:
                score += 10 - i
        scored.append((score, p.device, p.description))

    scored.sort(reverse=True)
    if scored[0][0] > 0:
        return scored[0][1]
    if len(ports) == 1:
        return ports[0].device
    return None


def clear_screen() -> None:
    os.system("cls" if os.name == "nt" else "clear")


def value_text(state: MonitorState, key: str, width: int = 9) -> str:
    if key not in state.latest_values:
        return "--".rjust(width)
    value, unit = state.latest_values[key]
    if isinstance(value, float):
        return f"{value:{width}.3f} {unit}".rstrip()
    return f"{str(value):>{width}} {unit}".rstrip()


def age_text(t: Optional[float], now: Optional[float] = None) -> str:
    if t is None:
        return "never"
    if now is None:
        now = time.time()
    return f"{now - t:.2f} s ago"


def print_dashboard(state: MonitorState, log_path: Path, dashboard: str) -> None:
    """
    Print a fixed-height dashboard.

    Important:
      - Every refresh prints the same number of visual lines.
      - Every printed line is clipped to the terminal width to prevent wrapping.
      - Variable-length sections, such as ID counts and recent events, are padded.
    """
    now = time.time()
    runtime = now - state.start_time

    target_width = 92
    terminal_width = shutil.get_terminal_size((target_width, 30)).columns
    width = max(60, min(target_width, terminal_width - 1))

    def fit(text: Any = "") -> str:
        text = str(text).replace("\t", " ")
        if len(text) > width:
            if width > 4:
                text = text[: width - 3] + "..."
            else:
                text = text[:width]
        return text.ljust(width)

    lines: list[str] = []

    def emit(text: Any = "") -> None:
        lines.append(fit(text))

    def short_path(path: Path, max_len: int) -> str:
        s = str(path)
        if len(s) <= max_len:
            return s
        name = path.name
        parent = path.parent.name
        suffix = f"...\\{parent}\\{name}"
        if len(suffix) <= max_len:
            return suffix
        return "..." + s[-(max_len - 3):]

    def value_fixed(key: str, width_chars: int = 12) -> str:
        return value_text(state, key, width_chars)[:width_chars].ljust(width_chars)

    def charger_command_line() -> str:
        cmd = state.charger_cmd
        if not cmd:
            return "Command: not seen yet"
        return (
            f"Command: {cmd['text']:<16s}  "
            f"Request: {cmd['voltage_v']:7.1f} V, {cmd['current_a']:6.1f} A, "
            f"byte4=0x{cmd['control']:02X}"
        )

    def charger_status_line() -> str:
        st = state.charger_status
        if not st:
            return "Status:  not seen yet"
        fault_text = "; ".join(st["faults"]) if st["faults"] else "none"
        return (
            f"Status:  Vout={st['voltage_v']:7.2f} V, "
            f"Iout={st['current_a']:6.2f} A, "
            f"status=0x{st['status']:02X}, faults={fault_text}"
        )

    count_items = [f"{k}:{v}" for k, v in sorted(state.id_counts.items())[:6]]
    while len(count_items) < 6:
        count_items.append("--")

    events = list(state.recent_events)[-10:]
    while len(events) < 10:
        events.insert(0, "")

    emit("=" * width)
    emit("BMS SERIAL LAPTOP MONITOR")
    emit("=" * width)
    emit(f"Mode: {state.mode:<10s}  Runtime: {runtime:9.1f} s  Lines: {state.line_count:8d}  Rate: {state.raw_line_rate_hz:6.1f} lines/s")
    emit(f"Log: {short_path(log_path, max(10, width - 5))}")
    emit()

    emit("BMS VALUES")
    emit("-" * width)
    emit(f"Pack Voltage:       {value_fixed('pack_voltage_v')}    Current:          {value_fixed('current_a')}")
    emit(f"SOC:                {value_fixed('soc_percent')}    Power Limit:      {value_fixed('power_limit_kw')}")
    emit(f"Max Cell Voltage:   {value_fixed('max_cell_voltage_v')}    Min Cell Voltage: {value_fixed('min_cell_voltage_v')}")
    emit(f"Max_temp:           {value_fixed('max_cell_temp_c')}    Min_temp:         {value_fixed('min_cell_temp_c')}")
    emit(f"Max Die Temp:       {value_fixed('max_die_temp_c')}    Min Die Temp:     {value_fixed('min_die_temp_c')}")
    emit()

    emit("CHARGER")
    emit("-" * width)
    emit(charger_command_line())
    emit(charger_status_line())
    emit()

    emit("CAN / SERIAL")
    emit("-" * width)
    emit(f"TX frames parsed: {state.tx_count:<8d}  last TX: {age_text(state.last_tx_time, now):<12s}")
    emit(f"RX frames parsed: {state.rx_count:<8d}  last RX: {age_text(state.last_rx_time, now):<12s}")
    emit(f"ID counts 1: {count_items[0]:<22s} {count_items[1]:<22s} {count_items[2]:<22s}")
    emit(f"ID counts 2: {count_items[3]:<22s} {count_items[4]:<22s} {count_items[5]:<22s}")
    emit()

    emit("RECENT EVENTS")
    emit("-" * width)
    for i, event in enumerate(events, start=1):
        emit(f"{i:02d}: {event}")

    emit()
    emit("Ctrl+C stops. The CSV log keeps every received serial line.")
    emit("Optional serial input mode: run with --input, then type a line and press Enter.")
    emit("=" * width)

    # Write the entire dashboard in one terminal update.
    # This prevents sections from appearing to update one-at-a-time.
    output = "\n".join(lines) + "\n"
    if dashboard == "clear":
        # ANSI clear-screen + cursor-home works in the VS Code terminal on Windows.
        sys.stdout.write("\033[2J\033[H")
    sys.stdout.write(output)
    sys.stdout.flush()

def input_thread_fn(ser: serial.Serial, stop_event: threading.Event) -> None:
    while not stop_event.is_set():
        try:
            line = sys.stdin.readline()
        except Exception:
            return
        if not line:
            time.sleep(0.05)
            continue
        text = line.rstrip("\r\n")
        if text.lower() in {"/quit", "/exit"}:
            stop_event.set()
            return
        try:
            ser.write((text + "\n").encode("utf-8"))
            ser.flush()
        except Exception as exc:
            print(f"Failed to send serial input: {exc}")


def main() -> int:
    global START_TIME

    parser = argparse.ArgumentParser(description="BMS human-scale serial monitor and timestamped CSV logger")
    parser.add_argument("--port", help="Serial port, example COM7, COM12, /dev/ttyACM0")
    parser.add_argument("--auto", action="store_true", help="Auto-select a likely Teensy/USB serial port")
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate. Current BMS code uses Serial.begin(9600).")
    parser.add_argument("--refresh", type=float, default=1.0, help="Dashboard refresh time in seconds")
    parser.add_argument("--dashboard", choices=["clear", "compact", "none"], default="clear", help="Dashboard display style")
    parser.add_argument("--raw", action="store_true", help="Also print every raw serial line live")
    parser.add_argument("--input", action="store_true", help="Allow typing lines to send to the Teensy")
    parser.add_argument("--log-dir", default="BMS_Serial_Laptop_Recieved", help="Folder for timestamped CSV logs")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return 0

    port = args.port
    if not port and args.auto:
        port = auto_select_port()

    if not port:
        print("No serial port selected.")
        print()
        list_serial_ports()
        print()
        print("Examples:")
        print("  python tools/bms_serial_monitor.py --port COM7 --baud 9600")
        print("  python tools/bms_serial_monitor.py --auto --baud 9600")
        return 2

    log_dir = Path(args.log_dir)
    log_dir.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    log_path = log_dir / f"BMS_Serial_{timestamp}.csv"

    state = MonitorState()
    START_TIME = state.start_time
    line_times = deque(maxlen=5000)
    stop_event = threading.Event()

    print(f"Opening serial port {port} at {args.baud} baud")
    print(f"CSV log file: {log_path}")
    print("Close Arduino Serial Monitor / PlatformIO built-in monitor if the port is busy.")

    try:
        ser = serial.Serial(port, args.baud, timeout=0.05)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}")
        return 3

    with ser:
        if args.input:
            threading.Thread(target=input_thread_fn, args=(ser, stop_event), daemon=True).start()
            print("Serial input enabled. Type text and press Enter to send it to the Teensy. Type /quit to stop.")

        with log_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
            writer.writeheader()

            last_dashboard = 0.0
            last_flush = time.time()

            try:
                while not stop_event.is_set():
                    raw = ser.readline()
                    now = time.time()

                    if raw:
                        line_times.append(now)
                        try:
                            line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                        except Exception:
                            line = repr(raw)

                        row = process_line(line, now, state)
                        writer.writerow(row)

                        if args.raw:
                            print(line)

                    # Maintain 1-second line-rate window.
                    while line_times and now - line_times[0] > 1.0:
                        line_times.popleft()
                    state.raw_line_rate_hz = float(len(line_times))

                    if now - last_flush >= 1.0:
                        f.flush()
                        last_flush = now

                    if args.dashboard != "none" and now - last_dashboard >= args.refresh:
                        last_dashboard = now
                        print_dashboard(state, log_path, args.dashboard)

            except KeyboardInterrupt:
                pass
            finally:
                f.flush()
                stop_event.set()
                print(f"\nStopped. CSV log saved to: {log_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
