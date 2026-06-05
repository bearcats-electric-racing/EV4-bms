#!/usr/bin/env python3
"""
BMS Serial GUI / Logger for the Teensy 4.1 LTC6813 BMS project.

Features:
  - Reads the Teensy serial stream using pyserial.
  - Logs every received line to a timestamped CSV file.
  - Shows the same live dashboard information as the text monitor.
  - Adds visual min/max voltage and temperature dials.
  - Maps per-board voltage and temperature data onto the module images.

Install:
  python -m pip install pyserial pillow

Run:
  python bms_serial_gui.py --auto --baud 9600
  python bms_serial_gui.py --port COM7 --baud 9600
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import queue
import re
import sys
import threading
import time
import tkinter as tk
from collections import defaultdict, deque
from dataclasses import dataclass, field
from datetime import datetime
from pathlib import Path
from tkinter import ttk
from typing import Any, Optional

try:
    import serial
    from serial.tools import list_ports
except ImportError:
    print("Missing dependency: pyserial")
    print("Install it with:")
    print("  python -m pip install pyserial")
    raise SystemExit(2)

try:
    from PIL import Image, ImageTk
except ImportError:
    print("Missing dependency: Pillow")
    print("Install it with:")
    print("  python -m pip install pillow")
    raise SystemExit(2)


# ------------------------------
# Serial / BMS parsing constants
# ------------------------------

CHARGER_CMD_ID = "1806E5F4"      # BMS -> charger, extended
CHARGER_STATUS_ID = "18FF50E5"   # charger -> BMS, extended
BMS_TX_ID = "7"

TX_RE = re.compile(
    r"TX ID=0x([0-9A-Fa-f]+)\s+EXT=(\d+)\s+DATA=([0-9A-Fa-f ]+)\s+write=(\d+)"
)
RX_RE = re.compile(
    r"RX ID=0x([0-9A-Fa-f]+)\s+EXT=(\d+)\s+LEN=(\d+)\s+DATA=([0-9A-Fa-f ]+)"
)

BMS_RX_ID_RE = re.compile(r"^ID:\s*([0-9A-Fa-f]+)\s+Data:\s*$")
HEX_BYTE_LINE_RE = re.compile(r"^\s*([0-9A-Fa-f]{1,2}\s*){1,8}\s*$")
CHARGER_CAN_MESSAGE_RE = re.compile(r"^Charger CAN Message:\s*([0-9A-Fa-f ]+)")

LABEL_VALUE_RE = re.compile(r"^([A-Za-z][A-Za-z0-9 _/().%-]*?):\s*([-+]?\d+(?:\.\d+)?)\s*$")
LABEL_ONLY_RE = re.compile(r"^([A-Za-z][A-Za-z0-9 _/().%-]*?):\s*$")
NUMBER_ONLY_RE = re.compile(r"^\s*([-+]?\d+(?:\.\d+)?)\s*$")
FLOAT_LIST_RE = re.compile(r"^\s*[-+]?\d+(?:\.\d+)?(?:\s+[-+]?\d+(?:\.\d+)?)+\s*$")
BOARD_RE = re.compile(r"^\s*Board:\s*(\d+)\s*$", re.IGNORECASE)
SECTION_RE = re.compile(r"^\s*(Voltages|Temperatures):\s*$", re.IGNORECASE)

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
    "min cell voltage": ("min_cell_voltage_v", "V"),
    "min cell_voltage": ("min_cell_voltage_v", "V"),
    "max_temp": ("max_cell_temp_c", "C"),
    "max temp": ("max_cell_temp_c", "C"),
    "max cell temp": ("max_cell_temp_c", "C"),
    "max cell_temp": ("max_cell_temp_c", "C"),
    "min_temp": ("min_cell_temp_c", "C"),
    "min temp": ("min_cell_temp_c", "C"),
    "min cell temp": ("min_cell_temp_c", "C"),
    "min cell_temp": ("min_cell_temp_c", "C"),
    "max die temp": ("max_die_temp_c", "C"),
    "min die temp": ("min_die_temp_c", "C"),
    "soc": ("soc_percent", "%"),
    "power limit": ("power_limit_kw", "kW"),
    "memory usage": ("sd_memory_usage_percent", "%"),
    "adc_a": ("adc_a_raw", "raw"),
    "adc_b": ("adc_b_raw", "raw"),
    "a voltage": ("adc_a_voltage_v", "V"),
    "b voltage": ("adc_b_voltage_v", "V"),
    "hall effect adc voltage": ("hall_adc_voltage_v", "V"),
    "current zero voltage": ("current_zero_voltage", "V"),
    "current zero voltage calibrated": ("current_zero_voltage_calibrated", "V"),
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

SUPPRESS_RECENT_EVENT_TYPES = {
    "can_tx",
    "can_rx",
    "can_rx_id_pending",
    "charger_can_command",
    "charger_status_decoded",
    "board_voltages",
    "board_temperatures",
}

DASHBOARD_NAN = "NaN"
ACTIVE_ALERT_LINES = 8
RECENT_EVENT_LINES = 12
EXPECTED_PACK_VOLTAGE_BOARDS = tuple(range(1, 11))
CELLS_PER_BOARD_FOR_PACK_VOLTAGE = 14

GUI_BG_NORMAL = "#0D1117"
GUI_BG_FAULT = "#4A1010"
GUI_BG_ORANGE = "#5A3300"
CELL_VOLTAGE_MIN_FAULT = 2.50
CELL_VOLTAGE_MAX_FAULT = 4.20
CELL_VOLTAGE_ORANGE_SENTINEL_LOW = 1.45
CELL_VOLTAGE_ORANGE_SENTINEL_HIGH = 1.55
CELL_TEMP_MAX_FAULT_C = 60.0
FLASH_PERIOD_S = 0.50
SERIAL_DATA_ALIVE_TIMEOUT_S = 2.0
CELL_VOLTAGE_ACTIVITY_FLASH_S = 0.60
CELL_VOLTAGE_REFRESH_WINDOW_SAMPLES = 50

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
    "array_type",
    "board",
    "array_values",
]


# ------------------------------
# Visual mapping configuration
# ------------------------------
#
# Coordinates are in source-image pixels:
#   Left_Side_module.png:  2048 x 790
#   Right_Side_Module.png: 2048 x 795
#
# If the overlay needs small alignment corrections, change these dictionaries.
# The GUI scales them automatically when the image is displayed at a different size.

PAIR_COLORS = {
    1: "#00AEEF",  # boards 1/2
    2: "#7AC943",  # boards 3/4
    3: "#FFD23F",  # boards 5/6
    4: "#FF8C1A",  # boards 7/8
    5: "#A66CFF",  # boards 9/10
}

# Left image shows the physical left-side cell overlay.
# Per the requested display rule, it only draws EVEN-numbered cell voltages.
LEFT_CELL_COORDS = {
    2: (665, 390),
    4: (950, 230),
    6: (1020, 390),
    8: (1290, 230),
    10: (1395, 390),
    12: (1665, 230),
    14: (1810, 455),  # moved down to clear nearby temp circle
    16: (1625, 625),  # moved down slightly
    18: (1445, 560),
    20: (1245, 535),
    22: (890, 615),
    24: (720, 560),
    26: (525, 535),
    28: (420, 440),
}

# Right image shows the physical right-side cell overlay.
# Per the requested display rule, it only draws ODD-numbered cell voltages.
RIGHT_CELL_COORDS = {
    1: (1340, 260),
    3: (1165, 325),
    5: (1000, 270),
    7: (910, 395),  # moved down/right
    9: (640, 275),
    11: (400, 375),
    13: (195, 250),
    15: (250, 590),
    17: (400, 500),
    19: (780, 705),  # moved down
    21: (960, 610),
    23: (1195, 550),  # moved up to cover Cell 23 text
    25: (1500, 705),  # moved down
    27: (1730, 645),
}

# Physical thermistor labels:
#   odd U numbers are on the left-side image
#   even U numbers are on the right-side image
LEFT_TEMP_COORDS = {
    1: (840, 360),
    3: (1210, 390),
    5: (1575, 390),
    7: (1780, 315),
    9: (1785, 620),
    11: (1670, 505),
    13: (1290, 640),
    15: (930, 520),
    17: (660, 645),
    19: (340, 615),
}

RIGHT_TEMP_COORDS = {
    2: (1450, 400),
    4: (1035, 440),
    6: (745, 335),
    8: (650, 425),
    10: (160, 500),
    12: (400, 620),
    14: (610, 600),
    16: (760, 580),
    18: (1330, 610),
    20: (1570, 605),
}


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
    active_alerts: dict[str, str] = field(default_factory=dict)
    recent_events: deque[str] = field(default_factory=lambda: deque(maxlen=30))
    pending_label: Optional[str] = None
    pending_label_line: Optional[int] = None
    pending_rx_id: Optional[str] = None
    array_section: Optional[str] = None
    pending_array_board: Optional[int] = None
    board_voltages: dict[int, list[float]] = field(default_factory=dict)
    board_temps: dict[int, list[float]] = field(default_factory=dict)
    log_path: Optional[Path] = None
    serial_status: str = "disconnected"
    last_serial_line_time: Optional[float] = None
    last_cell_voltage_update_time: Optional[float] = None
    cell_voltage_refresh_intervals_ms: deque[float] = field(
        default_factory=lambda: deque(maxlen=CELL_VOLTAGE_REFRESH_WINDOW_SAMPLES)
    )


START_TIME = time.time()


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


def parse_float_list(line: str) -> list[float]:
    out: list[float] = []
    for part in line.strip().split():
        try:
            out.append(float(part))
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


def set_active_alert(state: MonitorState, key: str, text: str, active: bool = True) -> None:
    if active:
        state.active_alerts[key] = text
    else:
        state.active_alerts.pop(key, None)


def update_charger_active_alert(state: MonitorState, status: dict[str, Any]) -> None:
    faults = status.get("faults", [])
    if faults:
        set_active_alert(state, "charger_status", "Charger: " + "; ".join(faults), True)
    else:
        set_active_alert(state, "charger_status", "", False)


def update_numeric_active_alert(state: MonitorState, key: str, value: Any) -> None:
    if key == "charger_fault_status_raw":
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            numeric = 0.0
        set_active_alert(state, key, f"Charge fault status: {value}", numeric != 0.0)


def get_latest_float(state: MonitorState, key: str) -> Optional[float]:
    if key not in state.latest_values:
        return None
    value, _unit = state.latest_values[key]
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def update_derived_active_alerts(state: MonitorState) -> None:
    now = time.time()

    if state.charger_status:
        update_charger_active_alert(state, state.charger_status)

    if state.mode == "charge":
        set_active_alert(state, "charger_cmd_missing", "Charge mode: charger command not seen yet", not bool(state.charger_cmd))
        set_active_alert(state, "charger_status_missing", "Charge mode: charger status not seen yet", not bool(state.charger_status))
    else:
        set_active_alert(state, "charger_cmd_missing", "", False)
        set_active_alert(state, "charger_status_missing", "", False)

    if state.tx_count > 0 and state.last_tx_time is not None:
        age = now - state.last_tx_time
        set_active_alert(state, "tx_stale", f"No parsed CAN TX for {age:.1f} s", age > 2.0)
    else:
        set_active_alert(state, "tx_stale", "", False)

    if state.rx_count > 0 and state.last_rx_time is not None:
        age = now - state.last_rx_time
        set_active_alert(state, "rx_stale", f"No parsed CAN RX for {age:.1f} s", age > 3.0)
    else:
        set_active_alert(state, "rx_stale", "", False)

    max_cell_v = get_latest_float(state, "max_cell_voltage_v")
    min_cell_v = get_latest_float(state, "min_cell_voltage_v")
    set_active_alert(
        state,
        "cell_overvoltage",
        f"Cell overvoltage: max cell = {max_cell_v:.3f} V" if max_cell_v is not None else "",
        max_cell_v is not None and max_cell_v > 4.20,
    )
    set_active_alert(
        state,
        "cell_undervoltage",
        f"Cell undervoltage: min cell = {min_cell_v:.3f} V" if min_cell_v is not None else "",
        min_cell_v is not None and min_cell_v < 2.50,
    )
    if max_cell_v is not None and min_cell_v is not None:
        imbalance = max_cell_v - min_cell_v
        set_active_alert(state, "cell_imbalance", f"Cell imbalance: {imbalance:.3f} V", imbalance > 0.100)
    else:
        set_active_alert(state, "cell_imbalance", "", False)

    max_temp = get_latest_float(state, "max_cell_temp_c")
    min_temp = get_latest_float(state, "min_cell_temp_c")
    set_active_alert(
        state,
        "cell_overtemp",
        f"Cell over-temperature: max = {max_temp:.1f} C" if max_temp is not None else "",
        max_temp is not None and max_temp >= 60.0,
    )
    set_active_alert(
        state,
        "cell_undertemp",
        f"Cell under-temperature: min = {min_temp:.1f} C" if min_temp is not None else "",
        min_temp is not None and min_temp < -20.0,
    )


def normalize_label(label: str) -> tuple[str, str]:
    lookup = label.strip().lower()
    return KEY_ALIASES.get(lookup, (lookup.replace(" ", "_"), ""))


def guess_mode_from_line(line: str, old_mode: str) -> str:
    stripped = line.strip()
    if stripped in MODE_LINES:
        return MODE_LINES[stripped]
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
        "array_type": "",
        "board": "",
        "array_values": "",
    }


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
            if "can message tx failed" in lower:
                set_active_alert(state, "can_tx_failed", stripped, True)
            else:
                set_active_alert(state, "latest_fault_event", stripped, True)
            break

    m = SECTION_RE.match(stripped)
    if m:
        section = m.group(1).lower()
        state.array_section = "voltages" if section.startswith("voltage") else "temperatures"
        state.pending_array_board = None
        row["parsed_type"] = "array_section"
        row["key"] = state.array_section
        row["array_type"] = state.array_section
        important = True

    m = TX_RE.search(stripped)
    if m:
        can_id = clean_can_id(m.group(1))
        ext = m.group(2)
        data = parse_hex_bytes(m.group(3))
        write_ok = m.group(4) == "1"
        set_active_alert(state, "can_tx_failed", "CAN message TX Failed", not write_ok)
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
                update_charger_active_alert(state, status)
                row.update({
                    "charger_status_voltage_v": f"{status['voltage_v']:.2f}",
                    "charger_status_current_a": f"{status['current_a']:.2f}",
                    "charger_status_byte_hex": f"0x{status['status']:02X}",
                    "charger_status_faults": "; ".join(status["faults"]),
                })
                important = True

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
                update_charger_active_alert(state, status)
                row.update({
                    "charger_status_voltage_v": f"{status['voltage_v']:.2f}",
                    "charger_status_current_a": f"{status['current_a']:.2f}",
                    "charger_status_byte_hex": f"0x{status['status']:02X}",
                    "charger_status_faults": "; ".join(status["faults"]),
                })
                important = True

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
        update_charger_active_alert(state, status)
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

    # Capture board number during voltage/temperature blocks.
    m = BOARD_RE.match(stripped)
    if m and state.array_section:
        state.pending_array_board = int(m.group(1))
        row["board"] = state.pending_array_board

    # Capture the numeric board voltage/temperature rows.
    if state.array_section and state.pending_array_board is not None and FLOAT_LIST_RE.match(stripped):
        values = parse_float_list(stripped)
        board = state.pending_array_board
        if state.array_section == "voltages":
            state.board_voltages[board] = values[:14]

            # Used by the Status panel's cell-voltage activity indicator.
            # The rolling refresh time is measured between received board-voltage rows.
            if state.last_cell_voltage_update_time is not None:
                dt_ms = (now - state.last_cell_voltage_update_time) * 1000.0
                if 0.0 < dt_ms < 60000.0:
                    state.cell_voltage_refresh_intervals_ms.append(dt_ms)

            state.last_cell_voltage_update_time = now
            row.update({
                "parsed_type": "board_voltages",
                "key": "board_voltages",
                "array_type": "voltages",
                "board": board,
                "array_values": " ".join(f"{v:.4g}" for v in values[:14]),
                "value": f"board {board}",
            })
        else:
            state.board_temps[board] = values[:10]
            row.update({
                "parsed_type": "board_temperatures",
                "key": "board_temperatures",
                "array_type": "temperatures",
                "board": board,
                "array_values": " ".join(f"{v:.4g}" for v in values[:10]),
                "value": f"board {board}",
            })
        state.pending_array_board = None
        important = True

    m = LABEL_VALUE_RE.match(stripped)
    if m:
        label, raw_value = m.group(1), m.group(2)
        key, unit = normalize_label(label)
        try:
            value: Any = float(raw_value) if "." in raw_value else int(raw_value)
        except ValueError:
            value = raw_value
        state.latest_values[key] = (value, unit)
        update_numeric_active_alert(state, key, value)
        row.update({
            "parsed_type": "value" if row["parsed_type"] == "raw" else row["parsed_type"],
            "key": key if not row.get("key") else row["key"],
            "value": value if row["parsed_type"] == "value" else row.get("value", value),
            "unit": unit,
        })
        important = important or key in {
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
            "current_zero_voltage",
            "current_zero_voltage_calibrated",
        }
        state.pending_label = None
        state.pending_label_line = None

    elif LABEL_ONLY_RE.match(stripped):
        label = LABEL_ONLY_RE.match(stripped).group(1)
        state.pending_label = label
        state.pending_label_line = state.line_count
        if row["parsed_type"] == "raw":
            row.update({
                "parsed_type": "label_pending",
                "key": normalize_label(label)[0],
            })

    elif (
        state.pending_label
        and state.pending_label_line is not None
        and state.line_count == state.pending_label_line + 1
        and NUMBER_ONLY_RE.match(stripped)
    ):
        key, unit = normalize_label(state.pending_label)
        raw_value = NUMBER_ONLY_RE.match(stripped).group(1)
        try:
            value = float(raw_value) if "." in raw_value else int(raw_value)
        except ValueError:
            value = raw_value
        state.latest_values[key] = (value, unit)
        update_numeric_active_alert(state, key, value)
        row.update({
            "parsed_type": "value",
            "key": key,
            "value": value,
            "unit": unit,
        })
        state.pending_label = None
        state.pending_label_line = None
        important = True

    if (
        state.pending_label is not None
        and state.pending_label_line is not None
        and state.line_count > state.pending_label_line
        and row["parsed_type"] != "label_pending"
    ):
        state.pending_label = None
        state.pending_label_line = None

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


def calculated_pack_voltage(state: MonitorState) -> Optional[float]:
    """
    Calculate pack voltage from the board voltage arrays.

    This is only used as a fallback when the BMS serial stream has not
    reported Pack Voltage directly. It requires every expected board to have
    a full cell-voltage row so the dashboard does not show a partial-pack sum.
    """
    total = 0.0

    for board in EXPECTED_PACK_VOLTAGE_BOARDS:
        values = state.board_voltages.get(board)

        if values is None or len(values) < CELLS_PER_BOARD_FOR_PACK_VOLTAGE:
            return None

        for raw_value in values[:CELLS_PER_BOARD_FOR_PACK_VOLTAGE]:
            try:
                total += float(raw_value)
            except (TypeError, ValueError):
                return None

    return total


def value_text(state: MonitorState, key: str, decimals: int = 3) -> str:
    if key not in state.latest_values:
        if key == "pack_voltage_v":
            pack_voltage = calculated_pack_voltage(state)
            if pack_voltage is not None:
                return f"{pack_voltage:.{decimals}f} V (cal)"
        return DASHBOARD_NAN

    value, unit = state.latest_values[key]
    if isinstance(value, float):
        return f"{value:.{decimals}f} {unit}".strip()
    return f"{value} {unit}".strip()


def age_text(t: Optional[float], now: Optional[float] = None) -> str:
    if t is None:
        return DASHBOARD_NAN
    if now is None:
        now = time.time()
    return f"{now - t:.2f} s ago"


def temp_color(temp: Optional[float]) -> str:
    if temp is None or math.isnan(temp) or temp <= -50.0:
        return "#333333"
    # Green <= 25C, yellow around 45C, red >= 60C.
    t = max(0.0, min(1.0, (temp - 25.0) / (60.0 - 25.0)))
    if t < 0.5:
        # green -> yellow
        k = t / 0.5
        r = int(40 + (255 - 40) * k)
        g = int(190 + (210 - 190) * k)
        b = int(80 + (0 - 80) * k)
    else:
        # yellow -> red
        k = (t - 0.5) / 0.5
        r = 255
        g = int(210 + (60 - 210) * k)
        b = 0
    return f"#{r:02X}{g:02X}{b:02X}"


def voltage_color(voltage: Optional[float]) -> str:
    """Purple at 2.5 V, blue at 4.2 V. Gray means unknown/not updated yet."""
    if voltage is None:
        return "#333333"
    try:
        v = float(voltage)
    except (TypeError, ValueError):
        return "#333333"
    if math.isnan(v):
        return "#333333"

    ratio = max(0.0, min(1.0, (v - 2.5) / (4.2 - 2.5)))

    # 2.5 V = purple, 4.2 V = blue
    r1, g1, b1 = 128, 0, 255
    r2, g2, b2 = 0, 122, 255
    r = int(r1 + (r2 - r1) * ratio)
    g = int(g1 + (g2 - g1) * ratio)
    b = int(b1 + (b2 - b1) * ratio)
    return f"#{r:02X}{g:02X}{b:02X}"


def readable_text_color(hex_color: str) -> str:
    hex_color = hex_color.lstrip("#")
    try:
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
    except Exception:
        return "white"
    luminance = 0.299 * r + 0.587 * g + 0.114 * b
    return "black" if luminance > 145 else "white"


class SerialWorker(threading.Thread):
    def __init__(
        self,
        state: MonitorState,
        state_lock: threading.RLock,
        stop_event: threading.Event,
        port: str,
        baud: int,
        log_dir: Path,
        status_queue: "queue.Queue[str]",
        raw_echo: bool = False,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.port = port
        self.baud = baud
        self.log_dir = log_dir
        self.status_queue = status_queue
        self.raw_echo = raw_echo

    def run(self) -> None:
        global START_TIME

        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        log_path = self.log_dir / f"BMS_Serial_{timestamp}.csv"

        with self.state_lock:
            self.state.log_path = log_path
            self.state.serial_status = f"Opening {self.port} @ {self.baud}"

        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.05)
        except serial.SerialException as exc:
            with self.state_lock:
                self.state.serial_status = f"Serial error: {exc}"
            self.status_queue.put(f"Serial error: {exc}")
            return

        with ser:
            with self.state_lock:
                self.state.serial_status = f"Connected: {self.port} @ {self.baud}"
                START_TIME = self.state.start_time

            line_times: deque[float] = deque(maxlen=5000)
            last_flush = time.time()

            try:
                with log_path.open("w", newline="", encoding="utf-8") as f:
                    writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
                    writer.writeheader()

                    while not self.stop_event.is_set():
                        raw = ser.readline()
                        now = time.time()

                        if raw:
                            line_times.append(now)
                            try:
                                line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                            except Exception:
                                line = repr(raw)
                            if self.raw_echo:
                                print(line)

                            with self.state_lock:
                                row = process_line(line, now, self.state)
                            writer.writerow(row)
                            # Green serial indicator is based on data actually being
                            # received and saved, not merely on the COM port opening.
                            with self.state_lock:
                                self.state.last_serial_line_time = now

                        while line_times and now - line_times[0] > 1.0:
                            line_times.popleft()
                        with self.state_lock:
                            self.state.raw_line_rate_hz = float(len(line_times))

                        if now - last_flush >= 1.0:
                            f.flush()
                            last_flush = now

            except Exception as exc:
                with self.state_lock:
                    self.state.serial_status = f"Logger error: {exc}"
                self.status_queue.put(f"Logger error: {exc}")
            finally:
                with self.state_lock:
                    self.state.serial_status = "Stopped"


class Dial(ttk.Frame):
    def __init__(
        self,
        parent: tk.Widget,
        title: str,
        min_value: float,
        max_value: float,
        redline_start: Optional[float] = None,
        units: str = "",
        width: int = 300,
        height: int = 190,
    ) -> None:
        super().__init__(parent)
        self.title = title
        self.min_value = min_value
        self.max_value = max_value
        self.redline_start = redline_start
        self.units = units
        self.width = width
        self.height = height
        self.canvas = tk.Canvas(self, width=width, height=height, bg="#101214", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)

    def _angle_for(self, value: float) -> float:
        v = max(self.min_value, min(self.max_value, value))
        ratio = (v - self.min_value) / (self.max_value - self.min_value)
        return math.radians(210 - ratio * 240)

    def _point(self, cx: float, cy: float, radius: float, value: float) -> tuple[float, float]:
        a = self._angle_for(value)
        return cx + radius * math.cos(a), cy - radius * math.sin(a)

    def draw(self, min_reading: Optional[float], max_reading: Optional[float]) -> None:
        c = self.canvas
        c.delete("all")
        w = self.width
        h = self.height
        cx = w / 2
        cy = h * 0.78
        r = min(w * 0.38, h * 0.58)

        c.create_text(cx, 18, text=self.title, fill="#F2F2F2", font=("Segoe UI", 12, "bold"))

        bbox = (cx - r, cy - r, cx + r, cy + r)
        c.create_arc(bbox, start=-30, extent=240, style="arc", width=14, outline="#30363D")

        if self.redline_start is not None:
            red_start_angle = 210 - ((self.redline_start - self.min_value) / (self.max_value - self.min_value)) * 240
            red_extent = ((self.max_value - self.redline_start) / (self.max_value - self.min_value)) * 240
            c.create_arc(bbox, start=-30, extent=red_extent, style="arc", width=14, outline="#D62828")

        # Tick marks
        for i in range(6):
            value = self.min_value + i * (self.max_value - self.min_value) / 5
            x1, y1 = self._point(cx, cy, r - 8, value)
            x2, y2 = self._point(cx, cy, r + 6, value)
            c.create_line(x1, y1, x2, y2, fill="#AAB2BD", width=2)
            xt, yt = self._point(cx, cy, r + 22, value)
            c.create_text(xt, yt, text=f"{value:g}", fill="#C9D1D9", font=("Segoe UI", 8))

        def draw_marker(value: Optional[float], color: str, label: str) -> None:
            if value is None:
                return
            x, y = self._point(cx, cy, r - 18, value)
            c.create_line(cx, cy, x, y, fill=color, width=4, arrow=tk.LAST)
            c.create_oval(x - 5, y - 5, x + 5, y + 5, fill=color, outline="white", width=1)
            c.create_text(x, y - 18, text=label, fill=color, font=("Segoe UI", 8, "bold"))

        draw_marker(min_reading, "#58A6FF", "MIN")
        draw_marker(max_reading, "#FF5C5C", "MAX")
        c.create_oval(cx - 5, cy - 5, cx + 5, cy + 5, fill="#F2F2F2", outline="")

        min_text = "NaN" if min_reading is None else f"{min_reading:.3g}{self.units}"
        max_text = "NaN" if max_reading is None else f"{max_reading:.3g}{self.units}"
        c.create_text(cx, h - 28, text=f"Min: {min_text}    Max: {max_text}", fill="#F2F2F2", font=("Segoe UI", 10, "bold"))


class ModuleMap(ttk.Frame):
    def __init__(
        self,
        parent: tk.Widget,
        title: str,
        image_path: Path,
        cell_coords: dict[int, tuple[int, int]],
        temp_coords: dict[int, tuple[int, int]],
        width: int = 840,
        voltage_parity: Optional[str] = None,
    ) -> None:
        super().__init__(parent)
        self.title = title
        self.image_path = image_path
        self.cell_coords = cell_coords
        self.temp_coords = temp_coords
        self.display_width = width
        self.voltage_parity = voltage_parity  # "even", "odd", or None
        self.source_image = Image.open(image_path).convert("RGBA")
        self.src_w, self.src_h = self.source_image.size
        self.display_height = int(self.src_h * (self.display_width / self.src_w))
        self.resized = self.source_image.resize((self.display_width, self.display_height), Image.LANCZOS)
        self.photo = ImageTk.PhotoImage(self.resized)

        ttk.Label(self, text=title, font=("Segoe UI", 11, "bold")).pack(anchor="w")
        self.canvas = tk.Canvas(
            self,
            width=self.display_width,
            height=self.display_height,
            bg="#070707",
            highlightthickness=1,
            highlightbackground="#333333",
        )
        self.canvas.pack(fill="both", expand=False)

    def scale_point(self, point: tuple[int, int]) -> tuple[float, float]:
        x, y = point
        return x * self.display_width / self.src_w, y * self.display_height / self.src_h

    def voltage_allowed(self, cell: int) -> bool:
        if self.voltage_parity == "even":
            return cell % 2 == 0
        if self.voltage_parity == "odd":
            return cell % 2 == 1
        return True

    @staticmethod
    def valid_temp_value(value: Any) -> Optional[float]:
        try:
            temp = float(value)
        except (TypeError, ValueError):
            return None
        # -55 C is the disconnected-thermistor sentinel in this BMS output.
        if temp <= -50.0:
            return None
        return temp

    @staticmethod
    def is_orange_voltage(value: Any) -> bool:
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False
        return CELL_VOLTAGE_ORANGE_SENTINEL_LOW <= v <= CELL_VOLTAGE_ORANGE_SENTINEL_HIGH

    @staticmethod
    def is_voltage_fault(value: Any) -> bool:
        try:
            v = float(value)
        except (TypeError, ValueError):
            return False
        if ModuleMap.is_orange_voltage(v):
            return False
        return v < CELL_VOLTAGE_MIN_FAULT or v > CELL_VOLTAGE_MAX_FAULT

    @staticmethod
    def is_temp_fault(value: Any) -> bool:
        temp = ModuleMap.valid_temp_value(value)
        return temp is not None and temp >= CELL_TEMP_MAX_FAULT_C

    @staticmethod
    def flashing_fill(base_fill: str, alert_fill: str, flash_on: bool) -> str:
        return alert_fill if flash_on else base_fill

    @staticmethod
    def minmax_pairs(values: dict[int, Optional[float]]) -> tuple[Optional[int], Optional[float], Optional[int], Optional[float]]:
        valid = [(idx, float(value)) for idx, value in values.items() if value is not None]
        if not valid:
            return None, None, None, None
        min_idx, min_val = min(valid, key=lambda item: item[1])
        max_idx, max_val = max(valid, key=lambda item: item[1])
        return min_idx, min_val, max_idx, max_val

    @staticmethod
    def fmt_voltage_item(item: Optional[dict[str, Any]]) -> str:
        if not item:
            return "NaN"
        board = item.get("board")
        cell = item.get("cell")
        value = item.get("value")
        if board is None or cell is None:
            return f"{value:.3f}V" if value is not None else "NaN"
        return f"B{board} C{cell} {value:.3f}V"

    @staticmethod
    def fmt_temp_item(item: Optional[dict[str, Any]]) -> str:
        if not item:
            return "NaN"
        board = item.get("board")
        sensor = item.get("sensor")
        value = item.get("value")
        if board is None or sensor is None:
            return f"{value:.1f}C" if value is not None else "NaN"
        return f"B{board} U{sensor} {value:.1f}C"

    def draw_allboard_range_overlay(
        self,
        board_color: str,
        allboard_ranges: dict[str, Any],
        flash_on: bool,
    ) -> None:
        """
        Min/Max-only overlay.

        Draws a min/max result for every physical cell and every physical
        thermistor location, using all boards that have reported data.

        Disconnected thermistors reported as -55 C are ignored.
        Faulty voltage/temp icons flash red.
        1.5 V sentinel voltage icons flash orange.
        """
        c = self.canvas
        voltage_ranges = allboard_ranges.get("voltage_ranges", {})
        temp_ranges = allboard_ranges.get("temp_ranges", {})

        def marker_pair_color(board: Optional[int]) -> str:
            if board is None:
                return "#FFFFFF"
            return PAIR_COLORS.get((int(board) + 1) // 2, "#FFFFFF")

        def item_text(item: Optional[dict[str, Any]], units: str, decimals: int) -> str:
            if not item:
                return f"NaN{units}"
            board = item.get("board")
            value = item.get("value")
            if value is None:
                return f"NaN{units}"
            prefix = f"B{board} " if board is not None else ""
            return f"{prefix}{float(value):.{decimals}f}{units}"

        # Draw every voltage location owned by this image side.
        for cell, point in self.cell_coords.items():
            if not self.voltage_allowed(cell):
                continue

            entry = voltage_ranges.get(cell, {})
            min_item = entry.get("min")
            max_item = entry.get("max")

            values_for_color = [
                float(item["value"])
                for item in (min_item, max_item)
                if item is not None and item.get("value") is not None
            ]
            color_value = sum(values_for_color) / len(values_for_color) if values_for_color else None
            fill = voltage_color(color_value)
            text_color = readable_text_color(fill)
            border = marker_pair_color(min_item.get("board") if min_item else (max_item.get("board") if max_item else None))

            has_orange = any(self.is_orange_voltage(item.get("value")) for item in (min_item, max_item) if item)
            has_fault = any(self.is_voltage_fault(item.get("value")) for item in (min_item, max_item) if item)

            x, y = self.scale_point(point)
            # Same height, reduced width so the voltage badges better fit the text.
            badge_w = 78 if self.display_width < 900 else 92
            badge_h = 50

            # Min/Max mode uses board-pair colors as the rectangle base:
            #   top half    = max value's board-pair color
            #   bottom half = min value's board-pair color
            max_base_fill = marker_pair_color(max_item.get("board") if max_item else None)
            min_base_fill = marker_pair_color(min_item.get("board") if min_item else None)

            top_fill = max_base_fill
            bottom_fill = min_base_fill
            top_text_color = readable_text_color(top_fill)
            bottom_text_color = readable_text_color(bottom_fill)

            if has_orange:
                top_fill = self.flashing_fill(top_fill, "#FF8C00", flash_on)
                bottom_fill = self.flashing_fill(bottom_fill, "#FF8C00", flash_on)
                border = "#FFD23F"
                top_text_color = "black"
                bottom_text_color = "black"
            elif has_fault:
                top_fill = self.flashing_fill(top_fill, "#FF2B2B", flash_on)
                bottom_fill = self.flashing_fill(bottom_fill, "#FF2B2B", flash_on)
                border = "#FFFFFF" if flash_on else "#FF2B2B"
                top_text_color = "white"
                bottom_text_color = "white"

            left = x - badge_w / 2
            right = x + badge_w / 2
            top = y - badge_h / 2
            middle = y
            bottom = y + badge_h / 2

            c.create_rectangle(left, top, right, middle, fill=top_fill, outline="", width=0)
            c.create_rectangle(left, middle, right, bottom, fill=bottom_fill, outline="", width=0)
            c.create_rectangle(
                left,
                top,
                right,
                bottom,
                fill="",
                outline=border,
                width=4 if (has_orange or has_fault) else 3,
            )

            c.create_text(
                x,
                top + badge_h * 0.25,
                text=f"C{cell} max\n{item_text(max_item, 'V', 3)}",
                fill=top_text_color,
                font=("Segoe UI", 7, "bold"),
                justify="center",
            )
            c.create_text(
                x,
                bottom - badge_h * 0.25,
                text=f"C{cell} min\n{item_text(min_item, 'V', 3)}",
                fill=bottom_text_color,
                font=("Segoe UI", 7, "bold"),
                justify="center",
            )

        # Draw every thermistor location owned by this image side.
        for sensor, point in self.temp_coords.items():
            entry = temp_ranges.get(sensor, {})
            min_item = entry.get("min")
            max_item = entry.get("max")

            values_for_color = [
                float(item["value"])
                for item in (min_item, max_item)
                if item is not None and item.get("value") is not None
            ]
            color_value = sum(values_for_color) / len(values_for_color) if values_for_color else None
            fill = temp_color(color_value)
            border = marker_pair_color(min_item.get("board") if min_item else (max_item.get("board") if max_item else None))

            has_fault = any(self.is_temp_fault(item.get("value")) for item in (min_item, max_item) if item)
            if has_fault:
                fill = self.flashing_fill(fill, "#FF2B2B", flash_on)
                border = "#FFFFFF" if flash_on else "#FF2B2B"

            x, y = self.scale_point(point)
            r = 27 if self.display_width < 900 else 31
            c.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline=border, width=5 if has_fault else 4)
            label = (
                f"U{sensor}\n"
                f"lo {item_text(min_item, 'C', 1)}\n"
                f"hi {item_text(max_item, 'C', 1)}"
            )
            c.create_text(
                x,
                y,
                text=label,
                fill="white" if fill != "#333333" else "#D0D0D0",
                font=("Segoe UI", 7, "bold"),
                justify="center",
            )

        overall_vmin = allboard_ranges.get("vmin")
        overall_vmax = allboard_ranges.get("vmax")
        overall_tmin = allboard_ranges.get("tmin")
        overall_tmax = allboard_ranges.get("tmax")

        c.create_rectangle(4, 4, 360, 30, fill="#101214", outline=board_color, width=2)
        c.create_text(
            14,
            17,
            anchor="w",
            text="All-board per-location Min/Max overlay",
            fill="#F2F2F2",
            font=("Segoe UI", 9, "bold"),
        )

        summary = (
            f"Overall Vmin: {self.fmt_voltage_item(overall_vmin)}    Overall Vmax: {self.fmt_voltage_item(overall_vmax)}\n"
            f"Overall Tmin: {self.fmt_temp_item(overall_tmin)}    Overall Tmax: {self.fmt_temp_item(overall_tmax)}"
        )
        c.create_rectangle(4, 34, 470, 78, fill="#101214", outline="#445", width=1)
        c.create_text(12, 56, anchor="w", text=summary, fill="#F2F2F2", font=("Segoe UI", 8, "bold"))

    def draw(
        self,
        board_label: str,
        board_color: str,
        voltages_by_cell: dict[int, Optional[float]],
        temps_by_sensor: dict[int, Optional[float]],
        overlay_filter: str = "all",
        allboard_ranges: Optional[dict[str, Any]] = None,
        flash_on: bool = False,
    ) -> None:
        """
        All cells/temps mode:
          - Shows every discrete value for the selected board pair.

        Min/Max only mode:
          - Ignores the selected board pair.
          - Uses every board that has reported data.
          - Draws per-location min/max for all 28 cells and all 20 thermistors.
        """
        c = self.canvas
        c.delete("all")
        c.create_image(0, 0, image=self.photo, anchor="nw")

        if overlay_filter == "minmax" and allboard_ranges is not None:
            self.draw_allboard_range_overlay(board_color, allboard_ranges, flash_on)
            return

        filtered_voltages = {
            cell: voltages_by_cell.get(cell)
            for cell in self.cell_coords
            if self.voltage_allowed(cell)
        }
        cleaned_temps = {
            sensor: self.valid_temp_value(value)
            for sensor, value in temps_by_sensor.items()
        }

        vmin_cell, vmin_val, vmax_cell, vmax_val = self.minmax_pairs(filtered_voltages)
        tmin_sensor, tmin_val, tmax_sensor, tmax_val = self.minmax_pairs(cleaned_temps)

        # Voltage badges: purple at 2.5 V -> blue at 4.2 V.
        for cell, point in self.cell_coords.items():
            if not self.voltage_allowed(cell):
                continue

            value = filtered_voltages.get(cell)
            x, y = self.scale_point(point)
            label = f"C{cell}\nNaN" if value is None else f"C{cell}\n{value:.3f}V"

            fill = voltage_color(value)
            text_color = readable_text_color(fill)
            outline = board_color
            outline_width = 2

            if cell == vmin_cell:
                outline = "#FF5C5C"
                outline_width = 4
            if cell == vmax_cell:
                outline = "#58A6FF"
                outline_width = 4
            if cell == vmin_cell and cell == vmax_cell:
                outline = "#FFFFFF"
                outline_width = 4

            if self.is_orange_voltage(value):
                fill = self.flashing_fill(fill, "#FF8C00", flash_on)
                text_color = "black"
                outline = "#FFD23F"
                outline_width = 5
            elif self.is_voltage_fault(value):
                fill = self.flashing_fill(fill, "#FF2B2B", flash_on)
                text_color = "white"
                outline = "#FFFFFF" if flash_on else "#FF2B2B"
                outline_width = 5

            badge_w = 56 if self.display_width < 900 else 66
            badge_h = 34
            c.create_rectangle(
                x - badge_w / 2,
                y - badge_h / 2,
                x + badge_w / 2,
                y + badge_h / 2,
                fill=fill,
                outline=outline,
                width=outline_width,
            )

        # Temperature circles: green low -> red at 60 C+.
        for sensor, point in self.temp_coords.items():
            value = cleaned_temps.get(sensor)
            x, y = self.scale_point(point)
            fill = temp_color(value)
            r = 17 if self.display_width < 900 else 20

            outline = board_color
            outline_width = 4
            if sensor == tmin_sensor:
                outline = "#58A6FF"
                outline_width = 5
            if sensor == tmax_sensor:
                outline = "#FF5C5C"
                outline_width = 5
            if sensor == tmin_sensor and sensor == tmax_sensor:
                outline = "#FFFFFF"
                outline_width = 5

            if self.is_temp_fault(value):
                fill = self.flashing_fill(fill, "#FF2B2B", flash_on)
                outline = "#FFFFFF" if flash_on else "#FF2B2B"
                outline_width = 6

            c.create_oval(x - r, y - r, x + r, y + r, fill=fill, outline=outline, width=outline_width)
            label = f"U{sensor}\nNaN" if value is None else f"U{sensor}\n{value:.1f}C"
            c.create_text(
                x,
                y,
                text=label,
                fill="white" if fill != "#333333" else "#D0D0D0",
                font=("Segoe UI", 8, "bold"),
                justify="center",
            )

        c.create_rectangle(4, 4, 292, 30, fill="#101214", outline=board_color, width=2)
        c.create_text(
            14,
            17,
            anchor="w",
            text=f"Selected boards {board_label}   map color {board_color}",
            fill="#F2F2F2",
            font=("Segoe UI", 9, "bold"),
        )

        def fmt_cell(cell: Optional[int], value: Optional[float]) -> str:
            return "NaN" if cell is None or value is None else f"C{cell} {value:.3f}V"

        def fmt_temp(sensor: Optional[int], value: Optional[float]) -> str:
            return "NaN" if sensor is None or value is None else f"U{sensor} {value:.1f}C"

        summary = (
            f"Visible Vmin: {fmt_cell(vmin_cell, vmin_val)}    Visible Vmax: {fmt_cell(vmax_cell, vmax_val)}\n"
            f"Visible Tmin: {fmt_temp(tmin_sensor, tmin_val)}    Visible Tmax: {fmt_temp(tmax_sensor, tmax_val)}"
        )
        c.create_rectangle(4, 34, 490, 78, fill="#101214", outline="#445", width=1)
        c.create_text(12, 56, anchor="w", text=summary, fill="#F2F2F2", font=("Segoe UI", 8, "bold"))


class BMSGuiApp:
    def __init__(self, root: tk.Tk, state: MonitorState, state_lock: threading.RLock, stop_event: threading.Event, left_image: Path, right_image: Path) -> None:
        self.root = root
        self.state = state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.status_queue: "queue.Queue[str]" = queue.Queue()
        self.selected_pair = tk.IntVar(value=1)
        self.overlay_filter = tk.StringVar(value="minmax")
        self.vars: dict[str, tk.StringVar] = {}

        root.title("EV4 BMS Serial GUI")
        root.geometry("1600x980")
        root.minsize(1200, 760)
        root.configure(bg="#0D1117")

        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except Exception:
            pass
        self.style.configure(".", font=("Segoe UI", 9))
        self.style.configure("TFrame", background="#0D1117")
        self.style.configure("TLabelframe", background="#0D1117", foreground="#F2F2F2")
        self.style.configure("TLabelframe.Label", background="#0D1117", foreground="#F2F2F2", font=("Segoe UI", 10, "bold"))
        self.style.configure("TLabel", background="#0D1117", foreground="#F2F2F2")
        self.style.configure("TButton", padding=5)
        self.style.configure("Pair.TButton", padding=4)

        self.build_ui(left_image, right_image)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.schedule_update()

    def make_var(self, name: str, default: str = "NaN") -> tk.StringVar:
        v = tk.StringVar(value=default)
        self.vars[name] = v
        return v

    def build_ui(self, left_image: Path, right_image: Path) -> None:
        main = ttk.Frame(self.root)
        main.pack(fill="both", expand=True, padx=8, pady=8)

        # Three-column layout:
        #   left   = status/dials/BMS data
        #   center = large stacked module image overlays
        #   right  = narrow Active Alerts / Recent Events
        main.columnconfigure(0, weight=0, minsize=390)
        main.columnconfigure(1, weight=1)
        main.columnconfigure(2, weight=0, minsize=300)
        main.rowconfigure(0, weight=1)

        left_panel = ttk.Frame(main)
        left_panel.grid(row=0, column=0, sticky="nswe", padx=(0, 8))

        map_panel = ttk.Frame(main)
        map_panel.grid(row=0, column=1, sticky="nswe", padx=(0, 8))
        map_panel.columnconfigure(0, weight=1)
        map_panel.rowconfigure(2, weight=1)

        event_panel = ttk.Frame(main)
        event_panel.grid(row=0, column=2, sticky="nswe")
        event_panel.columnconfigure(0, weight=1)
        event_panel.rowconfigure(0, weight=1)
        event_panel.rowconfigure(1, weight=1)

        self.build_status_panel(left_panel)
        self.voltage_dial = Dial(left_panel, "Cell Voltage Min/Max", 2.5, 4.2, units="V")
        self.voltage_dial.pack(fill="x", pady=(8, 0))

        self.temp_dial = Dial(left_panel, "Cell Temperature Min/Max", -20, 80, redline_start=60, units="C")
        self.temp_dial.pack(fill="x", pady=(8, 0))

        self.build_info_panel(left_panel, "BMS Values", [
            ("Pack Voltage", "pack_voltage_v"),
            ("Current", "current_a"),
            ("SOC", "soc_percent"),
            ("Power Limit", "power_limit_kw"),
            ("Max Cell Voltage", "max_cell_voltage_v"),
            ("Min Cell Voltage", "min_cell_voltage_v"),
            ("Max Cell Temp", "max_cell_temp_c"),
            ("Min Cell Temp", "min_cell_temp_c"),
            ("Max Die Temp", "max_die_temp_c"),
            ("Min Die Temp", "min_die_temp_c"),
            ("Hall ADC Voltage", "hall_adc_voltage_v"),
            ("Current Zero Voltage", "current_zero_voltage"),
        ])

        self.build_charger_panel(left_panel)
        self.build_can_panel(left_panel)

        top = ttk.Frame(map_panel)
        top.grid(row=0, column=0, sticky="we")

        controls = ttk.Frame(top)
        controls.grid(row=0, column=0, sticky="w")

        ttk.Label(controls, text="Board Pair:", font=("Segoe UI", 10, "bold")).pack(side="left", padx=(0, 6))
        for pair in range(1, 6):
            b1 = pair * 2 - 1
            b2 = pair * 2
            btn = ttk.Radiobutton(
                controls,
                text=f"{b1}/{b2}",
                variable=self.selected_pair,
                value=pair,
                command=self.force_redraw,
            )
            btn.pack(side="left", padx=(0, 8))

        ttk.Label(controls, text="Overlay:", font=("Segoe UI", 10, "bold")).pack(side="left", padx=(18, 6))
        ttk.Radiobutton(
            controls,
            text="All cells/temps",
            variable=self.overlay_filter,
            value="all",
            command=self.force_redraw,
        ).pack(side="left", padx=(0, 8))
        ttk.Radiobutton(
            controls,
            text="Min/Max only",
            variable=self.overlay_filter,
            value="minmax",
            command=self.force_redraw,
        ).pack(side="left", padx=(0, 8))

        self.legend = tk.Canvas(map_panel, height=54, bg="#0D1117", highlightthickness=0)
        self.legend.grid(row=1, column=0, sticky="we", pady=(4, 4))

        maps = ttk.Frame(map_panel)
        maps.grid(row=2, column=0, sticky="nwe")
        maps.columnconfigure(0, weight=1)

        # Larger overlays: the images are stacked vertically instead of side-by-side.
        # This gives the module maps most of the screen width while keeping the
        # dashboard and events visible.
        self.left_map = ModuleMap(
            maps,
            "Left Side: EVEN Cell Voltages / Odd U1-U19",
            left_image,
            LEFT_CELL_COORDS,
            LEFT_TEMP_COORDS,
            width=1180,
            voltage_parity="even",
        )
        self.left_map.grid(row=0, column=0, sticky="n", pady=(0, 8))

        self.right_map = ModuleMap(
            maps,
            "Right Side: ODD Cell Voltages / Even U2-U20",
            right_image,
            RIGHT_CELL_COORDS,
            RIGHT_TEMP_COORDS,
            width=1180,
            voltage_parity="odd",
        )
        self.right_map.grid(row=1, column=0, sticky="n")

        self.alert_text = self.make_text_box(event_panel, "ACTIVE ALERTS", 0, height=17, width=36)
        self.event_text = self.make_text_box(event_panel, "RECENT EVENTS", 1, height=17, width=36)

    def make_text_box(
        self,
        parent: tk.Widget,
        title: str,
        row: int,
        height: int = 10,
        width: int = 42,
    ) -> tk.Text:
        frame = ttk.LabelFrame(parent, text=title)
        frame.grid(row=row, column=0, sticky="nswe", pady=(0, 6) if row == 0 else (6, 0))
        frame.rowconfigure(0, weight=1)
        frame.columnconfigure(0, weight=1)

        text = tk.Text(
            frame,
            height=height,
            width=width,
            bg="#101214",
            fg="#F2F2F2",
            insertbackground="#F2F2F2",
            relief="flat",
            wrap="word",
        )
        text.grid(row=0, column=0, sticky="nswe", padx=4, pady=4)
        text.configure(state="disabled")
        return text

    def build_status_panel(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="Status")
        frame.pack(fill="x")
        fields = [
            ("Serial", "serial_status"),
            ("Cell V", "cell_voltage_status"),
            ("Mode", "mode"),
            ("Runtime", "runtime"),
            ("Lines", "lines"),
            ("Rate", "rate"),
            ("Log", "log"),
        ]

        for r, (label, key) in enumerate(fields):
            ttk.Label(frame, text=label + ":", width=10).grid(row=r, column=0, sticky="w", padx=4, pady=2)

            if key == "serial_status":
                serial_frame = ttk.Frame(frame)
                serial_frame.grid(row=r, column=1, sticky="w", padx=4, pady=2)

                self.serial_indicator = tk.Canvas(serial_frame, width=14, height=14, bg=GUI_BG_NORMAL, highlightthickness=0)
                self.serial_indicator.pack(side="left", padx=(0, 5))
                self.serial_indicator_id = self.serial_indicator.create_oval(
                    2, 2, 12, 12,
                    fill="#E53935",
                    outline="#F2F2F2",
                    width=1,
                )

                ttk.Label(serial_frame, textvariable=self.make_var(key), wraplength=245).pack(side="left")

            elif key == "cell_voltage_status":
                voltage_frame = ttk.Frame(frame)
                voltage_frame.grid(row=r, column=1, sticky="w", padx=4, pady=2)

                self.cell_voltage_indicator = tk.Canvas(voltage_frame, width=14, height=14, bg=GUI_BG_NORMAL, highlightthickness=0)
                self.cell_voltage_indicator.pack(side="left", padx=(0, 5))
                self.cell_voltage_indicator_id = self.cell_voltage_indicator.create_oval(
                    2, 2, 12, 12,
                    fill="#FFD23F",  # idle yellow
                    outline="#F2F2F2",
                    width=1,
                )

                ttk.Label(voltage_frame, textvariable=self.make_var(key), wraplength=245).pack(side="left")

            else:
                ttk.Label(frame, textvariable=self.make_var(key), wraplength=270).grid(row=r, column=1, sticky="w", padx=4, pady=2)

    def build_info_panel(self, parent: tk.Widget, title: str, fields: list[tuple[str, str]]) -> None:
        frame = ttk.LabelFrame(parent, text=title)
        frame.pack(fill="x", pady=(8, 0))
        for r, (label, key) in enumerate(fields):
            ttk.Label(frame, text=label + ":", width=19).grid(row=r, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(frame, textvariable=self.make_var(key), width=16).grid(row=r, column=1, sticky="e", padx=4, pady=2)

    def build_charger_panel(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="Charger")
        frame.pack(fill="x", pady=(8, 0))
        fields = [
            ("Command", "charger_cmd_text"),
            ("Request Voltage", "charger_cmd_voltage"),
            ("Request Current", "charger_cmd_current"),
            ("Control Byte", "charger_cmd_control"),
            ("Status Voltage", "charger_status_voltage"),
            ("Status Current", "charger_status_current"),
            ("Status Byte", "charger_status_byte"),
            ("Faults", "charger_status_faults"),
        ]
        for r, (label, key) in enumerate(fields):
            ttk.Label(frame, text=label + ":", width=17).grid(row=r, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(frame, textvariable=self.make_var(key), wraplength=230).grid(row=r, column=1, sticky="w", padx=4, pady=2)

    def build_can_panel(self, parent: tk.Widget) -> None:
        frame = ttk.LabelFrame(parent, text="CAN / Serial")
        frame.pack(fill="x", pady=(8, 0))
        fields = [
            ("TX Frames", "tx_count"),
            ("Last TX", "last_tx"),
            ("RX Frames", "rx_count"),
            ("Last RX", "last_rx"),
            ("ID Counts", "id_counts"),
        ]
        for r, (label, key) in enumerate(fields):
            ttk.Label(frame, text=label + ":", width=11).grid(row=r, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(frame, textvariable=self.make_var(key), wraplength=260).grid(row=r, column=1, sticky="w", padx=4, pady=2)

    def update_serial_indicator(self, serial_status: str, last_line_time: Optional[float]) -> None:
        """
        Green means the GUI is actively receiving and saving serial lines.

        Opening the COM port alone is not enough. If the port is open but no
        serial lines have been received recently, the indicator stays red.
        """
        now = time.time()
        port_open = serial_status.lower().startswith("connected")
        data_recent = (
            last_line_time is not None
            and (now - last_line_time) <= SERIAL_DATA_ALIVE_TIMEOUT_S
        )

        active_stream = port_open and data_recent
        color = "#2ECC71" if active_stream else "#E53935"

        if hasattr(self, "serial_indicator"):
            self.serial_indicator.configure(bg=getattr(self, "current_bg", GUI_BG_NORMAL))
            self.serial_indicator.itemconfig(self.serial_indicator_id, fill=color)


    def update_cell_voltage_indicator(
        self,
        last_voltage_time: Optional[float],
        avg_refresh_ms: Optional[float],
    ) -> None:
        """
        Flash green briefly when a board cell-voltage row is received.
        Idle color is yellow.

        The text shows the rolling average refresh time in milliseconds.
        """
        now = time.time()
        just_received = (
            last_voltage_time is not None
            and (now - last_voltage_time) <= CELL_VOLTAGE_ACTIVITY_FLASH_S
        )

        color = "#2ECC71" if just_received else "#FFD23F"

        if avg_refresh_ms is None:
            text = "avg -- ms"
        else:
            text = f"avg {avg_refresh_ms:.1f} ms"

        self.vars["cell_voltage_status"].set(text)

        if hasattr(self, "cell_voltage_indicator"):
            self.cell_voltage_indicator.configure(bg=getattr(self, "current_bg", GUI_BG_NORMAL))
            self.cell_voltage_indicator.itemconfig(self.cell_voltage_indicator_id, fill=color)

    def apply_alarm_background(self, allboard_ranges: dict[str, Any]) -> None:
        # Charger faults are intentionally ignored here; this is driven only by
        # board voltage/temp arrays.
        if allboard_ranges.get("has_1p5v"):
            bg = GUI_BG_ORANGE
        elif allboard_ranges.get("has_voltage_fault") or allboard_ranges.get("has_temp_fault"):
            bg = GUI_BG_FAULT
        else:
            bg = GUI_BG_NORMAL

        if getattr(self, "current_bg", None) == bg:
            return

        self.current_bg = bg
        self.root.configure(bg=bg)
        self.style.configure("TFrame", background=bg)
        self.style.configure("TLabelframe", background=bg, foreground="#F2F2F2")
        self.style.configure("TLabelframe.Label", background=bg, foreground="#F2F2F2", font=("Segoe UI", 10, "bold"))
        self.style.configure("TLabel", background=bg, foreground="#F2F2F2")

        if hasattr(self, "serial_indicator"):
            self.serial_indicator.configure(bg=bg)

        if hasattr(self, "cell_voltage_indicator"):
            self.cell_voltage_indicator.configure(bg=bg)

    def force_redraw(self) -> None:
        self.update_gui()

    def set_text_box(self, widget: tk.Text, lines: list[str]) -> None:
        widget.configure(state="normal")
        widget.delete("1.0", "end")
        widget.insert("1.0", "\n".join(lines))
        widget.configure(state="disabled")

    def get_board_voltage_cells(self, state: MonitorState, board: int, start_cell: int) -> dict[int, Optional[float]]:
        values = state.board_voltages.get(board, [])
        out: dict[int, Optional[float]] = {}
        for i in range(14):
            cell = start_cell + i
            out[cell] = values[i] if i < len(values) else None
        return out

    def get_pair_voltage_cells(self, state: MonitorState, odd_board: int, even_board: int) -> dict[int, Optional[float]]:
        """
        Combine one odd/even board pair into physical cell numbers 1-28.

        Odd boards provide Cell 1-14.
        Even boards provide Cell 15-28.
        The ModuleMap then filters those cells by physical side:
          left image  = even cell numbers only
          right image = odd cell numbers only
        """
        out: dict[int, Optional[float]] = {}

        odd_values = state.board_voltages.get(odd_board, [])
        for i in range(14):
            cell = i + 1
            out[cell] = odd_values[i] if i < len(odd_values) else None

        even_values = state.board_voltages.get(even_board, [])
        for i in range(14):
            cell = i + 15
            out[cell] = even_values[i] if i < len(even_values) else None

        return out

    @staticmethod
    def is_valid_thermistor_temp(value: Any) -> bool:
        try:
            temp = float(value)
        except (TypeError, ValueError):
            return False
        # -55 C means disconnected thermistor in this BMS output.
        return temp > -50.0

    @staticmethod
    def range_entry(items: list[dict[str, Any]]) -> dict[str, Optional[dict[str, Any]]]:
        if not items:
            return {"min": None, "max": None}
        return {
            "min": min(items, key=lambda item: item["value"]),
            "max": max(items, key=lambda item: item["value"]),
        }

    def get_allboard_ranges(self, state: MonitorState) -> dict[str, Any]:
        """
        Compute min/max from ALL reported boards, grouped by physical location.

        Voltage mapping:
          odd boards  -> physical cells C1-C14
          even boards -> physical cells C15-C28

        Temperature mapping:
          odd boards  -> odd physical U labels:  U1, U3, ... U19
          even boards -> even physical U labels: U2, U4, ... U20

        Disconnected thermistors reported as -55 C are ignored.

        Extra flags:
          has_voltage_fault = any non-charger cell voltage <2.5 V or >4.2 V
          has_temp_fault    = any non-charger thermistor >=60 C
          has_1p5v          = any board cell voltage near 1.5 V
        """
        voltage_by_cell: dict[int, list[dict[str, Any]]] = {cell: [] for cell in range(1, 29)}
        has_voltage_fault = False
        has_1p5v = False

        for board, values in state.board_voltages.items():
            start_cell = 1 if board % 2 == 1 else 15
            for i, raw_value in enumerate(values[:14]):
                try:
                    value = float(raw_value)
                except (TypeError, ValueError):
                    continue

                cell = start_cell + i
                item = {
                    "board": board,
                    "cell": cell,
                    "value": value,
                }
                voltage_by_cell.setdefault(cell, []).append(item)

                if CELL_VOLTAGE_ORANGE_SENTINEL_LOW <= value <= CELL_VOLTAGE_ORANGE_SENTINEL_HIGH:
                    has_1p5v = True
                elif value < CELL_VOLTAGE_MIN_FAULT or value > CELL_VOLTAGE_MAX_FAULT:
                    has_voltage_fault = True

        temp_by_sensor: dict[int, list[dict[str, Any]]] = {sensor: [] for sensor in range(1, 21)}
        has_temp_fault = False

        for board, values in state.board_temps.items():
            for i, raw_value in enumerate(values[:10]):
                if not self.is_valid_thermistor_temp(raw_value):
                    continue

                value = float(raw_value)
                # Odd-numbered boards map to odd physical U labels on the left image.
                # Even-numbered boards map to even physical U labels on the right image.
                sensor = (1 + 2 * i) if board % 2 == 1 else (2 + 2 * i)
                item = {
                    "board": board,
                    "sensor": sensor,
                    "value": value,
                }
                temp_by_sensor.setdefault(sensor, []).append(item)

                if value >= CELL_TEMP_MAX_FAULT_C:
                    has_temp_fault = True

        voltage_ranges = {
            cell: self.range_entry(items)
            for cell, items in voltage_by_cell.items()
        }
        temp_ranges = {
            sensor: self.range_entry(items)
            for sensor, items in temp_by_sensor.items()
        }

        all_voltage_items = [item for items in voltage_by_cell.values() for item in items]
        all_temp_items = [item for items in temp_by_sensor.values() for item in items]

        return {
            "voltage_ranges": voltage_ranges,
            "temp_ranges": temp_ranges,
            "vmin": min(all_voltage_items, key=lambda item: item["value"]) if all_voltage_items else None,
            "vmax": max(all_voltage_items, key=lambda item: item["value"]) if all_voltage_items else None,
            "tmin": min(all_temp_items, key=lambda item: item["value"]) if all_temp_items else None,
            "tmax": max(all_temp_items, key=lambda item: item["value"]) if all_temp_items else None,
            "has_voltage_fault": has_voltage_fault,
            "has_temp_fault": has_temp_fault,
            "has_1p5v": has_1p5v,
        }


    def get_board_temp_sensors(self, state: MonitorState, board: int, start_sensor: int = 0) -> dict[int, Optional[float]]:
        values = state.board_temps.get(board, [])
        out: dict[int, Optional[float]] = {}

        for i in range(10):
            # Odd-numbered boards map to odd physical U labels on the left image.
            # Even-numbered boards map to even physical U labels on the right image.
            sensor = (1 + 2 * i) if board % 2 == 1 else (2 + 2 * i)

            if i < len(values) and self.is_valid_thermistor_temp(values[i]):
                out[sensor] = values[i]
            else:
                out[sensor] = None

        return out

    def draw_legend(self) -> None:
        c = self.legend
        c.delete("all")

        x = 8
        y1 = 13
        y2 = 38

        for pair in range(1, 6):
            color = PAIR_COLORS[pair]
            b1 = pair * 2 - 1
            b2 = pair * 2
            c.create_rectangle(x, y1 - 8, x + 18, y1 + 10, fill=color, outline="#FFFFFF", width=1)
            c.create_text(
                x + 28,
                y1 + 1,
                anchor="w",
                text=f"Boards {b1}/{b2}",
                fill="#F2F2F2",
                font=("Segoe UI", 9, "bold"),
            )
            x += 150

        # Two-line legend so the voltage text does not get clipped on narrower windows.
        legend_x = x + 10
        c.create_oval(legend_x, y1 - 9, legend_x + 18, y1 + 9, fill=temp_color(25), outline="#F2F2F2", width=2)
        c.create_text(
            legend_x + 28,
            y1,
            anchor="w",
            text="Temp: green low → red at 60C+    -55C ignored",
            fill="#F2F2F2",
            font=("Segoe UI", 9, "bold"),
        )
        c.create_text(
            legend_x + 28,
            y2,
            anchor="w",
            text="Voltage: purple 2.5V → blue 4.2V    Min/Max = all boards / every location",
            fill="#F2F2F2",
            font=("Segoe UI", 9, "bold"),
        )

    def update_gui(self) -> None:
        with self.state_lock:
            update_derived_active_alerts(self.state)
            # Snapshot the state references/values needed for drawing.
            state = self.state

            now = time.time()
            runtime = now - state.start_time
            self.vars["serial_status"].set(state.serial_status)
            self.vars["mode"].set(state.mode)
            self.vars["runtime"].set(f"{runtime:.1f} s")
            self.vars["lines"].set(str(state.line_count))
            self.vars["rate"].set(f"{state.raw_line_rate_hz:.1f} lines/s")
            self.vars["log"].set(str(state.log_path) if state.log_path else "NaN")

            for key in [
                "pack_voltage_v",
                "current_a",
                "soc_percent",
                "power_limit_kw",
                "max_cell_voltage_v",
                "min_cell_voltage_v",
                "max_cell_temp_c",
                "min_cell_temp_c",
                "max_die_temp_c",
                "min_die_temp_c",
                "hall_adc_voltage_v",
                "current_zero_voltage",
            ]:
                self.vars[key].set(value_text(state, key))

            cmd = state.charger_cmd
            if cmd:
                self.vars["charger_cmd_text"].set(cmd["text"])
                self.vars["charger_cmd_voltage"].set(f"{cmd['voltage_v']:.1f} V")
                self.vars["charger_cmd_current"].set(f"{cmd['current_a']:.1f} A")
                self.vars["charger_cmd_control"].set(f"0x{cmd['control']:02X}")
            else:
                self.vars["charger_cmd_text"].set("NaN")
                self.vars["charger_cmd_voltage"].set("NaN")
                self.vars["charger_cmd_current"].set("NaN")
                self.vars["charger_cmd_control"].set("NaN")

            st = state.charger_status
            if st:
                self.vars["charger_status_voltage"].set(f"{st['voltage_v']:.2f} V")
                self.vars["charger_status_current"].set(f"{st['current_a']:.2f} A")
                self.vars["charger_status_byte"].set(f"0x{st['status']:02X}")
                self.vars["charger_status_faults"].set("; ".join(st["faults"]) if st["faults"] else "none")
            else:
                self.vars["charger_status_voltage"].set("NaN")
                self.vars["charger_status_current"].set("NaN")
                self.vars["charger_status_byte"].set("NaN")
                self.vars["charger_status_faults"].set("NaN")

            self.vars["tx_count"].set(str(state.tx_count))
            self.vars["rx_count"].set(str(state.rx_count))
            self.vars["last_tx"].set(age_text(state.last_tx_time, now))
            self.vars["last_rx"].set(age_text(state.last_rx_time, now))
            count_items = [f"{k}:{v}" for k, v in sorted(state.id_counts.items())[:8]]
            self.vars["id_counts"].set("   ".join(count_items) if count_items else "NaN")

            allboard_ranges = self.get_allboard_ranges(state)
            serial_status_for_indicator = state.serial_status
            last_line_time_for_indicator = state.last_serial_line_time
            last_voltage_time_for_indicator = state.last_cell_voltage_update_time
            if state.cell_voltage_refresh_intervals_ms:
                avg_voltage_refresh_ms = (
                    sum(state.cell_voltage_refresh_intervals_ms)
                    / len(state.cell_voltage_refresh_intervals_ms)
                )
            else:
                avg_voltage_refresh_ms = None
            min_v = allboard_ranges["vmin"]["value"] if allboard_ranges["vmin"] else get_latest_float(state, "min_cell_voltage_v")
            max_v = allboard_ranges["vmax"]["value"] if allboard_ranges["vmax"] else get_latest_float(state, "max_cell_voltage_v")
            min_t = allboard_ranges["tmin"]["value"] if allboard_ranges["tmin"] else get_latest_float(state, "min_cell_temp_c")
            max_t = allboard_ranges["tmax"]["value"] if allboard_ranges["tmax"] else get_latest_float(state, "max_cell_temp_c")
            alerts = [text for _key, text in sorted(state.active_alerts.items())][:ACTIVE_ALERT_LINES]
            while len(alerts) < ACTIVE_ALERT_LINES:
                alerts.append("")
            events = list(state.recent_events)[-RECENT_EVENT_LINES:]
            while len(events) < RECENT_EVENT_LINES:
                events.insert(0, "")

            pair = self.selected_pair.get()
            odd_board = pair * 2 - 1
            even_board = pair * 2
            board_color = PAIR_COLORS.get(pair, "#00AEEF")
            pair_label = f"{odd_board}/{even_board}"
            pair_voltages = self.get_pair_voltage_cells(state, odd_board, even_board)
            left_voltages = pair_voltages
            right_voltages = pair_voltages
            left_temps = self.get_board_temp_sensors(state, odd_board)
            right_temps = self.get_board_temp_sensors(state, even_board)
            overlay_filter = self.overlay_filter.get()

        flash_on = (int(time.time() / FLASH_PERIOD_S) % 2) == 0
        self.apply_alarm_background(allboard_ranges)
        self.update_serial_indicator(serial_status_for_indicator, last_line_time_for_indicator)
        self.update_cell_voltage_indicator(last_voltage_time_for_indicator, avg_voltage_refresh_ms)

        self.voltage_dial.draw(min_v, max_v)
        self.temp_dial.draw(min_t, max_t)
        self.draw_legend()
        self.left_map.draw(pair_label, board_color, left_voltages, left_temps, overlay_filter, allboard_ranges, flash_on)
        self.right_map.draw(pair_label, board_color, right_voltages, right_temps, overlay_filter, allboard_ranges, flash_on)

        self.set_text_box(self.alert_text, [f"{i:02d}: {a}" for i, a in enumerate(alerts, start=1)])
        self.set_text_box(self.event_text, [f"{i:02d}: {e}" for i, e in enumerate(events, start=1)])

    def schedule_update(self) -> None:
        try:
            while True:
                msg = self.status_queue.get_nowait()
                print(msg)
        except queue.Empty:
            pass
        self.update_gui()
        if not self.stop_event.is_set():
            self.root.after(250, self.schedule_update)

    def on_close(self) -> None:
        self.stop_event.set()
        self.root.after(150, self.root.destroy)


def resolve_image_path(path_arg: Optional[str], default_name: str) -> Path:
    if path_arg:
        p = Path(path_arg)
        if p.exists():
            return p
    candidates = [
        Path(__file__).with_name(default_name),
        Path.cwd() / default_name,
        Path(__file__).parent.parent / default_name,
    ]
    for p in candidates:
        if p.exists():
            return p
    raise FileNotFoundError(
        f"Could not find {default_name}. Put it next to bms_serial_gui.py or pass --{default_name.lower().replace('_module.png','').replace('_side','')}-image."
    )


def main() -> int:
    parser = argparse.ArgumentParser(description="EV4 BMS Serial GUI with CSV logger and module image overlays")
    parser.add_argument("--port", help="Serial port, example COM7, COM12, /dev/ttyACM0")
    parser.add_argument("--auto", action="store_true", help="Auto-select a likely Teensy/USB serial port")
    parser.add_argument("--list", action="store_true", help="List serial ports and exit")
    parser.add_argument("--baud", type=int, default=9600, help="Serial baud rate. Current BMS code uses Serial.begin(9600).")
    parser.add_argument("--log-dir", default="BMS_Serial_Laptop_Recieved", help="Folder for timestamped CSV logs")
    parser.add_argument("--left-image", help="Path to Left_Side_module.png")
    parser.add_argument("--right-image", help="Path to Right_Side_Module.png")
    parser.add_argument("--raw", action="store_true", help="Also echo every raw serial line to the console")
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
        print("  python bms_serial_gui.py --port COM7 --baud 9600")
        print("  python bms_serial_gui.py --auto --baud 9600")
        return 2

    try:
        left_image = resolve_image_path(args.left_image, "Left_Side_module.png")
        right_image = resolve_image_path(args.right_image, "Right_Side_Module.png")
    except FileNotFoundError as exc:
        print(exc)
        return 2

    state = MonitorState()
    lock = threading.RLock()
    stop_event = threading.Event()
    status_queue: "queue.Queue[str]" = queue.Queue()

    worker = SerialWorker(
        state=state,
        state_lock=lock,
        stop_event=stop_event,
        port=port,
        baud=args.baud,
        log_dir=Path(args.log_dir),
        status_queue=status_queue,
        raw_echo=args.raw,
    )
    worker.start()

    root = tk.Tk()
    app = BMSGuiApp(root, state, lock, stop_event, left_image, right_image)
    app.status_queue = status_queue
    root.mainloop()

    stop_event.set()
    worker.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
