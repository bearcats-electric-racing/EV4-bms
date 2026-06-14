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
  python tools\bms_serial_gui.py --auto --baud 115200
  python bms_serial_gui.py --port COM7 --baud 115200
"""

from __future__ import annotations

import argparse
import csv
import gc
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
# Firmware has used both "Car CAN Message" and "BMS_Car CAN Message"
# prefixes. Accept both so the GUI TX frame count increments for the
# actual transmitted BMS/car CAN payload line instead of leaving it as raw.
CHARGER_CAN_MESSAGE_RE = re.compile(r"^(?:BMS_)?Charger CAN Message:\s*([0-9A-Fa-f ]+)")
CAR_CAN_MESSAGE_RE = re.compile(r"^(?:BMS_)?Car CAN Message:\s*([0-9A-Fa-f ]+)")
SERIAL_COMMAND_RE = re.compile(r"^Serial command received:\s*(.+?)\s*$", re.IGNORECASE)
FILE_SIZE_RE = re.compile(r"^(.+?)\s+(\d+)\s+bytes\s*$", re.IGNORECASE)
DISCHARGE_CELL_RE = re.compile(
    r"^Board:\s*(\d+)\s+Cell:\s*(\d+)\s+Volt:\s*([-+]?\d+(?:\.\d+)?)\s*$",
    re.IGNORECASE,
)
LABEL_VALUE_UNIT_RE = re.compile(r"^([A-Za-z][A-Za-z0-9 _/().%-]*?):\s*([-+]?\d+(?:\.\d+)?)\s*([A-Za-z%]+)?\s*$")
PCB_TEMP_RE = re.compile(
    r"^(VI|HV)\s+PCB\s+Temp:\s*"
    r"(?:(INVALID)|([-+]?\d+(?:\.\d+)?)\s*C"
    r"(?:,\s*([-+]?\d+(?:\.\d+)?)\s*Hz)?"
    r"(?:,\s*([-+]?\d+(?:\.\d+)?)\s*ohm)?)\s*$",
    re.IGNORECASE,
)

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
    "vi pcb temp": ("vi_pcb_temp_c", "C"),
    "hv pcb temp": ("hv_sense_temp_c", "C"),
    "hv sense temp": ("hv_sense_temp_c", "C"),
    "soc": ("soc_percent", "%"),
    "power limit": ("power_limit_kw", "kW"),
    "memory usage": ("sd_memory_usage_percent", "%"),
    "adc_a": ("adc_a_raw", "raw"),
    "adc_b": ("adc_b_raw", "raw"),
    "a voltage": ("adc_a_voltage_v", "V"),
    "b voltage": ("adc_b_voltage_v", "V"),
    "hall effect adc voltage": ("hall_adc_voltage_v", "V"),
    "raw current": ("raw_current_a", "A"),
    "current offset": ("current_offset_a", "A"),
    "current offset calibration voltage": ("current_offset_calibration_voltage_v", "V"),
    "current offset raw current": ("current_offset_raw_current_a", "A"),
    "current offset calibrated": ("current_offset_a", "A"),
    "current zero voltage": ("current_zero_voltage", "V"),
    "current zero voltage calibrated": ("current_zero_voltage_calibrated", "V"),
    "current zero calibration failed. adc voltage": ("current_zero_calibration_failed_adc_voltage_v", "V"),
    "start time": ("bms_start_time_ms", "ms"),
    "temp": ("spi_temp_raw", "raw"),
    "adc_a": ("adc_a_raw", "raw"),
    "adc_b": ("adc_b_raw", "raw"),
    "a voltage": ("adc_a_voltage_v", "V"),
    "b voltage": ("adc_b_voltage_v", "V"),
    "memory usage": ("sd_memory_usage_percent", "%"),
    "min cell voltage": ("min_cell_voltage_v", "V"),
    "max cell voltage": ("max_cell_voltage_v", "V"),
    "die temp": ("die_temp_c", "C"),
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

# Firmware one-line status outputs that are useful events even when they do not
# contain a numeric value.  Routing them here keeps the CSV parser from leaving
# important known .ino outputs as generic raw rows.
KNOWN_EVENT_LINES = {
    "End of Setup Loop",
    "New volt",
    "New Temp",
    "Send CAN",
    "Debug command accepted",
    "Charger Error",
    "Charger Fault",
    "ADC_initialization ERROR",
    "SD card initialization failed!",
    "SD card over 90% full",
    "Initializing state of charge to 100%",
    "state.txt initialized.",
    "ADBMS6830B ADC TImeout Error",
    "Open Voltage Sense Lead!",
    "Open voltage sense lead detected",
    "invalid voltage",
    "Open fusible link detected - max voltage differential exceeded",
    "Charger E-Stop Active",
    "Failed to send charger CAN message",
    "Charger CAN message sent",
    "Failed to send car CAN message",
    "Car CAN message sent",
    "Clearing faults",
    "No Stored Faults",
    "Cells to be discharged",
    "die_temps",
    "voltage flags",
    "Callback called",
    "Wakeup Sleep",
    "done",
    "serial dump done",
}

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

GUI_BG_NORMAL = "#2B2B2B"
GUI_BG_FAULT = "#4A1010"
GUI_BG_ORANGE = "#5A3300"
CELL_VOLTAGE_MIN_FAULT = 2.50
CELL_VOLTAGE_MAX_FAULT = 4.20
CELL_VOLTAGE_ORANGE_SENTINEL_LOW = 1.45
CELL_VOLTAGE_ORANGE_SENTINEL_HIGH = 1.55
CELL_TEMP_MAX_FAULT_C = 60.0
TEMP_COLOR_GREEN_C = 25.0
TEMP_COLOR_RED_C = CELL_TEMP_MAX_FAULT_C
FLASH_PERIOD_S = 0.50
SERIAL_DATA_ALIVE_TIMEOUT_S = 2.0
SERIAL_RECONNECT_DELAY_S = 1.0
# If a COM port opens but no BMS text arrives, it is probably the wrong/stale
# port after a USB unplug/replug.  Close it and continue scanning.
SERIAL_CONNECT_NO_DATA_TIMEOUT_S = 3.0
# If a previously-working connection goes silent, force a close/re-open so
# Windows can re-enumerate the Teensy on its new COM port.
SERIAL_STALE_DATA_RECONNECT_S = 3.0
# Auto-port mode temporarily skips ports that error or produce no BMS data so
# it does not get stuck on an idle Bluetooth/old COM port.
SERIAL_BAD_PORT_COOLDOWN_S = 2.0
SERIAL_NO_DATA_PORT_COOLDOWN_S = 6.0
CELL_VOLTAGE_ACTIVITY_FLASH_S = 0.60
CELL_TEMP_ACTIVITY_FLASH_S = 0.60
CURRENT_ACTIVITY_FLASH_S = 0.60
CELL_VOLTAGE_REFRESH_WINDOW_SAMPLES = 50
CELL_TEMP_REFRESH_WINDOW_SAMPLES = 50
CURRENT_REFRESH_WINDOW_SAMPLES = 50
CURRENT_HISTORY_WINDOW_S = 60.0
CURRENT_HISTORY_MAX_SAMPLES = 10000
CURRENT_DIAL_MIN_A = -36.0
CURRENT_DIAL_MAX_A = 300.0

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
    14: (1850, 455),  # adjusted right
    16: (1625, 625),  # moved down slightly
    18: (1445, 560),
    20: (1245, 535),
    22: (890, 615),
    24: (720, 535),  # adjusted slightly up
    26: (525, 535),
    28: (420, 440),
}

# Right image shows the physical right-side cell overlay.
# Per the requested display rule, it only draws ODD-numbered cell voltages.
RIGHT_CELL_COORDS = {
    1: (1340, 260),
    3: (1165, 350),  # adjusted slightly down
    5: (1000, 270),
    7: (880, 395),  # adjusted slightly left
    9: (610, 250),  # adjusted slightly up/left
    11: (485, 375),  # adjusted further right
    13: (225, 275),  # adjusted slightly down/right
    15: (250, 590),
    17: (400, 475),  # adjusted slightly up
    19: (780, 655),  # adjusted up
    21: (960, 560),  # adjusted slightly up again
    23: (1160, 550),  # adjusted slightly left
    25: (1500, 635),  # adjusted up
    27: (1730, 620),  # adjusted slightly up
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
    11: (1670, 480),  # adjusted slightly up
    13: (1290, 640),
    15: (930, 520),
    17: (660, 645),
    19: (340, 615),
}

RIGHT_TEMP_COORDS = {
    2: (1390, 400),  # adjusted left
    4: (1035, 400),  # adjusted up
    6: (785, 270),  # adjusted slightly up
    8: (650, 400),  # adjusted slightly up
    10: (185, 500),  # adjusted slightly right
    12: (425, 620),  # adjusted slightly right
    14: (610, 575),  # adjusted slightly up
    16: (760, 530),  # adjusted slightly up again
    18: (1305, 550),  # adjusted slightly up/left
    20: (1510, 530),  # adjusted further left/slightly up
}


GRAPH_SERIES = [
    ("pack_voltage_v", "Pack Voltage", "V", "#58A6FF"),
    ("charger_voltage_v", "Charger Voltage", "V", "#7AC943"),
    ("charger_status_voltage_v", "Status Voltage", "V", "#FFD23F"),
    ("max_cell_voltage_v", "Max Cell Voltage", "V", "#FF8C1A"),
    ("min_cell_voltage_v", "Min Cell Voltage", "V", "#A66CFF"),
    ("hall_adc_voltage_v", "Hall ADC Voltage", "V", "#00D1FF"),
    ("current_zero_voltage", "Current Zero Voltage", "V", "#C9D1D9"),
    ("current_a", "Current", "A", "#FFD23F"),
    ("charger_current_a", "Charger Current", "A", "#7AC943"),
    ("charger_status_current_a", "Status Current", "A", "#00AEEF"),
    ("max_cell_temp_c", "Max Cell Temp", "C", "#FF5C5C"),
    ("min_cell_temp_c", "Min Cell Temp", "C", "#58A6FF"),
    ("vi_pcb_temp_c", "VI Temp", "C", "#A66CFF"),
    ("hv_sense_temp_c", "HV Sense Temp", "C", "#FF8C1A"),
    ("soc_percent", "SOC", "%", "#2ECC71"),
    ("power_limit_kw", "Power Limit", "kW", "#F2F2F2"),
]
GRAPH_SERIES_META = {key: {"label": label, "unit": unit, "color": color} for key, label, unit, color in GRAPH_SERIES}
DEFAULT_GRAPH_KEYS = {
    "pack_voltage_v", "current_a", "max_cell_voltage_v", "min_cell_voltage_v",
    "max_cell_temp_c", "min_cell_temp_c", "vi_pcb_temp_c", "hv_sense_temp_c",
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
    log_status: str = "Log file not opened yet"
    last_closed_log_path: Optional[Path] = None
    log_rollover_requested: bool = False
    log_rollover_in_progress: bool = False
    log_rollover_reason: str = ""
    serial_status: str = "disconnected"
    last_serial_line_time: Optional[float] = None
    serial_command_tx_count: int = 0
    last_serial_command_sent_time: Optional[float] = None
    last_serial_command_sent: str = ""
    serial_command_status: str = "No command sent"
    last_cell_voltage_update_time: Optional[float] = None
    cell_voltage_refresh_intervals_ms: deque[float] = field(
        default_factory=lambda: deque(maxlen=CELL_VOLTAGE_REFRESH_WINDOW_SAMPLES)
    )
    last_cell_temp_update_time: Optional[float] = None
    cell_temp_refresh_intervals_ms: deque[float] = field(
        default_factory=lambda: deque(maxlen=CELL_TEMP_REFRESH_WINDOW_SAMPLES)
    )
    last_current_update_time: Optional[float] = None
    current_refresh_intervals_ms: deque[float] = field(
        default_factory=lambda: deque(maxlen=CURRENT_REFRESH_WINDOW_SAMPLES)
    )
    current_samples: deque[tuple[float, float]] = field(
        default_factory=lambda: deque(maxlen=CURRENT_HISTORY_MAX_SAMPLES)
    )
    time_series: deque[tuple[float, str, float, str]] = field(
        default_factory=lambda: deque(maxlen=50000)
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


def pcb_temp_key(label: str) -> tuple[str, str]:
    """Map firmware PCB-temperature labels to GUI row keys and display names."""
    prefix = label.strip().upper()
    if prefix == "VI":
        return "vi_pcb_temp_c", "VI"
    return "hv_sense_temp_c", "HV Sense"


def update_pcb_temp_value(
    state: MonitorState,
    key: str,
    temp_c_text: Optional[str],
    frequency_hz_text: Optional[str],
    ntc_ohm_text: Optional[str],
    invalid: bool,
) -> str:
    """Store a compact one-line PCB temperature value for the BMS Values panel."""
    if invalid:
        state.latest_values[key] = ("INVALID", "")
        return "INVALID"

    if temp_c_text is None:
        state.latest_values[key] = (DASHBOARD_NAN, "")
        return DASHBOARD_NAN

    temp_c = float(temp_c_text)
    display = f"{temp_c:.2f} C"

    # Keep the extra firmware diagnostics available to the CSV/event row while
    # leaving the BMS Values panel as a short one-line temperature readout.
    detail_parts = [display]
    if frequency_hz_text is not None:
        detail_parts.append(f"{float(frequency_hz_text):.1f} Hz")
    if ntc_ohm_text is not None:
        detail_parts.append(f"{float(ntc_ohm_text):.1f} ohm")

    state.latest_values[key] = (display, "")
    return ", ".join(detail_parts)


def get_latest_float(state: MonitorState, key: str) -> Optional[float]:
    if key not in state.latest_values:
        return None
    value, _unit = state.latest_values[key]
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def record_time_series_sample(
    state: MonitorState,
    now: float,
    key: str,
    value: Any,
    unit: str = "",
) -> None:
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        return
    if math.isnan(numeric):
        return

    # Normalize units for graph grouping.  The graph metadata wins when present.
    graph_unit = GRAPH_SERIES_META.get(key, {}).get("unit", unit)
    state.time_series.append((now, key, numeric, graph_unit))

    cutoff = now - 7200.0
    while state.time_series and state.time_series[0][0] < cutoff:
        state.time_series.popleft()


def record_current_sample(state: MonitorState, now: float, value: Any) -> None:
    try:
        current_a = float(value)
    except (TypeError, ValueError):
        return
    if math.isnan(current_a):
        return

    # Used by the Status panel's current activity indicator.
    # The rolling refresh time is measured between parsed calculated-current rows.
    if state.last_current_update_time is not None:
        dt_ms = (now - state.last_current_update_time) * 1000.0
        if 0.0 < dt_ms < 60000.0:
            state.current_refresh_intervals_ms.append(dt_ms)

    state.last_current_update_time = now
    state.current_samples.append((now, current_a))


def recent_current_minmax(
    state: MonitorState,
    now: Optional[float] = None,
    window_s: float = CURRENT_HISTORY_WINDOW_S,
) -> tuple[Optional[float], Optional[float], Optional[float]]:
    if now is None:
        now = time.time()

    cutoff = now - window_s
    while state.current_samples and state.current_samples[0][0] < cutoff:
        state.current_samples.popleft()

    values = [value for _sample_time, value in state.current_samples]
    latest = values[-1] if values else get_latest_float(state, "current_a")
    if not values:
        return latest, latest, latest
    return min(values), max(values), latest


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

    if stripped in KNOWN_EVENT_LINES:
        row["parsed_type"] = "event"
        row["key"] = "event"
        row["value"] = stripped
        important = True

    m = SERIAL_COMMAND_RE.match(stripped)
    if m:
        row.update({
            "parsed_type": "serial_command",
            "key": "serial_command",
            "value": m.group(1).strip(),
        })
        important = True

    m = FILE_SIZE_RE.match(stripped)
    if m and row["parsed_type"] == "raw":
        row.update({
            "parsed_type": "sd_file",
            "key": "sd_file",
            "value": m.group(1).strip(),
            "unit": "bytes",
        })
        try:
            row["array_values"] = str(int(m.group(2)))
        except ValueError:
            row["array_values"] = m.group(2)
        important = True

    m = DISCHARGE_CELL_RE.match(stripped)
    if m:
        row.update({
            "parsed_type": "cell_discharge_candidate",
            "key": "cell_discharge_candidate",
            "board": int(m.group(1)),
            "value": float(m.group(3)),
            "unit": "V",
        })
        row["array_values"] = f"cell {int(m.group(2))}"
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

    m = PCB_TEMP_RE.match(stripped)
    if m:
        label = m.group(1)
        invalid = bool(m.group(2))
        temp_c_text = m.group(3)
        frequency_hz_text = m.group(4)
        ntc_ohm_text = m.group(5)
        key, display_name = pcb_temp_key(label)
        display_value = update_pcb_temp_value(
            state,
            key,
            temp_c_text,
            frequency_hz_text,
            ntc_ohm_text,
            invalid,
        )
        row.update({
            "parsed_type": "pcb_temperature" if row["parsed_type"] == "raw" else row["parsed_type"],
            "key": key,
            "value": display_value,
            "unit": "",
        })
        if not invalid and temp_c_text is not None:
            record_time_series_sample(state, now, key, float(temp_c_text), "C")
        set_active_alert(state, key, f"{display_name} PCB Temp: INVALID", invalid)
        important = True

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
                record_time_series_sample(state, now, "charger_voltage_v", cmd["voltage_v"], "V")
                record_time_series_sample(state, now, "charger_current_a", cmd["current_a"], "A")
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
                record_time_series_sample(state, now, "charger_status_voltage_v", status["voltage_v"], "V")
                record_time_series_sample(state, now, "charger_status_current_a", status["current_a"], "A")
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
                record_time_series_sample(state, now, "charger_status_voltage_v", status["voltage_v"], "V")
                record_time_series_sample(state, now, "charger_status_current_a", status["current_a"], "A")
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
            record_time_series_sample(state, now, "charger_voltage_v", cmd["voltage_v"], "V")
            record_time_series_sample(state, now, "charger_current_a", cmd["current_a"], "A")
            row.update({
                "value": cmd["text"],
                "charger_cmd_voltage_v": f"{cmd['voltage_v']:.1f}",
                "charger_cmd_current_a": f"{cmd['current_a']:.1f}",
                "charger_cmd_control": f"0x{cmd['control']:02X}",
                "charger_cmd_text": cmd["text"],
            })
        important = True

    m = CAR_CAN_MESSAGE_RE.match(stripped)
    if m:
        data = parse_hex_bytes(m.group(1))
        row.update({
            "parsed_type": "car_can_message",
            "can_direction": "TX",
            "can_id": f"0x{BMS_TX_ID}",
            "can_ext": "0",
            "can_len": len(data),
            "can_data_hex": hex_bytes(data),
            "key": "car_can_message",
            "value": f"{len(data)} bytes",
        })
        state.tx_count += 1
        state.last_tx_time = now
        state.id_counts[f"TX 0x{BMS_TX_ID}"] += 1
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

            # Used by the Status panel's cell-temperature activity indicator.
            # The rolling refresh time is measured between received board-temperature rows.
            if state.last_cell_temp_update_time is not None:
                dt_ms = (now - state.last_cell_temp_update_time) * 1000.0
                if 0.0 < dt_ms < 60000.0:
                    state.cell_temp_refresh_intervals_ms.append(dt_ms)

            state.last_cell_temp_update_time = now
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

    m = LABEL_VALUE_UNIT_RE.match(stripped)
    if m:
        label, raw_value = m.group(1), m.group(2)
        raw_unit = m.group(3) or ""
        key, unit = normalize_label(label)
        if not unit and raw_unit:
            unit = raw_unit
        try:
            value: Any = float(raw_value) if "." in raw_value else int(raw_value)
        except ValueError:
            value = raw_value
        state.latest_values[key] = (value, unit)
        if key == "current_a":
            record_current_sample(state, now, value)
        record_time_series_sample(state, now, key, value, unit)
        update_numeric_active_alert(state, key, value)
        row.update({
            "parsed_type": "value" if row["parsed_type"] == "raw" else row["parsed_type"],
            "key": key if not row.get("key") else row["key"],
            "value": row["value"] if row.get("value") not in ("", None) else value,
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
            "vi_pcb_temp_c",
            "hv_sense_temp_c",
            "soc_percent",
            "power_limit_kw",
            "current_zero_voltage",
            "current_zero_voltage_calibrated",
            "current_offset_calibration_voltage_v",
            "current_offset_raw_current_a",
            "current_offset_a",
            "raw_current_a",
            "bms_start_time_ms",
            "sd_memory_usage_percent",
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
        if key == "current_a":
            record_current_sample(state, now, value)
        record_time_series_sample(state, now, key, value, unit)
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


def serial_port_candidates() -> list[str]:
    """
    Return COM ports in best-first order for the BMS.

    The previous auto-select logic could reconnect to an idle/stale COM port
    after a USB unplug/replug because any port with the word "serial" scored
    well enough.  This keeps all usable ports in order, but strongly prefers
    Teensy/USB/Arduino-style ports and pushes Bluetooth-style virtual ports to
    the bottom.
    """
    ports = list(list_ports.comports())
    if not ports:
        return []

    scored: list[tuple[int, str, str]] = []
    for p in ports:
        device = str(p.device)
        desc = f"{p.description} {getattr(p, 'manufacturer', '')} {getattr(p, 'hwid', '')}".lower()
        score = 0

        if "teensy" in desc:
            score += 120
        if "arduino" in desc:
            score += 90
        if "usb serial" in desc or "usb-serial" in desc or "usb-to-serial" in desc:
            score += 80
        if "usb" in desc:
            score += 50
        if "serial" in desc:
            score += 10

        # Windows often exposes Bluetooth SPP ports as COM ports.  They can open
        # successfully but never emit BMS data, which prevents a real reconnect.
        if "bluetooth" in desc or "bth" in desc:
            score -= 200

        scored.append((score, device, p.description))

    scored.sort(key=lambda item: (item[0], item[1]), reverse=True)

    # Prefer non-Bluetooth-ish ports.  If everything was filtered out, fall back
    # to every discovered port so manual/odd adapters still have a chance.
    preferred = [device for score, device, _desc in scored if score > -100]
    return preferred if preferred else [device for _score, device, _desc in scored]


def auto_select_port() -> Optional[str]:
    candidates = serial_port_candidates()
    return candidates[0] if candidates else None

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


def clamp01(value: float) -> float:
    return max(0.0, min(1.0, value))


def rgb_to_hex(rgb: tuple[int, int, int]) -> str:
    r, g, b = rgb
    return f"#{r:02X}{g:02X}{b:02X}"


def hex_to_rgb(hex_color: str) -> tuple[int, int, int]:
    hex_color = hex_color.lstrip("#")
    if len(hex_color) != 6:
        return (255, 255, 255)
    try:
        return (
            int(hex_color[0:2], 16),
            int(hex_color[2:4], 16),
            int(hex_color[4:6], 16),
        )
    except ValueError:
        return (255, 255, 255)


def interpolate_rgb(a: tuple[int, int, int], b: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    t = clamp01(amount)
    return (
        int(round(a[0] + (b[0] - a[0]) * t)),
        int(round(a[1] + (b[1] - a[1]) * t)),
        int(round(a[2] + (b[2] - a[2]) * t)),
    )


def blend_hex(base: str, target: str, amount: float) -> str:
    return rgb_to_hex(interpolate_rgb(hex_to_rgb(base), hex_to_rgb(target), amount))


def temp_color(temp: Optional[float]) -> str:
    if temp is None:
        return "#333333"
    try:
        t_c = float(temp)
    except (TypeError, ValueError):
        return "#333333"
    if math.isnan(t_c) or t_c <= -50.0:
        return "#333333"

    # Continuous RGB temperature scale:
    #   <=25 C = green, midpoint = yellow/orange, >=60 C = red.
    ratio = clamp01((t_c - TEMP_COLOR_GREEN_C) / (TEMP_COLOR_RED_C - TEMP_COLOR_GREEN_C))
    green = (0, 210, 80)
    yellow = (255, 215, 0)
    red = (255, 0, 0)

    if ratio < 0.5:
        return rgb_to_hex(interpolate_rgb(green, yellow, ratio / 0.5))
    return rgb_to_hex(interpolate_rgb(yellow, red, (ratio - 0.5) / 0.5))


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
        port: Optional[str],
        baud: int,
        log_dir: Path,
        status_queue: "queue.Queue[str]",
        command_queue: "queue.Queue[str]",
        raw_echo: bool = False,
        auto_port: bool = False,
    ) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.port = port
        self.baud = baud
        self.log_dir = log_dir
        self.status_queue = status_queue
        self.command_queue = command_queue
        self.raw_echo = raw_echo
        self.auto_port = auto_port
        self.bad_port_until: dict[str, float] = {}

    def _sleep_until_retry_or_stop(self) -> None:
        self.stop_event.wait(SERIAL_RECONNECT_DELAY_S)

    def _mark_auto_port_bad(self, port: str, reason: str, cooldown: float) -> None:
        if not self.auto_port:
            return
        self.bad_port_until[port] = time.time() + cooldown
        self.status_queue.put(f"Skipping {port} for {cooldown:.0f}s after {reason}")

    def _candidate_port(self) -> Optional[str]:
        if not self.auto_port:
            return self.port

        now = time.time()
        for bad_port, until in list(self.bad_port_until.items()):
            if until <= now:
                self.bad_port_until.pop(bad_port, None)

        candidates = serial_port_candidates()
        for candidate in candidates:
            if self.bad_port_until.get(candidate, 0.0) > now:
                continue
            self.port = candidate
            return candidate

        return None

    def _set_serial_status(self, text: str, clear_last_line_time: bool = False) -> None:
        with self.state_lock:
            self.state.serial_status = text
            if clear_last_line_time:
                self.state.last_serial_line_time = None
                self.state.raw_line_rate_hz = 0.0

    def _set_log_status(self, text: str) -> None:
        with self.state_lock:
            self.state.log_status = text

    def _write_pending_commands(self, ser: serial.Serial) -> None:
        """Write GUI-entered serial commands from the worker thread.

        The Tkinter GUI must not write to pyserial directly because the serial
        object is owned by this worker thread. Commands are queued by the GUI and
        drained here while the COM port is open.
        """
        while not self.stop_event.is_set():
            try:
                command = self.command_queue.get_nowait()
            except queue.Empty:
                return

            command = command.rstrip("\r\n")
            if not command.strip():
                continue

            payload = (command + "\n").encode("utf-8")
            try:
                ser.write(payload)
                ser.flush()
            except (serial.SerialException, OSError) as exc:
                with self.state_lock:
                    self.state.serial_command_status = f"Send failed: {exc}"
                self.status_queue.put(f"Serial command send failed: {exc}")
                raise serial.SerialException(exc) from exc

            now = time.time()
            with self.state_lock:
                self.state.serial_command_tx_count += 1
                self.state.last_serial_command_sent_time = now
                self.state.last_serial_command_sent = command
                self.state.serial_command_status = f"Sent: {command}"
                self.state.recent_events.append(f"PC->BMS Serial command: {command}")
            self.status_queue.put(f"Sent serial command to BMS: {command}")

    def _make_log_path(self) -> Path:
        self.log_dir.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        base = self.log_dir / f"BMS_Serial_{timestamp}.csv"
        if not base.exists():
            return base

        for index in range(1, 1000):
            candidate = self.log_dir / f"BMS_Serial_{timestamp}_{index:03d}.csv"
            if not candidate.exists():
                return candidate
        return self.log_dir / f"BMS_Serial_{timestamp}_{int(time.time() * 1000)}.csv"

    def _open_log_file(self) -> tuple[Any, csv.DictWriter]:
        log_path = self._make_log_path()
        f = log_path.open("w", newline="", encoding="utf-8")
        writer = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        writer.writeheader()
        f.flush()

        with self.state_lock:
            self.state.log_path = log_path
            self.state.log_rollover_requested = False
            self.state.log_rollover_in_progress = False
            self.state.log_rollover_reason = ""
            # Reset the displayed/logged line counter for each new CSV log file.
            # This also makes the CSV line_number column start back at 1 after
            # the next received serial line is parsed.
            self.state.line_count = 0
            self.state.pending_label = None
            self.state.pending_label_line = None
            self.state.pending_rx_id = None
            self.state.array_section = None
            self.state.pending_array_board = None
            self.state.log_status = f"Logging to {log_path.name}"

        self.status_queue.put(f"Logging to {log_path}")
        return f, writer

    def _close_log_file(self, log_file: Optional[Any], closed_path: Optional[Path] = None) -> None:
        if log_file is None:
            return

        # On Windows, a CSV cannot be moved while this process still owns the
        # file handle.  Flush, fsync, close, then force collection of any stale
        # csv writer/file references before reporting that the old log is safe
        # to move or archive.
        try:
            log_file.flush()
        except Exception:
            pass

        try:
            os.fsync(log_file.fileno())
        except Exception:
            pass

        try:
            log_file.close()
        except Exception:
            pass

        gc.collect()

        if closed_path is not None:
            with self.state_lock:
                self.state.last_closed_log_path = closed_path
                self.state.log_status = f"Closed {closed_path.name}; old file is safe to archive"
            self.status_queue.put(f"Closed log file: {closed_path}")

    def _take_log_rollover_request(self) -> bool:
        with self.state_lock:
            requested = self.state.log_rollover_requested
            if requested:
                self.state.log_rollover_requested = False
                self.state.log_rollover_in_progress = True
                self.state.log_status = "Closing current log file..."
            return requested

    def _rollover_log_file(
        self,
        log_file: Optional[Any],
        writer: Optional[csv.DictWriter],
    ) -> tuple[Any, csv.DictWriter]:
        old_path: Optional[Path]
        with self.state_lock:
            old_path = self.state.log_path
            self.state.log_status = "Closing current log file..."

        # Drop the writer reference before closing the file.  csv.writer keeps a
        # reference to the file object's write method, which can delay releasing
        # the Windows file lock if the old writer remains reachable.
        writer = None
        self._close_log_file(log_file, old_path)
        log_file = None
        gc.collect()

        with self.state_lock:
            self.state.log_status = "Opening new log file..."

        return self._open_log_file()

    def run(self) -> None:
        global START_TIME

        log_file: Optional[Any] = None
        writer: Optional[csv.DictWriter] = None

        try:
            log_file, writer = self._open_log_file()
            last_status_message = ""

            while not self.stop_event.is_set():
                if self._take_log_rollover_request():
                    log_file, writer = self._rollover_log_file(log_file, writer)

                port = self._candidate_port()
                if not port:
                    status = f"Searching for BMS serial port; retrying every {SERIAL_RECONNECT_DELAY_S:.0f}s"
                    self._set_serial_status(status, clear_last_line_time=True)
                    if status != last_status_message:
                        self.status_queue.put(status)
                        last_status_message = status
                    self._sleep_until_retry_or_stop()
                    continue

                self._set_serial_status(f"Opening {port} @ {self.baud}", clear_last_line_time=True)

                try:
                    with serial.Serial(port, self.baud, timeout=0.05) as ser:
                        opened_status = f"Opened {port} @ {self.baud}; waiting for BMS data"
                        self._set_serial_status(opened_status, clear_last_line_time=True)
                        self.status_queue.put(opened_status)
                        last_status_message = opened_status

                        line_times: deque[float] = deque(maxlen=5000)
                        opened_at = time.time()
                        last_data_time: Optional[float] = None
                        announced_connected = False
                        last_flush = time.time()

                        while not self.stop_event.is_set():
                            if self._take_log_rollover_request():
                                log_file, writer = self._rollover_log_file(log_file, writer)
                                last_flush = time.time()

                            self._write_pending_commands(ser)

                            try:
                                raw = ser.readline()
                            except (serial.SerialException, OSError) as exc:
                                raise serial.SerialException(exc) from exc

                            now = time.time()

                            if raw:
                                if not announced_connected:
                                    connected_status = f"Connected: {port} @ {self.baud}"
                                    with self.state_lock:
                                        self.state.serial_status = connected_status
                                        START_TIME = self.state.start_time
                                    self.status_queue.put(connected_status)
                                    last_status_message = ""
                                    announced_connected = True

                                last_data_time = now
                                line_times.append(now)
                                try:
                                    line = raw.decode("utf-8", errors="replace").rstrip("\r\n")
                                except Exception:
                                    line = repr(raw)
                                if self.raw_echo:
                                    print(line)

                                with self.state_lock:
                                    row = process_line(line, now, self.state)
                                if writer is not None:
                                    writer.writerow(row)

                                # Green serial indicator is based on data actually being
                                # received and saved, not merely on the COM port opening.
                                with self.state_lock:
                                    self.state.last_serial_line_time = now

                            else:
                                if last_data_time is None:
                                    silent_for = now - opened_at
                                    if silent_for >= SERIAL_CONNECT_NO_DATA_TIMEOUT_S:
                                        raise serial.SerialException(
                                            f"No BMS serial data received from {port} for {silent_for:.1f}s"
                                        )
                                else:
                                    silent_for = now - last_data_time
                                    if silent_for >= SERIAL_STALE_DATA_RECONNECT_S:
                                        raise serial.SerialException(
                                            f"BMS serial data stopped on {port} for {silent_for:.1f}s"
                                        )

                            while line_times and now - line_times[0] > 1.0:
                                line_times.popleft()
                            with self.state_lock:
                                self.state.raw_line_rate_hz = float(len(line_times))

                            if log_file is not None and now - last_flush >= 1.0:
                                log_file.flush()
                                last_flush = now

                except (serial.SerialException, OSError) as exc:
                    exc_text = str(exc)
                    status = f"Serial disconnected/error on {port}: {exc_text}; retrying"
                    self._set_serial_status(status, clear_last_line_time=True)
                    self.status_queue.put(status)
                    lower_exc = exc_text.lower()
                    cooldown = SERIAL_NO_DATA_PORT_COOLDOWN_S if "no bms serial data" in lower_exc else SERIAL_BAD_PORT_COOLDOWN_S
                    self._mark_auto_port_bad(port, exc_text, cooldown)
                    self._sleep_until_retry_or_stop()

                except Exception as exc:
                    exc_text = str(exc)
                    status = f"Logger error: {exc_text}; retrying"
                    self._set_serial_status(status, clear_last_line_time=True)
                    self.status_queue.put(status)
                    self._mark_auto_port_bad(port, exc_text, SERIAL_BAD_PORT_COOLDOWN_S)
                    self._sleep_until_retry_or_stop()

                finally:
                    try:
                        if log_file:
                            log_file.flush()
                    except Exception:
                        pass

        finally:
            old_path: Optional[Path]
            with self.state_lock:
                old_path = self.state.log_path
            writer = None
            self._close_log_file(log_file, old_path)
            with self.state_lock:
                self.state.serial_status = "Stopped" if self.stop_event.is_set() else "Logger stopped"
                self.state.log_status = "Log file closed"


class Dial(ttk.Frame):
    def __init__(
        self,
        parent: tk.Widget,
        title: str,
        min_value: float,
        max_value: float,
        redline_start: Optional[float] = None,
        red_ranges: Optional[list[tuple[float, float]]] = None,
        units: str = "",
        width: int = 300,
        height: int = 190,
    ) -> None:
        super().__init__(parent)
        self.title = title
        self.min_value = min_value
        self.max_value = max_value
        self.redline_start = redline_start
        self.red_ranges = list(red_ranges or [])
        self.units = units
        self.width = width
        self.height = height
        self._last_min: Optional[float] = None
        self._last_max: Optional[float] = None
        self._last_current: Optional[float] = None
        self._last_current_label = "NOW"
        self._last_delta: Optional[float] = None
        self._last_median: Optional[float] = None
        self.canvas = tk.Canvas(self, width=width, height=height, bg=GUI_BG_NORMAL, highlightthickness=0)
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", self._on_canvas_resize)

    def _on_canvas_resize(self, event: tk.Event) -> None:
        self.width = max(1, int(event.width))
        self.height = max(1, int(event.height))
        self._draw_cached()

    def set_canvas_size(self, width: int, height: int) -> bool:
        # Allow the dials to shrink when the window is restored from fullscreen
        # or when the left pane is made shorter.  The draw code already switches
        # into compact mode for small heights.
        width = max(120, int(width))
        height = max(48, int(height))
        old_width = int(float(self.canvas.cget("width")))
        old_height = int(float(self.canvas.cget("height")))
        if abs(width - old_width) < 2 and abs(height - old_height) < 2:
            return False
        self.width = width
        self.height = height
        self.canvas.configure(width=width, height=height)
        self._draw_cached()
        return True

    def _angle_for(self, value: float) -> float:
        v = max(self.min_value, min(self.max_value, value))
        ratio = (v - self.min_value) / (self.max_value - self.min_value)
        return math.radians(210 - ratio * 240)

    def _point(self, cx: float, cy: float, radius: float, value: float) -> tuple[float, float]:
        a = self._angle_for(value)
        return cx + radius * math.cos(a), cy - radius * math.sin(a)

    def draw(
        self,
        min_reading: Optional[float],
        max_reading: Optional[float],
        current_reading: Optional[float] = None,
        current_label: str = "NOW",
        median_reading: Optional[float] = None,
        delta_reading: Optional[float] = None,
    ) -> None:
        self._last_min = min_reading
        self._last_max = max_reading
        self._last_current = current_reading
        self._last_current_label = current_label
        self._last_median = median_reading

        if delta_reading is not None:
            self._last_delta = delta_reading
        elif min_reading is not None and max_reading is not None and median_reading is not None:
            # Only auto-display delta on dials where a median was requested.
            # This keeps the current dial unchanged.
            self._last_delta = max_reading - min_reading
        else:
            self._last_delta = None

        self._draw_cached()

    def _draw_cached(self) -> None:
        c = self.canvas
        c.delete("all")

        w = max(1, self.canvas.winfo_width() or self.width)
        h = max(1, self.canvas.winfo_height() or self.height)
        if h < 40 or w < 80:
            return

        compact = h < 130
        ultra_compact = h < 78
        title_size = 8 if ultra_compact else 10 if compact else 12
        tick_size = 6 if ultra_compact else 7 if compact else 8
        value_size = 7 if ultra_compact else 8 if compact else 10
        marker_text_size = 6 if ultra_compact else 7 if compact else 8
        arc_width = max(5, min(14, int(min(w, h) * 0.075)))
        marker_line_width = max(2, int(arc_width * 0.35))

        cx = w / 2
        cy = h * (0.84 if ultra_compact else 0.80 if compact else 0.78)
        top_pad = 14 if ultra_compact else 20 if compact else 26
        bottom_pad = 12 if ultra_compact else 18 if compact else 28
        r = max(18.0, min(w * 0.38, (cy - top_pad), (h - bottom_pad) * 0.62))

        c.create_text(cx, 9 if ultra_compact else 12 if compact else 18, text=self.title, fill="#F2F2F2", font=("Segoe UI", title_size, "bold"))

        bbox = (cx - r, cy - r, cx + r, cy + r)
        c.create_arc(bbox, start=-30, extent=240, style="arc", width=arc_width, outline="#30363D")

        red_ranges = list(self.red_ranges)
        if self.redline_start is not None:
            red_ranges.append((self.redline_start, self.max_value))

        def dial_angle_deg(value: float) -> float:
            v = max(self.min_value, min(self.max_value, value))
            ratio = (v - self.min_value) / (self.max_value - self.min_value)
            return 210 - ratio * 240

        for red_lo, red_hi in red_ranges:
            lo = max(self.min_value, min(self.max_value, red_lo))
            hi = max(self.min_value, min(self.max_value, red_hi))
            if hi <= lo:
                continue
            start_angle = dial_angle_deg(hi)
            extent = dial_angle_deg(lo) - start_angle
            c.create_arc(bbox, start=start_angle, extent=extent, style="arc", width=arc_width, outline="#D62828")

        # Tick marks
        for i in range(6):
            value = self.min_value + i * (self.max_value - self.min_value) / 5
            x1, y1 = self._point(cx, cy, r - 8, value)
            x2, y2 = self._point(cx, cy, r + 4, value)
            c.create_line(x1, y1, x2, y2, fill="#AAB2BD", width=max(1, arc_width // 5))
            if h >= 88:
                xt, yt = self._point(cx, cy, r + (16 if compact else 22), value)
                c.create_text(xt, yt, text=f"{value:g}", fill="#C9D1D9", font=("Segoe UI", tick_size))

        def draw_marker(value: Optional[float], color: str, label: str, line_scale: float = 1.0) -> None:
            if value is None:
                return
            x, y = self._point(cx, cy, r - 18, value)
            c.create_line(cx, cy, x, y, fill=color, width=max(2, int(marker_line_width * line_scale)), arrow=tk.LAST)
            dot_r = 4 if compact else 5
            c.create_oval(x - dot_r, y - dot_r, x + dot_r, y + dot_r, fill=color, outline="white", width=1)
            if h >= 110:
                c.create_text(x, y - (13 if compact else 18), text=label, fill=color, font=("Segoe UI", marker_text_size, "bold"))

        draw_marker(self._last_min, "#58A6FF", "MIN")
        draw_marker(self._last_max, "#FF5C5C", "MAX")
        draw_marker(self._last_current, "#FFD23F", self._last_current_label, 1.2)
        center_r = 4 if compact else 5
        c.create_oval(cx - center_r, cy - center_r, cx + center_r, cy + center_r, fill="#F2F2F2", outline="")

        def fmt(value: Optional[float]) -> str:
            return "NaN" if value is None else f"{value:.3g}{self.units}"

        min_text = fmt(self._last_min)
        max_text = fmt(self._last_max)
        if self._last_current is None:
            bottom_text = f"Min: {min_text}    Max: {max_text}"
        else:
            bottom_text = f"Now: {fmt(self._last_current)}    Min: {min_text}    Max: {max_text}"

        stats_parts: list[str] = []
        if self._last_delta is not None:
            stats_parts.append(f"Delta: {fmt(self._last_delta)}")
        if self._last_median is not None:
            stats_parts.append(f"Median: {fmt(self._last_median)}")
        stats_text = "    ".join(stats_parts)

        if stats_text and h >= 118:
            c.create_text(
                cx,
                h - (24 if compact else 30),
                text=bottom_text,
                fill="#F2F2F2",
                font=("Segoe UI", value_size, "bold"),
            )
            c.create_text(
                cx,
                h - (9 if compact else 14),
                text=stats_text,
                fill="#F2F2F2",
                font=("Segoe UI", max(6, value_size - 1), "bold"),
            )
        elif stats_text:
            c.create_text(
                cx,
                h - (10 if compact else 18),
                text=f"{bottom_text}    {stats_text}",
                fill="#F2F2F2",
                font=("Segoe UI", max(6, value_size - 1), "bold"),
            )
        else:
            c.create_text(
                cx,
                h - (10 if compact else 18),
                text=bottom_text,
                fill="#F2F2F2",
                font=("Segoe UI", value_size, "bold"),
            )


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
            bg=GUI_BG_NORMAL,
            highlightthickness=1,
            highlightbackground="#333333",
        )
        self.canvas.pack(fill="both", expand=False)

    def resize_to_width(self, width: int) -> bool:
        """Resize the module image/canvas while preserving image aspect ratio.

        Overlay coordinates stay correct because every badge/circle is scaled
        from the fixed source-image coordinates by scale_point().
        Returns True only when the rendered size actually changed.
        """
        width = max(320, int(width))
        if abs(width - self.display_width) < 2:
            return False

        self.display_width = width
        self.display_height = max(1, int(round(self.src_h * (self.display_width / self.src_w))))
        self.resized = self.source_image.resize((self.display_width, self.display_height), Image.LANCZOS)
        self.photo = ImageTk.PhotoImage(self.resized)
        self.canvas.configure(width=self.display_width, height=self.display_height)
        return True

    def width_for_canvas_height(self, canvas_height: int) -> int:
        canvas_height = max(1, int(canvas_height))
        return max(320, int(round(canvas_height * self.src_w / self.src_h)))

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
    def draw_gradient_oval(
        canvas: tk.Canvas,
        x: float,
        y: float,
        radius: float,
        fill: str,
        outline: str,
        outline_width: int,
    ) -> None:
        """Draw a circular marker with a shaded fill and a separate outline color."""
        if fill == "#333333":
            canvas.create_oval(
                x - radius,
                y - radius,
                x + radius,
                y + radius,
                fill=fill,
                outline=outline,
                width=outline_width,
            )
            return

        # Tk canvas does not have native gradient fills, so draw concentric
        # ovals from a darker edge toward a brighter center.  The outline is
        # allowed to be a different color from the fill.
        edge_fill = blend_hex(fill, "#050505", 0.28)
        center_fill = blend_hex(fill, "#FFFFFF", 0.10)
        steps = 12
        for step in range(steps, 0, -1):
            ring_ratio = (step - 1) / max(1, steps - 1)
            ring_radius = radius * step / steps
            ring_fill = blend_hex(center_fill, edge_fill, ring_ratio)
            canvas.create_oval(
                x - ring_radius,
                y - ring_radius,
                x + ring_radius,
                y + ring_radius,
                fill=ring_fill,
                outline=ring_fill,
                width=1,
            )

        canvas.create_oval(
            x - radius,
            y - radius,
            x + radius,
            y + radius,
            fill="",
            outline=outline,
            width=outline_width,
        )

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
            border = voltage_color(color_value)

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

            # Fill uses the board-pair color; outline carries the temperature
            # gradient.  For all-board min/max, use the hottest board as the
            # fill owner because it is the most safety-relevant board at this
            # thermistor location.
            fill_owner = max_item.get("board") if max_item else (min_item.get("board") if min_item else None)
            fill = marker_pair_color(fill_owner)
            border = temp_color(color_value)

            has_fault = any(self.is_temp_fault(item.get("value")) for item in (min_item, max_item) if item)
            if has_fault:
                fill = self.flashing_fill(fill, "#FF2B2B", flash_on)
                border = "#FFFFFF" if flash_on else "#FF2B2B"

            x, y = self.scale_point(point)
            r = 27 if self.display_width < 900 else 31
            self.draw_gradient_oval(c, x, y, r, fill, border, 5 if has_fault else 4)
            label = (
                f"U{sensor}\n"
                f"lo {item_text(min_item, 'C', 1)}\n"
                f"hi {item_text(max_item, 'C', 1)}"
            )
            c.create_text(
                x,
                y,
                text=label,
                fill=readable_text_color(fill) if fill != "#333333" else "#D0D0D0",
                font=("Segoe UI", 7, "bold"),
                justify="center",
            )

        overall_vmin = allboard_ranges.get("vmin")
        overall_vmax = allboard_ranges.get("vmax")
        overall_tmin = allboard_ranges.get("tmin")
        overall_tmax = allboard_ranges.get("tmax")

        c.create_rectangle(4, 4, 360, 30, fill=GUI_BG_NORMAL, outline=board_color, width=2)
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
        c.create_rectangle(4, 34, 470, 78, fill=GUI_BG_NORMAL, outline="#445", width=1)
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

        # Voltage badges: board-pair fill; purple->blue voltage-gradient border.
        for cell, point in self.cell_coords.items():
            if not self.voltage_allowed(cell):
                continue

            value = filtered_voltages.get(cell)
            x, y = self.scale_point(point)
            label = f"C{cell}\nNaN" if value is None else f"C{cell}\n{value:.3f}V"

            # Fill uses the selected board-pair color.  The border carries
            # the voltage gradient so voltage is shown without overwriting the
            # board identity color.
            fill = board_color
            text_color = readable_text_color(fill)
            outline = voltage_color(value)
            outline_width = 3

            # Selected min/max are shown by a thicker border, but the border
            # remains the voltage-gradient color.
            if cell == vmin_cell:
                outline_width = 4
            if cell == vmax_cell:
                outline_width = 4
            if cell == vmin_cell and cell == vmax_cell:
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

            # Draw the voltage value after the badge rectangle so it is not
            # covered. This is the All cells/temps display for the selected
            # board pair only; Min/Max mode returns earlier through
            # draw_allboard_range_overlay().
            c.create_text(
                x,
                y,
                text=label,
                fill=text_color,
                font=("Segoe UI", 8, "bold"),
                justify="center",
            )

        # Temperature circles: board-pair fill; green->yellow->red temperature-gradient border.
        for sensor, point in self.temp_coords.items():
            value = cleaned_temps.get(sensor)
            x, y = self.scale_point(point)

            # Fill uses the selected board-pair color.  The border carries
            # the temperature gradient so temperature is shown without
            # overwriting the board identity color.
            fill = board_color
            r = 17 if self.display_width < 900 else 20

            outline = temp_color(value)
            outline_width = 4
            # Selected min/max are shown by a thicker border, but the border
            # remains the temperature-gradient color.
            if sensor == tmin_sensor:
                outline_width = 5
            if sensor == tmax_sensor:
                outline_width = 5
            if sensor == tmin_sensor and sensor == tmax_sensor:
                outline_width = 5

            if self.is_temp_fault(value):
                fill = self.flashing_fill(fill, "#FF2B2B", flash_on)
                outline = "#FFFFFF" if flash_on else "#FF2B2B"
                outline_width = 6

            self.draw_gradient_oval(c, x, y, r, fill, outline, outline_width)
            label = f"U{sensor}\nNaN" if value is None else f"U{sensor}\n{value:.1f}C"
            c.create_text(
                x,
                y,
                text=label,
                fill=readable_text_color(fill) if fill != "#333333" else "#D0D0D0",
                font=("Segoe UI", 8, "bold"),
                justify="center",
            )

        c.create_rectangle(4, 4, 292, 30, fill=GUI_BG_NORMAL, outline=board_color, width=2)
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
        c.create_rectangle(4, 34, 490, 78, fill=GUI_BG_NORMAL, outline="#445", width=1)
        c.create_text(12, 56, anchor="w", text=summary, fill="#F2F2F2", font=("Segoe UI", 8, "bold"))


class GraphView(ttk.Frame):
    def __init__(self, parent: tk.Widget) -> None:
        super().__init__(parent)
        self.time_window_var = tk.StringVar(value="60")
        self.series_vars: dict[str, tk.BooleanVar] = {
            key: tk.BooleanVar(value=key in DEFAULT_GRAPH_KEYS)
            for key, _label, _unit, _color in GRAPH_SERIES
        }

        controls = ttk.Frame(self)
        controls.pack(fill="x", pady=(0, 4))

        ttk.Label(controls, text="Graph window:", font=("Segoe UI", 9, "bold")).pack(side="left", padx=(0, 4))
        self.window_entry = ttk.Entry(controls, textvariable=self.time_window_var, width=8)
        self.window_entry.pack(side="left", padx=(0, 4))
        ttk.Label(controls, text="seconds").pack(side="left", padx=(0, 10))
        ttk.Button(controls, text="Select all", command=self.select_all).pack(side="left", padx=(0, 4))
        ttk.Button(controls, text="Clear", command=self.clear_all).pack(side="left", padx=(0, 10))

        self.series_frame = ttk.Frame(self)
        self.series_frame.pack(fill="x", pady=(0, 4))
        self.build_series_checks(columns=4)

        self.canvas = tk.Canvas(self, bg=GUI_BG_NORMAL, highlightthickness=1, highlightbackground="#333333")
        self.canvas.pack(fill="both", expand=True)
        self.canvas.bind("<Configure>", lambda _event: self._draw_cached())
        self._last_samples: list[tuple[float, str, float, str]] = []
        self._last_now = time.time()

    def build_series_checks(self, columns: int = 4) -> None:
        for child in self.series_frame.winfo_children():
            child.destroy()
        columns = max(1, columns)
        for idx, (key, label, unit, color) in enumerate(GRAPH_SERIES):
            r = idx // columns
            c = idx % columns
            item = ttk.Frame(self.series_frame)
            item.grid(row=r, column=c, sticky="w", padx=(0, 10), pady=1)
            swatch = tk.Canvas(item, width=12, height=12, bg=GUI_BG_NORMAL, highlightthickness=0)
            swatch.pack(side="left", padx=(0, 3))
            swatch.create_rectangle(1, 1, 11, 11, fill=color, outline="#F2F2F2", width=1)
            ttk.Checkbutton(
                item,
                text=f"{label} ({unit})",
                variable=self.series_vars[key],
                command=self._draw_cached,
            ).pack(side="left")
        for c in range(columns):
            self.series_frame.columnconfigure(c, weight=1)

    def select_all(self) -> None:
        for var in self.series_vars.values():
            var.set(True)
        self._draw_cached()

    def clear_all(self) -> None:
        for var in self.series_vars.values():
            var.set(False)
        self._draw_cached()

    def window_seconds(self) -> float:
        text = self.time_window_var.get().strip()
        try:
            value = float(text)
        except ValueError:
            value = 60.0
        return max(1.0, min(7200.0, value))

    def draw_samples(self, samples: list[tuple[float, str, float, str]], now: Optional[float] = None) -> None:
        self._last_samples = samples
        self._last_now = time.time() if now is None else now
        self._draw_cached()

    def _draw_cached(self) -> None:
        c = self.canvas
        c.delete("all")
        w = max(1, c.winfo_width())
        h = max(1, c.winfo_height())
        if w < 160 or h < 120:
            return

        window_s = self.window_seconds()
        now = self._last_now
        cutoff = now - window_s
        selected_keys = [key for key, var in self.series_vars.items() if var.get()]
        if not selected_keys:
            c.create_text(w / 2, h / 2, text="Select one or more data channels to graph.", fill="#F2F2F2", font=("Segoe UI", 12, "bold"))
            return

        grouped: dict[str, dict[str, list[tuple[float, float]]]] = defaultdict(lambda: defaultdict(list))
        for sample_time, key, value, unit in self._last_samples:
            if sample_time < cutoff or key not in selected_keys:
                continue
            meta = GRAPH_SERIES_META.get(key, {})
            graph_unit = str(meta.get("unit", unit or "value"))
            grouped[graph_unit][key].append((sample_time, value))

        if not grouped:
            c.create_text(
                w / 2,
                h / 2,
                text=f"No selected graph data in the last {window_s:g} seconds.",
                fill="#F2F2F2",
                font=("Segoe UI", 12, "bold"),
            )
            return

        units = [unit for unit in ["V", "A", "C", "%", "kW", "value"] if unit in grouped]
        units += [unit for unit in grouped if unit not in units]
        graph_count = len(units)
        top_pad = 10
        gap = 16
        graph_h = max(90, int((h - top_pad - gap * (graph_count - 1) - 10) / graph_count))
        y = top_pad

        for unit in units:
            series_by_key = grouped[unit]
            self.draw_unit_graph(c, 10, y, w - 20, graph_h, unit, series_by_key, cutoff, now)
            y += graph_h + gap

    def draw_unit_graph(
        self,
        c: tk.Canvas,
        x: int,
        y: int,
        w: int,
        h: int,
        unit: str,
        series_by_key: dict[str, list[tuple[float, float]]],
        cutoff: float,
        now: float,
    ) -> None:
        left_pad = 62
        right_pad = 14
        top_pad = 26
        bottom_pad = 24
        plot_left = x + left_pad
        plot_right = x + w - right_pad
        plot_top = y + top_pad
        plot_bottom = y + h - bottom_pad
        plot_w = max(1, plot_right - plot_left)
        plot_h = max(1, plot_bottom - plot_top)

        all_values = [value for values in series_by_key.values() for _t, value in values]
        if not all_values:
            return
        y_min = min(all_values)
        y_max = max(all_values)
        if unit == "A":
            # Keep the current graph visually comparable to the current gauge range.
            y_min = min(y_min, CURRENT_DIAL_MIN_A)
            y_max = max(y_max, CURRENT_DIAL_MAX_A)
        if math.isclose(y_min, y_max):
            pad = max(1.0, abs(y_min) * 0.05)
            y_min -= pad
            y_max += pad
        else:
            pad = (y_max - y_min) * 0.08
            y_min -= pad
            y_max += pad

        c.create_rectangle(x, y, x + w, y + h, fill=GUI_BG_NORMAL, outline="#30363D", width=1)
        c.create_text(x + 8, y + 12, anchor="w", text=f"{unit} graph", fill="#F2F2F2", font=("Segoe UI", 10, "bold"))

        # Grid and y-axis labels.
        for i in range(5):
            frac = i / 4
            yy = plot_bottom - frac * plot_h
            value = y_min + frac * (y_max - y_min)
            c.create_line(plot_left, yy, plot_right, yy, fill="#27313A", width=1)
            c.create_text(plot_left - 6, yy, anchor="e", text=f"{value:.3g}", fill="#C9D1D9", font=("Segoe UI", 8))
        for i in range(5):
            frac = i / 4
            xx = plot_left + frac * plot_w
            c.create_line(xx, plot_top, xx, plot_bottom, fill="#1B222A", width=1)

        c.create_rectangle(plot_left, plot_top, plot_right, plot_bottom, outline="#8B949E", width=1)
        c.create_text(plot_left, plot_bottom + 14, anchor="w", text=f"-{now - cutoff:.0f}s", fill="#C9D1D9", font=("Segoe UI", 8))
        c.create_text(plot_right, plot_bottom + 14, anchor="e", text="now", fill="#C9D1D9", font=("Segoe UI", 8))

        legend_x = plot_left + 4
        legend_y = y + 12
        for key, values in series_by_key.items():
            meta = GRAPH_SERIES_META.get(key, {})
            label = str(meta.get("label", key))
            color = str(meta.get("color", "#F2F2F2"))
            if not values:
                continue

            points: list[float] = []
            for sample_time, value in values:
                xx = plot_left + ((sample_time - cutoff) / max(0.001, now - cutoff)) * plot_w
                yy = plot_bottom - ((value - y_min) / max(0.001, y_max - y_min)) * plot_h
                points.extend([xx, yy])
            if len(points) >= 4:
                c.create_line(*points, fill=color, width=2, smooth=False)
            elif len(points) == 2:
                xx, yy = points
                c.create_oval(xx - 3, yy - 3, xx + 3, yy + 3, fill=color, outline="white")

            c.create_rectangle(legend_x, legend_y - 5, legend_x + 10, legend_y + 5, fill=color, outline="#F2F2F2")
            c.create_text(legend_x + 14, legend_y, anchor="w", text=label, fill="#F2F2F2", font=("Segoe UI", 8, "bold"))
            legend_x += 120
            if legend_x > x + w - 110:
                legend_x = plot_left + 4
                legend_y += 14


class BMSGuiApp:
    def __init__(
        self,
        root: tk.Tk,
        state: MonitorState,
        state_lock: threading.RLock,
        stop_event: threading.Event,
        command_queue: "queue.Queue[str]",
        left_image: Path,
        right_image: Path,
    ) -> None:
        self.root = root
        self.state = state
        self.state_lock = state_lock
        self.stop_event = stop_event
        self.command_queue = command_queue
        self.status_queue: "queue.Queue[str]" = queue.Queue()
        self.selected_pair = tk.IntVar(value=1)
        self.overlay_filter = tk.StringVar(value="minmax")
        self.view_mode = tk.StringVar(value="overlay")
        self.vars: dict[str, tk.StringVar] = {}
        self._left_resize_job: Optional[str] = None
        self._last_left_layout_geometry: tuple[int, int] = (0, 0)

        root.title("EV4 BMS Serial GUI")
        root.geometry("1600x980")
        root.minsize(1200, 760)
        root.configure(bg=GUI_BG_NORMAL)

        self.style = ttk.Style()
        try:
            self.style.theme_use("clam")
        except Exception:
            pass
        self.style.configure(".", font=("Segoe UI", 9))
        self.style.configure("TFrame", background=GUI_BG_NORMAL)
        self.style.configure("TLabelframe", background=GUI_BG_NORMAL, foreground="#F2F2F2")
        self.style.configure("TLabelframe.Label", background="#5E5E5E", foreground="#F2F2F2", font=("Segoe UI", 10, "bold"))
        self.style.configure("TLabel", background=GUI_BG_NORMAL, foreground="#F2F2F2")
        self.style.configure("TButton", padding=5)
        self.style.configure("Pair.TButton", padding=4)

        self.build_ui(left_image, right_image)
        # A restore/maximize transition can resize the root before all child panes
        # have finished reporting their new requested sizes.  Watch the root too,
        # not only the left pane, so the gauges are recalculated after every
        # window-size change.
        self.root.bind("<Configure>", self.on_root_configure, add="+")
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.schedule_update()

    def serial_stream_active(self) -> bool:
        with self.state_lock:
            serial_status = self.state.serial_status
            last_line_time = self.state.last_serial_line_time

        return (
            serial_status.lower().startswith("connected")
            and last_line_time is not None
            and (time.time() - last_line_time) <= SERIAL_DATA_ALIVE_TIMEOUT_S
        )

    def send_serial_command(self, event: Optional[tk.Event] = None) -> str:
        command = self.serial_command_var.get().rstrip("\r\n")
        if not command.strip():
            message = "Command is empty; nothing sent"
            self.serial_command_status_var.set(message)
            with self.state_lock:
                self.state.serial_command_status = message
            return "break"

        if not self.serial_stream_active():
            message = "Serial not connected; command not sent"
            self.serial_command_status_var.set(message)
            with self.state_lock:
                self.state.serial_command_status = message
            return "break"

        self.command_queue.put(command)
        message = f"Queued: {command}"
        self.serial_command_status_var.set(message)
        with self.state_lock:
            self.state.serial_command_status = message
        self.serial_command_var.set("")
        return "break"

    def request_new_log_file(self) -> None:
        with self.state_lock:
            if self.state.log_rollover_requested or self.state.log_rollover_in_progress:
                self.state.log_status = "New log already requested; waiting for logger..."
                return
            self.state.log_rollover_requested = True
            self.state.log_rollover_reason = "manual_button"
            current_log = self.state.log_path.name if self.state.log_path else "current log"
            self.state.log_status = f"New log requested; closing {current_log}..."
        self.status_queue.put("Manual log rollover requested")

    def make_var(self, name: str, default: str = "NaN") -> tk.StringVar:
        v = tk.StringVar(value=default)
        self.vars[name] = v
        return v

    def build_ui(self, left_image: Path, right_image: Path) -> None:
        # Sash-resizable three-pane layout:
        #   left   = status/dials
        #   center = image overlays or graph mode
        #   right  = alerts/events/BMS values/charger/CAN
        main = ttk.Panedwindow(self.root, orient=tk.HORIZONTAL)
        main.pack(fill="both", expand=True, padx=8, pady=8)
        self.main_paned = main

        left_panel = ttk.Frame(main, width=410)
        map_panel = ttk.Frame(main, width=860)
        event_panel = ttk.Frame(main, width=300)

        try:
            main.add(left_panel, weight=0)
            main.add(map_panel, weight=1)
            main.add(event_panel, weight=0)
        except tk.TclError:
            main.add(left_panel)
            main.add(map_panel)
            main.add(event_panel)

        self.left_panel = left_panel
        self.map_panel = map_panel
        self.event_panel = event_panel

        map_panel.columnconfigure(0, weight=1)
        map_panel.rowconfigure(2, weight=1)

        event_panel.columnconfigure(0, weight=1)
        # Right-side sections use their requested heights so Active Alerts and
        # Recent Events only consume the number of text lines they actually show.
        for right_row in range(5):
            event_panel.rowconfigure(right_row, weight=0)
        event_panel.rowconfigure(5, weight=1)  # bottom spacer absorbs extra height

        log_controls = ttk.Frame(left_panel)
        log_controls.pack(fill="x", pady=(0, 6))
        ttk.Button(
            log_controls,
            text="Start New Log File",
            command=self.request_new_log_file,
        ).pack(side="left", padx=(0, 6))

        command_controls = ttk.LabelFrame(left_panel, text="Serial Command")
        command_controls.pack(fill="x", pady=(0, 6))
        command_controls.columnconfigure(0, weight=1)

        self.serial_command_var = tk.StringVar()
        self.serial_command_status_var = tk.StringVar(value="Enter a BMS command and press Send")

        self.serial_command_entry = ttk.Entry(
            command_controls,
            textvariable=self.serial_command_var,
        )
        self.serial_command_entry.grid(row=0, column=0, sticky="we", padx=(4, 4), pady=(4, 2))
        self.serial_command_entry.bind("<Return>", self.send_serial_command)

        ttk.Button(
            command_controls,
            text="Send",
            command=self.send_serial_command,
        ).grid(row=0, column=1, sticky="e", padx=(0, 4), pady=(4, 2))

        ttk.Label(
            command_controls,
            textvariable=self.serial_command_status_var,
            wraplength=360,
        ).grid(row=1, column=0, columnspan=2, sticky="w", padx=4, pady=(0, 4))

        self.build_status_panel(left_panel)

        self.dial_area = ttk.Frame(left_panel)
        self.dial_area.pack(fill="x", pady=(8, 0))
        for col in range(3):
            self.dial_area.columnconfigure(col, weight=1, uniform="gauges")

        self.voltage_dial = Dial(self.dial_area, "Cell Voltage Min/Max", 2.5, 4.2, units="V")
        self.temp_dial = Dial(self.dial_area, "Cell Temperature Min/Max", -20, 80, redline_start=60, units="C")
        self.current_dial = Dial(
            self.dial_area,
            "Current / 60s Min-Max",
            CURRENT_DIAL_MIN_A,
            CURRENT_DIAL_MAX_A,
            red_ranges=[(CURRENT_DIAL_MIN_A, -9.0), (240.0, CURRENT_DIAL_MAX_A)],
            units="A",
        )
        self.resizable_dials = [self.voltage_dial, self.temp_dial, self.current_dial]
        self._dial_layout_mode = ""
        self.layout_dials("stack", left_panel.winfo_reqwidth(), 120)

        left_panel.bind("<Configure>", self.request_left_panel_resize)
        main.bind("<ButtonRelease-1>", lambda _event: (self.request_left_panel_resize(), self.request_map_resize()))
        self.root.after_idle(self.resize_left_gauges_to_available_space)

        top = ttk.Frame(map_panel)
        top.grid(row=0, column=0, sticky="we")
        top.columnconfigure(0, weight=1)

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

        self.view_toggle = ttk.Button(controls, text="Graph Mode", command=self.toggle_view_mode)
        self.view_toggle.pack(side="left", padx=(16, 0))

        self.legend = tk.Canvas(map_panel, height=54, bg=GUI_BG_NORMAL, highlightthickness=0)
        self.legend.grid(row=1, column=0, sticky="we", pady=(4, 4))
        self.legend.bind("<Configure>", lambda _event: self.draw_legend())

        content = ttk.Frame(map_panel)
        content.grid(row=2, column=0, sticky="nsew")
        content.columnconfigure(0, weight=1)
        content.rowconfigure(0, weight=1)
        self.content_frame = content

        maps = ttk.Frame(content)
        maps.grid(row=0, column=0, sticky="nsew")
        maps.columnconfigure(0, weight=1)
        maps.rowconfigure(0, weight=0)
        maps.rowconfigure(1, weight=0)
        self.maps_frame = maps
        self._map_resize_job: Optional[str] = None

        # The map width is no longer fixed.  Start at a conservative size so
        # the side panels are not pushed off-screen, then resize to the actual
        # center-column space after Tk finishes laying out the window.
        self.left_map = ModuleMap(
            maps,
            "Left Side: EVEN Cell Voltages / Odd U1-U19",
            left_image,
            LEFT_CELL_COORDS,
            LEFT_TEMP_COORDS,
            width=840,
            voltage_parity="even",
        )
        self.left_map.grid(row=0, column=0, sticky="n", pady=(0, 8))

        self.right_map = ModuleMap(
            maps,
            "Right Side: ODD Cell Voltages / Even U2-U20",
            right_image,
            RIGHT_CELL_COORDS,
            RIGHT_TEMP_COORDS,
            width=840,
            voltage_parity="odd",
        )
        self.right_map.grid(row=1, column=0, sticky="n")

        self.graph_view = GraphView(content)
        self.graph_view.grid(row=0, column=0, sticky="nsew")
        self.graph_view.grid_remove()

        map_panel.bind("<Configure>", self.request_map_resize)
        maps.bind("<Configure>", self.request_map_resize)
        content.bind("<Configure>", self.request_map_resize)
        self.root.after_idle(self.resize_maps_to_available_space)

        self.alert_text = self.make_text_box(event_panel, "ACTIVE ALERTS", 0, height=ACTIVE_ALERT_LINES, width=36)
        self.event_text = self.make_text_box(event_panel, "RECENT EVENTS", 1, height=RECENT_EVENT_LINES, width=36)

        self.build_bms_values_panel(event_panel, row=2)
        self.build_charger_panel(event_panel, row=3)
        self.build_can_panel(event_panel, row=4)


    def layout_dials(self, mode: str, panel_w: int, dial_h: int) -> None:
        """Reflow the three gauge widgets based on available side-panel space."""
        if not hasattr(self, "dial_area"):
            return

        for child in getattr(self, "resizable_dials", []):
            child.grid_forget()

        for r in range(3):
            self.dial_area.rowconfigure(r, weight=0)
        for c in range(3):
            self.dial_area.columnconfigure(c, weight=1, uniform="gauges")

        panel_w = max(140, int(panel_w))
        dial_h = max(48, int(dial_h))

        if mode == "row3":
            area_h = dial_h
            gauge_w = max(120, int((panel_w - 16) / 3))
            for idx, dial in enumerate(self.resizable_dials):
                dial.grid(row=0, column=idx, sticky="nsew", padx=(0 if idx == 0 else 4, 0), pady=0)
                dial.set_canvas_size(gauge_w, dial_h)
        elif mode == "row2_current_below":
            area_h = dial_h * 2 + 8
            half_w = max(120, int((panel_w - 12) / 2))
            full_w = max(160, panel_w - 8)
            self.voltage_dial.grid(row=0, column=0, sticky="nsew", padx=(0, 4), pady=(0, 4))
            self.temp_dial.grid(row=0, column=1, sticky="nsew", padx=(4, 0), pady=(0, 4))
            self.current_dial.grid(row=1, column=0, columnspan=2, sticky="nsew", pady=(4, 0))
            self.voltage_dial.set_canvas_size(half_w, dial_h)
            self.temp_dial.set_canvas_size(half_w, dial_h)
            self.current_dial.set_canvas_size(full_w, dial_h)
        else:
            area_h = dial_h * 3 + 16
            full_w = max(160, panel_w - 8)
            for idx, dial in enumerate(self.resizable_dials):
                dial.grid(row=idx, column=0, columnspan=3, sticky="nsew", pady=(0 if idx == 0 else 8, 0))
                dial.set_canvas_size(full_w, dial_h)

        # Make the gauge section obey the calculated height.  Without this, the
        # dial canvases can keep an old large requested height after a window is
        # restored from fullscreen, pushing BMS/Charger rows below the visible
        # left panel.
        self.dial_area.configure(height=max(48, area_h))
        self.dial_area.pack_propagate(False)
        self.dial_area.grid_propagate(False)
        self._dial_layout_mode = mode

    def toggle_view_mode(self) -> None:
        if self.view_mode.get() == "overlay":
            self.view_mode.set("graph")
            self.maps_frame.grid_remove()
            self.graph_view.grid()
            self.view_toggle.configure(text="Image Overlay")
            self.legend.grid_remove()
        else:
            self.view_mode.set("overlay")
            self.graph_view.grid_remove()
            self.maps_frame.grid()
            self.legend.grid()
            self.view_toggle.configure(text="Graph Mode")
            self.request_map_resize()
        self.force_redraw()

    def on_root_configure(self, event: tk.Event) -> None:
        """Handle full-screen/restore window transitions reliably.

        Tk often emits several configure events while a maximized window is being
        restored.  Recalculate after the final size has settled so the gauges can
        shrink as well as grow.
        """
        if event.widget is self.root:
            self.request_left_panel_resize()
            self.request_map_resize()

    def request_left_panel_resize(self, event: Optional[tk.Event] = None) -> None:
        """Debounce gauge resizing while the left panel/window is changing."""
        existing = getattr(self, "_left_resize_job", None)
        if existing is not None:
            try:
                self.root.after_cancel(existing)
            except Exception:
                pass
        self._left_resize_job = self.root.after(80, self.resize_left_gauges_to_available_space)

    def resize_left_gauges_to_available_space(self) -> None:
        """Resize/reflow gauges so fixed left-panel sections remain visible.

        The gauges are the only flexible part of the left panel.  They expand into
        blank space on tall windows, but they also shrink immediately when the
        window is restored from fullscreen or made shorter.
        """
        self._left_resize_job = None

        if not hasattr(self, "left_panel") or not hasattr(self, "resizable_dials"):
            return

        # Flush pending geometry work so winfo_height() reflects the new restored
        # size instead of the previous fullscreen size.
        try:
            self.root.update_idletasks()
        except Exception:
            pass

        panel_h = int(self.left_panel.winfo_height())
        panel_w = int(self.left_panel.winfo_width())
        if panel_h < 120 or panel_w < 120:
            return

        fixed_h = 0
        fixed_count = 0
        for child in self.left_panel.winfo_children():
            if child is getattr(self, "dial_area", None):
                continue
            if not child.winfo_ismapped():
                continue

            # Use requested height for fixed panels so they are allocated enough
            # room even if they are currently clipped by an older oversized gauge
            # layout.
            req_h = child.winfo_reqheight()
            actual_h = child.winfo_height()
            fixed_h += max(req_h, actual_h if actual_h > 1 else 0)
            fixed_count += 1

        reserved_padding = 12 + fixed_count * 2
        available_for_dials = panel_h - fixed_h - reserved_padding

        # Layout preference:
        #   wide panel   -> three gauges on one row
        #   medium panel -> voltage/temp on one row, current below
        #   narrow panel -> stacked, with height compressed as needed
        if panel_w >= 720 and available_for_dials >= 70:
            mode = "row3"
            dial_h = max(48, min(320, available_for_dials - 4))
        elif panel_w >= 520 and available_for_dials >= 112:
            mode = "row2_current_below"
            dial_h = max(48, min(320, int((available_for_dials - 8) / 2)))
        else:
            mode = "stack"
            dial_h = max(48, min(320, int((available_for_dials - 16) / 3)))

        self.layout_dials(mode, panel_w, dial_h)

        # A second pass catches the final pane height after Tk applies the new
        # gauge requested size.  This is important during maximize/restore where
        # the first configure event may be intermediate.
        geometry = (panel_w, panel_h)
        if geometry != getattr(self, "_last_left_layout_geometry", (0, 0)):
            self._last_left_layout_geometry = geometry
            self.root.after(180, self.request_left_panel_resize)

        self.root.after_idle(self.force_redraw)

    def request_map_resize(self, event: Optional[tk.Event] = None) -> None:
        """Debounce center-map resizing while the window/layout is changing."""
        if getattr(self, "_map_resize_job", None) is not None:
            return
        self._map_resize_job = self.root.after(50, self.resize_maps_to_available_space)

    def resize_maps_to_available_space(self) -> None:
        """Fit the two image overlays inside the center area between side panels.

        The source coordinates do not change.  Only the rendered image/canvas
        size changes, and ModuleMap.scale_point() keeps every cell-voltage badge
        and temperature circle locked to the same relative position on the image.
        """
        self._map_resize_job = None

        if not hasattr(self, "maps_frame"):
            return
        if hasattr(self, "view_mode") and self.view_mode.get() == "graph":
            return

        # Width available inside the center panel.
        container = getattr(self, "content_frame", self.maps_frame)
        available_width = max(0, container.winfo_width() - 4)

        # Height available below the controls/legend.  Use it too so both stacked
        # maps stay visible instead of forcing a horizontal/vertical overflow.
        available_height = max(0, container.winfo_height() - 8)

        if available_width < 100:
            return

        width_from_height: Optional[int] = None
        if available_height >= 250:
            # Two maps are stacked vertically.  Subtract their label/requested
            # header heights and the inter-map gap before computing canvas size.
            label_h = max(
                self.left_map.winfo_children()[0].winfo_reqheight(),
                self.right_map.winfo_children()[0].winfo_reqheight(),
                18,
            )
            canvas_height_each = max(160, int((available_height - (2 * label_h) - 12) / 2))
            width_from_height = min(
                self.left_map.width_for_canvas_height(canvas_height_each),
                self.right_map.width_for_canvas_height(canvas_height_each),
            )

        desired_width = available_width
        if width_from_height is not None:
            desired_width = min(desired_width, width_from_height)

        desired_width = max(320, int(desired_width))

        changed = False
        changed = self.left_map.resize_to_width(desired_width) or changed
        changed = self.right_map.resize_to_width(desired_width) or changed

        if changed:
            self.root.after_idle(self.force_redraw)

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
            bg=GUI_BG_NORMAL,
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
            ("Cell T", "cell_temp_status"),
            ("Current", "current_status"),
            ("Mode", "mode"),
            ("Run/Lines/Rate", "runtime_line_rate"),
            ("Log", "log"),
            ("Log State", "log_status"),
        ]

        for r, (label, key) in enumerate(fields):
            ttk.Label(frame, text=label + ":", width=14).grid(row=r, column=0, sticky="w", padx=4, pady=2)

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

            elif key in {"cell_voltage_status", "cell_temp_status", "current_status"}:
                activity_frame = ttk.Frame(frame)
                activity_frame.grid(row=r, column=1, sticky="w", padx=4, pady=2)

                indicator = tk.Canvas(activity_frame, width=14, height=14, bg=GUI_BG_NORMAL, highlightthickness=0)
                indicator.pack(side="left", padx=(0, 5))
                indicator_id = indicator.create_oval(
                    2, 2, 12, 12,
                    fill="#FFD23F",  # idle yellow
                    outline="#F2F2F2",
                    width=1,
                )

                if key == "cell_voltage_status":
                    self.cell_voltage_indicator = indicator
                    self.cell_voltage_indicator_id = indicator_id
                elif key == "cell_temp_status":
                    self.cell_temp_indicator = indicator
                    self.cell_temp_indicator_id = indicator_id
                else:
                    self.current_indicator = indicator
                    self.current_indicator_id = indicator_id

                ttk.Label(activity_frame, textvariable=self.make_var(key), wraplength=245).pack(side="left")

            elif key == "runtime_line_rate":
                compact_frame = ttk.Frame(frame)
                compact_frame.grid(row=r, column=1, sticky="w", padx=4, pady=2)

                ttk.Label(compact_frame, text="Runtime ").pack(side="left")
                ttk.Label(compact_frame, textvariable=self.make_var("runtime"), width=8).pack(side="left", padx=(0, 8))
                ttk.Label(compact_frame, text="Lines ").pack(side="left")
                ttk.Label(compact_frame, textvariable=self.make_var("lines"), width=7).pack(side="left", padx=(0, 8))
                ttk.Label(compact_frame, text="Rate ").pack(side="left")
                ttk.Label(compact_frame, textvariable=self.make_var("rate"), width=12).pack(side="left")

            else:
                ttk.Label(frame, textvariable=self.make_var(key), wraplength=270).grid(row=r, column=1, sticky="w", padx=4, pady=2)

    def build_info_panel(
        self,
        parent: tk.Widget,
        title: str,
        fields: list[tuple[str, str]],
        value_columns: int = 1,
        row: Optional[int] = None,
        label_width: int = 17,
        value_width: int = 13,
        value_wraplength: int = 115,
    ) -> None:
        """Build a compact value panel with one or more label/value columns."""
        frame = ttk.LabelFrame(parent, text=title)
        if row is None:
            frame.pack(fill="x", pady=(8, 0))
        else:
            frame.grid(row=row, column=0, sticky="we", pady=(6, 0))

        value_columns = max(1, value_columns)
        rows_per_column = max(1, math.ceil(len(fields) / value_columns))

        for col in range(value_columns):
            base_col = col * 2
            frame.columnconfigure(base_col, weight=0)
            frame.columnconfigure(base_col + 1, weight=1)

        for idx, (label, key) in enumerate(fields):
            block_col = idx // rows_per_column
            row = idx % rows_per_column
            base_col = block_col * 2
            value_pad = (4, 12) if block_col < value_columns - 1 else 4

            ttk.Label(frame, text=label + ":", width=label_width).grid(
                row=row,
                column=base_col,
                sticky="w",
                padx=(4, 2),
                pady=2,
            )
            ttk.Label(frame, textvariable=self.make_var(key), width=value_width, wraplength=value_wraplength).grid(
                row=row,
                column=base_col + 1,
                sticky="w",
                padx=value_pad,
                pady=2,
            )

    def build_bms_values_panel(self, parent: tk.Widget, row: Optional[int] = None) -> None:
        self.build_info_panel(parent, "BMS Values", [
            ("Pack Voltage", "pack_voltage_v"),
            ("Current", "current_a"),
            ("SOC", "soc_percent"),
            ("Power Limit", "power_limit_kw"),
            ("Max Cell Voltage", "max_cell_voltage_v"),
            ("Min Cell Voltage", "min_cell_voltage_v"),
            ("Max Cell Temp", "max_cell_temp_c"),
            ("Min Cell Temp", "min_cell_temp_c"),
            ("VI Temp", "vi_pcb_temp_c"),
            ("HV Sense Temp", "hv_sense_temp_c"),
            ("Hall ADC Voltage", "hall_adc_voltage_v"),
            ("Current Zero Voltage", "current_zero_voltage"),
        ], value_columns=1, row=row, label_width=18, value_width=14, value_wraplength=170)

    def build_charger_panel(self, parent: tk.Widget, row: Optional[int] = None) -> None:
        self.build_info_panel(parent, "Charger", [
            ("Command", "charger_cmd_text"),
            ("Request Voltage", "charger_cmd_voltage"),
            ("Request Current", "charger_cmd_current"),
            ("Control Byte", "charger_cmd_control"),
            ("Status Voltage", "charger_status_voltage"),
            ("Status Current", "charger_status_current"),
            ("Status Byte", "charger_status_byte"),
            ("Faults", "charger_status_faults"),
        ], value_columns=1, row=row, label_width=18, value_width=14, value_wraplength=170)

    def build_can_panel(self, parent: tk.Widget, row: Optional[int] = None) -> None:
        frame = ttk.LabelFrame(parent, text="CAN / Serial")
        if row is None:
            frame.pack(fill="x", pady=(8, 0))
        else:
            frame.grid(row=row, column=0, sticky="we", pady=(6, 0))

        # Split CAN activity into TX and RX columns so the panel stays compact
        # and the two directions can be compared at a glance.
        frame.columnconfigure(0, weight=0)
        frame.columnconfigure(1, weight=1)
        frame.columnconfigure(2, weight=0)
        frame.columnconfigure(3, weight=1)

        ttk.Label(frame, text="TX", font=("Segoe UI", 9, "bold")).grid(
            row=0, column=0, columnspan=2, sticky="w", padx=4, pady=(2, 4)
        )
        ttk.Label(frame, text="RX", font=("Segoe UI", 9, "bold")).grid(
            row=0, column=2, columnspan=2, sticky="w", padx=(14, 4), pady=(2, 4)
        )

        rows = [
            ("Frames", "tx_count", "Frames", "rx_count"),
            ("Last", "last_tx", "Last", "last_rx"),
            ("IDs", "tx_id_counts", "IDs", "rx_id_counts"),
        ]

        for r, (tx_label, tx_key, rx_label, rx_key) in enumerate(rows, start=1):
            ttk.Label(frame, text=tx_label + ":", width=7).grid(row=r, column=0, sticky="w", padx=4, pady=2)
            ttk.Label(frame, textvariable=self.make_var(tx_key), wraplength=120).grid(row=r, column=1, sticky="w", padx=(0, 8), pady=2)
            ttk.Label(frame, text=rx_label + ":", width=7).grid(row=r, column=2, sticky="w", padx=(14, 2), pady=2)
            ttk.Label(frame, textvariable=self.make_var(rx_key), wraplength=120).grid(row=r, column=3, sticky="w", padx=(0, 4), pady=2)

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


    def update_activity_indicator(
        self,
        status_key: str,
        indicator_attr: str,
        indicator_id_attr: str,
        last_update_time: Optional[float],
        avg_refresh_ms: Optional[float],
        activity_flash_s: float,
    ) -> None:
        """Flash green on fresh data and show a rolling average refresh time."""
        now = time.time()
        just_received = (
            last_update_time is not None
            and (now - last_update_time) <= activity_flash_s
        )

        color = "#2ECC71" if just_received else "#FFD23F"
        text = "avg -- ms" if avg_refresh_ms is None else f"avg {avg_refresh_ms:.1f} ms"
        self.vars[status_key].set(text)

        indicator = getattr(self, indicator_attr, None)
        indicator_id = getattr(self, indicator_id_attr, None)
        if indicator is not None and indicator_id is not None:
            indicator.configure(bg=getattr(self, "current_bg", GUI_BG_NORMAL))
            indicator.itemconfig(indicator_id, fill=color)

    def update_cell_voltage_indicator(
        self,
        last_voltage_time: Optional[float],
        avg_refresh_ms: Optional[float],
    ) -> None:
        self.update_activity_indicator(
            "cell_voltage_status",
            "cell_voltage_indicator",
            "cell_voltage_indicator_id",
            last_voltage_time,
            avg_refresh_ms,
            CELL_VOLTAGE_ACTIVITY_FLASH_S,
        )

    def update_cell_temp_indicator(
        self,
        last_temp_time: Optional[float],
        avg_refresh_ms: Optional[float],
    ) -> None:
        self.update_activity_indicator(
            "cell_temp_status",
            "cell_temp_indicator",
            "cell_temp_indicator_id",
            last_temp_time,
            avg_refresh_ms,
            CELL_TEMP_ACTIVITY_FLASH_S,
        )

    def update_current_indicator(
        self,
        last_current_time: Optional[float],
        avg_refresh_ms: Optional[float],
    ) -> None:
        self.update_activity_indicator(
            "current_status",
            "current_indicator",
            "current_indicator_id",
            last_current_time,
            avg_refresh_ms,
            CURRENT_ACTIVITY_FLASH_S,
        )

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

        if hasattr(self, "cell_temp_indicator"):
            self.cell_temp_indicator.configure(bg=bg)

        if hasattr(self, "current_indicator"):
            self.current_indicator.configure(bg=bg)

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

    @staticmethod
    def median_item_value(items: list[dict[str, Any]]) -> Optional[float]:
        """Return the median of an item list that stores numeric values under item["value"]."""
        if not items:
            return None

        values: list[float] = []
        for item in items:
            try:
                value = float(item["value"])
            except (TypeError, ValueError, KeyError):
                continue
            if not math.isnan(value):
                values.append(value)

        if not values:
            return None

        values.sort()
        mid = len(values) // 2
        if len(values) % 2:
            return values[mid]
        return (values[mid - 1] + values[mid]) / 2.0

    def get_allboard_ranges(self, state: MonitorState) -> dict[str, Any]:
        """
        Compute min/max from ALL reported boards, grouped by physical location.

        Voltage mapping:
          odd boards  -> physical cells C1-C14
          even boards -> physical cells C15-C28

        Temperature mapping:
          odd boards in each selected pair  -> U1..U10
          even boards in each selected pair -> U11..U20

        Note: U# overlay coordinates are not moved.  Only the data routing
        from each board's 10 temperature values to physical U labels changes.

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
                # Board-local temperature index routing:
                #   odd board in pair  -> U1..U10
                #   even board in pair -> U11..U20
                # Do not move the U# overlay coordinates; only route values here.
                sensor = self.board_temp_sensor_number(board, i)
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
            "vmedian": self.median_item_value(all_voltage_items),
            "tmin": min(all_temp_items, key=lambda item: item["value"]) if all_temp_items else None,
            "tmax": max(all_temp_items, key=lambda item: item["value"]) if all_temp_items else None,
            "tmedian": self.median_item_value(all_temp_items),
            "has_voltage_fault": has_voltage_fault,
            "has_temp_fault": has_temp_fault,
            "has_1p5v": has_1p5v,
        }


    def board_temp_sensor_number(self, board: int, temp_index: int) -> int:
        """
        Route a board-local temperature index to the fixed physical U label.

        For every board pair:
          odd board  -> temp[0..9] maps to U1..U10
          even board -> temp[0..9] maps to U11..U20

        Examples:
          board 1 temp[9] -> U10
          board 2 temp[0] -> U11
          board 2 temp[9] -> U20

        The LEFT_TEMP_COORDS / RIGHT_TEMP_COORDS dictionaries remain the source
        of truth for where each U# is drawn.  This function only decides which
        board temperature value is routed to each U#.
        """
        if board % 2 == 1:
            return temp_index + 1
        return temp_index + 11

    def get_board_temp_sensors(self, state: MonitorState, board: int) -> dict[int, Optional[float]]:
        values = state.board_temps.get(board, [])
        out: dict[int, Optional[float]] = {}

        for i in range(10):
            sensor = self.board_temp_sensor_number(board, i)

            if i < len(values) and self.is_valid_thermistor_temp(values[i]):
                out[sensor] = values[i]
            else:
                out[sensor] = None

        return out

    def get_pair_temp_sensors(self, state: MonitorState, odd_board: int, even_board: int) -> dict[int, Optional[float]]:
        """
        Combine one board pair into a single U1..U20 temperature map.

        Odd board values fill U1..U10.
        Even board values fill U11..U20.
        """
        out: dict[int, Optional[float]] = {}
        out.update(self.get_board_temp_sensors(state, odd_board))
        out.update(self.get_board_temp_sensors(state, even_board))
        return out

    def draw_legend(self) -> None:
        if not hasattr(self, "legend"):
            return
        c = self.legend
        c.delete("all")

        width = max(260, c.winfo_width() or int(float(c.cget("width") or 600)))
        x = 8
        y = 14
        row_h = 24

        def wrap(item_w: int) -> None:
            nonlocal x, y
            if x > 8 and x + item_w > width - 8:
                x = 8
                y += row_h

        for pair in range(1, 6):
            color = PAIR_COLORS[pair]
            b1 = pair * 2 - 1
            b2 = pair * 2
            item_w = 135
            wrap(item_w)
            c.create_rectangle(x, y - 8, x + 18, y + 10, fill=color, outline="#FFFFFF", width=1)
            c.create_text(
                x + 28,
                y + 1,
                anchor="w",
                text=f"Boards {b1}/{b2}",
                fill="#F2F2F2",
                font=("Segoe UI", 9, "bold"),
            )
            x += item_w

        temp_text = "Temp border: RGB green→yellow→red at 60C+    -55C ignored"
        voltage_text = "Voltage border: purple 2.5V → blue 4.2V    Fill = board color    Min/Max = all boards / every location"

        for kind, text in (("temp", temp_text), ("voltage", voltage_text)):
            item_w = min(width - 16, 520 if kind == "temp" else 650)
            wrap(item_w)
            if kind == "temp":
                c.create_oval(x, y - 9, x + 18, y + 9, fill=PAIR_COLORS[1], outline=temp_color(60), width=3)
            else:
                c.create_rectangle(x, y - 8, x + 18, y + 10, fill=PAIR_COLORS[1], outline=voltage_color(3.4), width=3)
            c.create_text(
                x + 28,
                y,
                anchor="w",
                text=text,
                fill="#F2F2F2",
                font=("Segoe UI", 9, "bold"),
                width=max(160, width - x - 40),
            )
            x += item_w

        desired_h = max(54, y + row_h)
        if int(float(c.cget("height"))) != desired_h:
            c.configure(height=desired_h)

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
            if "log_status" in self.vars:
                self.vars["log_status"].set(state.log_status)
            if hasattr(self, "serial_command_status_var"):
                command_status = state.serial_command_status
                if state.last_serial_command_sent_time is not None:
                    age_s = now - state.last_serial_command_sent_time
                    command_status = f"{command_status} ({age_s:.1f}s ago)"
                self.serial_command_status_var.set(command_status)

            for key in [
                "pack_voltage_v",
                "current_a",
                "soc_percent",
                "power_limit_kw",
                "max_cell_voltage_v",
                "min_cell_voltage_v",
                "max_cell_temp_c",
                "min_cell_temp_c",
                "vi_pcb_temp_c",
                "hv_sense_temp_c",
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

            tx_count_items = [f"{k[3:]}:{v}" for k, v in sorted(state.id_counts.items()) if k.startswith("TX ")]
            rx_count_items = [f"{k[3:]}:{v}" for k, v in sorted(state.id_counts.items()) if k.startswith("RX ")]
            self.vars["tx_id_counts"].set("   ".join(tx_count_items[:4]) if tx_count_items else "NaN")
            self.vars["rx_id_counts"].set("   ".join(rx_count_items[:4]) if rx_count_items else "NaN")

            if "id_counts" in self.vars:
                count_items = [f"{k}:{v}" for k, v in sorted(state.id_counts.items())[:8]]
                self.vars["id_counts"].set("   ".join(count_items) if count_items else "NaN")

            allboard_ranges = self.get_allboard_ranges(state)
            computed_pack_voltage = calculated_pack_voltage(state)
            if computed_pack_voltage is not None:
                record_time_series_sample(state, now, "pack_voltage_v", computed_pack_voltage, "V")
            if allboard_ranges["vmin"]:
                record_time_series_sample(state, now, "min_cell_voltage_v", allboard_ranges["vmin"]["value"], "V")
            if allboard_ranges["vmax"]:
                record_time_series_sample(state, now, "max_cell_voltage_v", allboard_ranges["vmax"]["value"], "V")
            if allboard_ranges["tmin"]:
                record_time_series_sample(state, now, "min_cell_temp_c", allboard_ranges["tmin"]["value"], "C")
            if allboard_ranges["tmax"]:
                record_time_series_sample(state, now, "max_cell_temp_c", allboard_ranges["tmax"]["value"], "C")
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

            last_temp_time_for_indicator = state.last_cell_temp_update_time
            if state.cell_temp_refresh_intervals_ms:
                avg_temp_refresh_ms = (
                    sum(state.cell_temp_refresh_intervals_ms)
                    / len(state.cell_temp_refresh_intervals_ms)
                )
            else:
                avg_temp_refresh_ms = None

            last_current_time_for_indicator = state.last_current_update_time
            if state.current_refresh_intervals_ms:
                avg_current_refresh_ms = (
                    sum(state.current_refresh_intervals_ms)
                    / len(state.current_refresh_intervals_ms)
                )
            else:
                avg_current_refresh_ms = None

            min_v = allboard_ranges["vmin"]["value"] if allboard_ranges["vmin"] else get_latest_float(state, "min_cell_voltage_v")
            max_v = allboard_ranges["vmax"]["value"] if allboard_ranges["vmax"] else get_latest_float(state, "max_cell_voltage_v")
            median_v = allboard_ranges.get("vmedian")
            delta_v = (max_v - min_v) if min_v is not None and max_v is not None else None

            min_t = allboard_ranges["tmin"]["value"] if allboard_ranges["tmin"] else get_latest_float(state, "min_cell_temp_c")
            max_t = allboard_ranges["tmax"]["value"] if allboard_ranges["tmax"] else get_latest_float(state, "max_cell_temp_c")
            median_t = allboard_ranges.get("tmedian")
            delta_t = (max_t - min_t) if min_t is not None and max_t is not None else None

            current_min, current_max, current_now = recent_current_minmax(state, now)
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
            pair_temps = self.get_pair_temp_sensors(state, odd_board, even_board)
            left_temps = pair_temps
            right_temps = pair_temps
            overlay_filter = self.overlay_filter.get()
            time_series_samples = list(state.time_series)

        flash_on = (int(time.time() / FLASH_PERIOD_S) % 2) == 0
        self.apply_alarm_background(allboard_ranges)
        self.update_serial_indicator(serial_status_for_indicator, last_line_time_for_indicator)
        self.update_cell_voltage_indicator(last_voltage_time_for_indicator, avg_voltage_refresh_ms)
        self.update_cell_temp_indicator(last_temp_time_for_indicator, avg_temp_refresh_ms)
        self.update_current_indicator(last_current_time_for_indicator, avg_current_refresh_ms)

        self.voltage_dial.draw(min_v, max_v, median_reading=median_v, delta_reading=delta_v)
        self.temp_dial.draw(min_t, max_t, median_reading=median_t, delta_reading=delta_t)
        self.current_dial.draw(current_min, current_max, current_now, "NOW")
        if self.view_mode.get() == "graph":
            self.graph_view.draw_samples(time_series_samples, now)
        else:
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
        # Periodic geometry check catches Windows maximize/restore transitions that
        # do not deliver a useful left-pane Configure event.
        if hasattr(self, "left_panel"):
            geom = (self.left_panel.winfo_width(), self.left_panel.winfo_height())
            if geom != getattr(self, "_last_left_layout_geometry", (0, 0)):
                self.request_left_panel_resize()
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
    parser.add_argument("--baud", type=int, default=115200, help="Serial baud rate. Current BMS code uses Serial.begin(115200).")
    parser.add_argument("--log-dir", default="BMS_Serial_Laptop_Recieved", help="Folder for timestamped CSV logs")
    parser.add_argument("--left-image", help="Path to Left_Side_Module.png")
    parser.add_argument("--right-image", help="Path to Right_Side_Module.png")
    parser.add_argument("--raw", action="store_true", help="Also echo every raw serial line to the console")
    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return 0

    port = args.port
    if not port and args.auto:
        port = auto_select_port()

    if not port and not args.auto:
        print("No serial port selected.")
        print()
        list_serial_ports()
        print()
        print("Examples:")
        print("  python bms_serial_gui.py --port COM7 --baud 115200")
        print("  python bms_serial_gui.py --auto --baud 115200")
        return 2

    if not port and args.auto:
        print("No serial port found yet. The GUI will stay open and retry automatically.")

    try:
        left_image = resolve_image_path(args.left_image, "Left_Side_Module.png")
        right_image = resolve_image_path(args.right_image, "Right_Side_Module.png")
    except FileNotFoundError as exc:
        print(exc)
        return 2

    state = MonitorState()
    lock = threading.RLock()
    stop_event = threading.Event()
    status_queue: "queue.Queue[str]" = queue.Queue()
    command_queue: "queue.Queue[str]" = queue.Queue()

    worker = SerialWorker(
        state=state,
        state_lock=lock,
        stop_event=stop_event,
        port=port,
        baud=args.baud,
        log_dir=Path(args.log_dir),
        status_queue=status_queue,
        command_queue=command_queue,
        raw_echo=args.raw,
        auto_port=args.auto,
    )
    worker.start()

    root = tk.Tk()
    app = BMSGuiApp(root, state, lock, stop_event, command_queue, left_image, right_image)
    app.status_queue = status_queue
    root.mainloop()

    stop_event.set()
    worker.join(timeout=1.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
