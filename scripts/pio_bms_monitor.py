from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any


# PlatformIO injects Import("env") when running extra_scripts.
# Pylance does not know about it, so we ignore that one name.
env: Any

try:
    Import("env")  # type: ignore[name-defined]
except NameError:
    # This only happens if you open/run this file outside PlatformIO.
    env = None


def require_platformio_env() -> Any:
    if env is None:
        raise RuntimeError(
            "This script must be run by PlatformIO as an extra_script, "
            "not directly with Python."
        )
    return env


def get_option(name: str, default: str = "") -> str:
    e = require_platformio_env()

    try:
        value = e.GetProjectOption(name, default)
    except TypeError:
        try:
            value = e.GetProjectOption(name)
        except Exception:
            value = default
    except Exception:
        value = default

    if value is None:
        return default

    value = str(value).strip()
    return value if value else default


def quote_arg(arg: str) -> str:
    if " " in arg:
        return f'"{arg}"'
    return arg


def run_bms_monitor(target: Any = None, source: Any = None, env: Any = None) -> int:
    e = require_platformio_env()

    project_dir = Path(e.subst("$PROJECT_DIR"))
    monitor_script = project_dir / "tools" / "bms_serial_monitor.py"

    if not monitor_script.exists():
        print()
        print("ERROR: Could not find BMS monitor script:")
        print(f"  {monitor_script}")
        print()
        print("Expected location:")
        print("  tools/bms_serial_monitor.py")
        return 1

    baud = get_option("custom_bms_monitor_baud", get_option("monitor_speed", "9600"))
    refresh = get_option("custom_bms_monitor_refresh", "5.0")
    dashboard = get_option("custom_bms_monitor_dashboard", "clear")
    log_dir = get_option("custom_bms_monitor_log_dir", "BMS_Serial_Laptop_Recieved")
    port = get_option("custom_bms_monitor_port", get_option("monitor_port", ""))

    cmd = [
        sys.executable,
        str(monitor_script),
        "--baud",
        baud,
        "--refresh",
        refresh,
        "--dashboard",
        dashboard,
        "--log-dir",
        log_dir,
    ]

    if port and port.lower() != "auto":
        cmd += ["--port", port]
    else:
        cmd += ["--auto"]

    print()
    print("Running BMS Python serial monitor:")
    print("  " + " ".join(quote_arg(str(x)) for x in cmd))
    print()

    return subprocess.call(cmd, cwd=str(project_dir))


e = require_platformio_env()

e.AddCustomTarget(
    name="bms_monitor",
    dependencies=None,
    actions=[
        e.VerboseAction(
            run_bms_monitor,
            "Starting BMS Serial Monitor",
        )
    ],
    title="BMS Serial Monitor",
    description="Run the BMS timestamped CSV logger and fixed-height dashboard",
)