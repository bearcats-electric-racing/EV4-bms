from __future__ import annotations

import subprocess
import sys
from pathlib import Path
from typing import Any

# PlatformIO injects Import("env") when running extra_scripts.
env: Any
try:
    Import("env")  # type: ignore[name-defined]
except NameError:
    env = None


def require_platformio_env() -> Any:
    if env is None:
        raise RuntimeError("This script must be run by PlatformIO as an extra_script.")
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


def run_bms_gui(target: Any = None, source: Any = None, env: Any = None) -> int:
    e = require_platformio_env()
    project_dir = Path(e.subst("$PROJECT_DIR"))
    gui_script = project_dir / "tools" / "bms_serial_gui.py"

    if not gui_script.exists():
        print()
        print("ERROR: Could not find BMS GUI script:")
        print(f"  {gui_script}")
        print()
        return 1

    baud = get_option("custom_bms_gui_baud", get_option("monitor_speed", "9600"))
    log_dir = get_option("custom_bms_gui_log_dir", "BMS_Serial_Laptop_Recieved")
    port = get_option("custom_bms_gui_port", get_option("monitor_port", "auto"))

    cmd = [
        sys.executable,
        str(gui_script),
        "--baud",
        baud,
        "--log-dir",
        log_dir,
    ]

    if port and port.lower() != "auto":
        cmd += ["--port", port]
    else:
        cmd += ["--auto"]

    print()
    print("Running BMS Python GUI:")
    print("  " + " ".join(str(x) for x in cmd))
    print()

    return subprocess.call(cmd, cwd=str(project_dir))


e = require_platformio_env()
e.AddCustomTarget(
    name="bms_gui",
    dependencies=None,
    actions=[
        e.VerboseAction(
            run_bms_gui,
            "Starting BMS Serial GUI",
        )
    ],
    title="BMS Serial GUI",
    description="Run the BMS visual GUI, timestamped CSV logger, and serial monitor",
)
