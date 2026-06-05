"""
PlatformIO custom target for the BMS Python serial monitor.

Adds this PlatformIO target:
  pio run -e teensy41 -t bms_monitor

In VS Code PlatformIO, it should appear under:
  Project Tasks -> teensy41 -> Custom -> bms_monitor
"""

from __future__ import annotations

Import("env")  # PlatformIO/SCons provided

import os
import subprocess
import sys
from pathlib import Path


def _project_option(name: str, default: str = "") -> str:
    try:
        value = env.GetProjectOption(name, default)
    except Exception:
        value = default
    if value is None:
        return default
    return str(value)


def _run_bms_monitor(source, target, env):
    project_dir = Path(env.subst("$PROJECT_DIR"))
    monitor_script = project_dir / "tools" / "bms_serial_monitor.py"

    if not monitor_script.exists():
        print(f"BMS monitor script not found: {monitor_script}")
        print("Expected file location: tools/bms_serial_monitor.py")
        return 1

    baud = _project_option("monitor_speed", "9600")
    monitor_port = _project_option("monitor_port", "")
    upload_port = _project_option("upload_port", "")
    port = monitor_port or upload_port

    cmd = [sys.executable, str(monitor_script), "--baud", baud]
    if port:
        cmd += ["--port", port]
    else:
        cmd += ["--auto"]

    # Keep the display human-scale and log everything.
    cmd += ["--refresh", "1.0", "--dashboard", "clear"]

    print("Running BMS Python serial monitor:")
    print(" ".join(cmd))
    print()
    print("Press Ctrl+C to stop the monitor.")
    print()

    return subprocess.call(cmd, cwd=str(project_dir))


env.AddCustomTarget(
    name="bms_monitor",
    dependencies=None,
    actions=_run_bms_monitor,
    title="BMS Serial Monitor",
    description="Run the custom BMS Python serial monitor/logger instead of flooding the normal serial monitor.",
)
