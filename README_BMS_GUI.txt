BMS Serial GUI Package
======================

Files:
  tools/bms_serial_gui.py
  tools/Left_Side_module.png
  tools/Right_Side_Module.png
  requirements_bms_gui.txt
  scripts/pio_bms_gui.py   optional PlatformIO custom target

Install dependencies:
  python -m pip install -r requirements_bms_gui.txt

Run directly:
  python tools\bms_serial_gui.py --auto --baud 9600

Or with a known port:
  python tools\bms_serial_gui.py --port COM7 --baud 9600

What it does:
  - Reads the Teensy serial stream.
  - Logs every received serial line to BMS_Serial_Laptop_Recieved/BMS_Serial_<timestamp>.csv.
  - Shows the same core BMS values, charger values, CAN/serial status, active alerts, and recent events as the terminal dashboard.
  - Shows min/max cell voltage on one dial from 2.5 V to 4.2 V.
  - Shows min/max cell temperature on one dial with 60 C+ redline.
  - Maps selected board-pair voltages and temperatures onto the module images.

Board-pair mapping:
  Pair 1 = boards 1/2
  Pair 2 = boards 3/4
  Pair 3 = boards 5/6
  Pair 4 = boards 7/8
  Pair 5 = boards 9/10

Odd board display:
  - Left image
  - Voltages: Cell 1 to Cell 14
  - Temperatures: U1 to U10

Even board display:
  - Right image
  - Voltages: Cell 15 to Cell 28
  - Temperatures: U11 to U20

If the overlay is slightly off:
  Open tools/bms_serial_gui.py and edit:
    LEFT_CELL_COORDS
    RIGHT_CELL_COORDS
    LEFT_TEMP_COORDS
    RIGHT_TEMP_COORDS
The coordinates are in the original image pixel space. The GUI scales them automatically.
