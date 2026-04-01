import serial
import time
import os

# --- CONFIGURATION ---
SERIAL_PORT = '/dev/ttyACM0' 
BAUD_RATE = 9600

if not os.path.exists("data"):
    os.makedirs("data")

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Error: {e}")
    exit()

# --- PART 1: HANDSHAKE ---
print("Attempting to enter Debug Mode...")
entered_dump_mode = False
first_filename = None

while not entered_dump_mode:
    if ser.in_waiting > 0:
        try:
            raw_line = ser.readline()
            line = raw_line.decode('utf-8', errors='ignore').strip()
        except:
            continue
        
        if "bytes" in line:
            print(f"BMS Startup Info: {line}")
            continue

        print(f"BMS: {line}")

        if "Setup" in line or "Voltages:" in line:
            print(">>> Stuck in Setup. Sending 'debug'...")
            ser.write(b"debug\n")
            time.sleep(0.5)

        elif "Debug Mode Entered" in line:
            print(">>> Debug Mode Confirmed. Sending 'begin'...")
            ser.write(b"begin\n")

        elif (line.endswith(".csv") or line.endswith(".txt")) and "bytes" not in line:
            print(">>> Download starting!")
            entered_dump_mode = True
            first_filename = line
            break

# --- PART 2: DOWNLOAD LOOP ---
print(f"--- Starting Data Transfer ---")
current_file = None

if entered_dump_mode and first_filename:
    filename = os.path.join("data", first_filename)
    print(f"Creating file: {filename}")
    current_file = open(filename, 'w')

last_data_time = time.time()

try:
    while True:
        # A. READ RAW DATA
        # We read raw bytes first to distinguish "Timeout" from "Empty Line"
        raw_line = ser.readline()
        
        # B. CHECK FOR TIMEOUT (THE KICK)
        if len(raw_line) == 0:
            # No data received within timeout (1 second)
            if time.time() - last_data_time > 2.0 and current_file:
                print("...Stalled. Kicking Arduino...")
                ser.write(b"next line\n")
                ser.flush() # Ensure message goes out
                last_data_time = time.time()
            continue

        # If we got here, we received SOMETHING (even just a newline)
        last_data_time = time.time()
        
        try:
            line = raw_line.decode('utf-8', errors='ignore').strip()
        except:
            continue

        # C. PROCESS COMMANDS
        if line == "serial dump done":
            print("\nSUCCESS: All files transferred.")
            break

        elif line == "done":
            if current_file:
                current_file.close()
                current_file = None
                print("File saved. Requesting next file...")
            
            time.sleep(0.1) 
            ser.write(b"next file\n")
            ser.flush()

        elif (line.endswith(".csv") or line.endswith(".txt")) and "bytes" not in line:
            if current_file:
                current_file.close()
            
            filename = os.path.join("data", line)
            print(f"Creating new file: {filename}")
            current_file = open(filename, 'w')

        # D. PROCESS DATA (This is the fix)
        else:
            if current_file:
                # Even if 'line' is empty string, we might want to write a newline to file
                # But more importantly, we MUST tell Arduino to continue
                if line:
                    current_file.write(line + "\n")
                    current_file.flush()
                
                # ALWAYS send handshake if we have a file open, 
                # even if the line was blank/whitespace
                ser.write(b"next line\n")
            else:
                if line: print(f"Info: {line}")

except KeyboardInterrupt:
    print("\nScript stopped by user.")

finally:
    if current_file:
        current_file.close()
    ser.close()