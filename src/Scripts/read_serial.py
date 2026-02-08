import serial
import serial.tools.list_ports
import time

#initialize serial port

ser = serial.Serial()
ports = serial.tools.list_ports.comports()
port_num = 13
ser.port = f'COM{port_num}' #Arduino serial port  - try statement to find proper serial port
#ser.baudrate = 100000
ser.baudrate = 9600
ser.timeout = 1000 #specify timeout when using readline() in ms
ser.open()

def read_line():
    line = ""
    while True:
        if ser.in_waiting > 0:
            byte = ser.read()         # Read one byte
            char = byte.decode('utf-8', errors='ignore')  # Convert byte to string
            if char == '\n':
                print(line)
                return line.strip()   # Remove trailing spaces/newlines
            else:
                line += char

if ser.is_open==True:
    print("\nAll right, serial port now open. Configuration:\n")
    print(ser, "\n") #print serial parameters
    ser.write(b"debug\n")
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.write(b'begin\n')
    start_time = time.time()
    line = ""
    while(line != "serial dump done"):
        line = ser.readline().decode('utf-8').strip()
        with open(f"data\{line}", 'w') as file:

            while(line.strip() != "done"):
                line = ser.readline().decode('utf-8')
                file.write(line)
                print(line)
                ser.write(b'next line\n')
                #print(line)

            # while(line.strip() != "done"):
            #     line = read_line()
            #     print(line)
           
            file.close()
            ser.write(b"next file\n")
