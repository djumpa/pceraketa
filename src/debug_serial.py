import serial

ser = serial.Serial('COM11', 115200, timeout=5)  # open serial port
while(1):
    line = ser.readline().decode('utf-8', 'ignore').rstrip()
    
    print(line)