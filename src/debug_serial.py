import serial

ser = serial.Serial('COM11', 38400, timeout=5)  # open serial port
while(1):
    line = ser.readline().decode('utf-8', 'ignore').rstrip()
    
    print(line)