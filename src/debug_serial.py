import serial

ser = serial.Serial('COM11', 9600, timeout=1)  # open serial port
while(1):
    line = ser.readline().decode('utf-8', 'ignore').rstrip()
    
    print(line)