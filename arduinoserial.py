import serial

ser = serial.Serial('/dev/ttyACM0', 9600)
#ser.baudrate = 9600

def off():
    mybytes = bytearray()
    mybytes.append(1)
    ser.write(mybytes)

def on():
    mybytes = bytearray()
    mybytes.append(2)
    ser.write(mybytes)
