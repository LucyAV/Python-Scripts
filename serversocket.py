import socket
import serial

HOST = "10.0.0.88"
PORT = 6969
sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
print("SOCKET: Created")

ser = serial.Serial("/dev/ttyACM1", 9600)

try:
    sock.bind((HOST, PORT))
except socket.error:
    print("[ERROR] %s\n" & msg[1])

sock.listen(1)
print("SOCKET: Awaiting connection")
(conn, addr) = sock.accept()
print("SOCKET: Connected, awaiting messages")

def sendCmd(value):
    mybytes = bytearray()
    mybytes.append(value)
    ser.write(mybytes)

while True:
    data = conn.recv(1)
    if data == b'\x01': #stop
        print("CMD: STOP")
        sendCmd(1)
    if data == b'\x02': #go
        print("CMD: GO")
        sendCmd(2)
    if data == b'\x03': #end connection
        print("CMD: END")
#        conn.shutdown(0)
#conn.close()
