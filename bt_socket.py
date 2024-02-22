import socket

bmac = "A0:A3:B3:2B:F2:2A"

connection = socket.socket(socket.AF_BLUETOOTH, socket.SOCK_STREAM, socket.BTPROTO_RFCOMM)
connection.connect((bmac, 1))
while True:
    data = connection.recv(1024)
    print(data)
    connection.send("OK\n".encode())
connection.close()