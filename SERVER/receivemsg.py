import socket
import pickle
import json

# Define the IP address and the Port Number
IP    = "172.21.72.126"
# IP    = "192.168.1.71"
PORT  = 8080

listeningAddress = (IP, PORT)



b = b''
datagramSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
datagramSocket.bind(listeningAddress)

while 1:
    localization, sourceAddress = datagramSocket.recvfrom(10000)
    print("LEN: ", len(localization))
    # print("MESSAGE : ", type(localization.decode('utf-8')))
    str_msg = localization.decode('utf-8')
    print("STR MSG: ", str_msg)

    json_msg = json.loads(str_msg)


