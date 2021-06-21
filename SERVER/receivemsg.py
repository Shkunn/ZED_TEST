# import socket
# import pickle

# # Define the IP address and the Port Number
# IP    = "172.21.72.133"
# PORT  = 8080

# listeningAddress = (IP, PORT)

# # Create a datagram based server socket that uses IPv4 addressing scheme
# datagramSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
# datagramSocket.bind(listeningAddress)

# while(True):
#     localization, sourceAddress = datagramSocket.recvfrom(128)
#     d = pickle.loads(localization)
#     print(d)
#     # print(localization.decode())




import socket
import pickle
import json

HEADERSIZE = 10


# Define the IP address and the Port Number
IP    = "172.21.72.126"
# IP    = "192.168.1.71"
PORT  = 8080

listeningAddress = (IP, PORT)

# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.connect((IP, PORT))

b = b''
datagramSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
datagramSocket.bind(listeningAddress)

while 1:
    # print("hell")
    localization, sourceAddress = datagramSocket.recvfrom(10000)
    print("LEN: ", len(localization))
    # print("MESSAGE : ", type(localization.decode('utf-8')))
    str_msg = localization.decode('utf-8')
    print("STR MSG: ", str_msg)

    json_msg = json.loads(str_msg)
    # print("JSON: ", json_msg)

    # d = json.loads((localization.decode('utf-8'))
    # print(d)

# d = pickle.loads(localization)
# print(d)
# print(localization.decode())

# while True:
#     full_msg = b''
#     new_msg = True
#     while True:
#         msg = datagramSocket.recv(16)
#         if new_msg:
#             print("new msg len:",msg[:HEADERSIZE])
#             msglen = int(msg[:HEADERSIZE])
#             new_msg = False

#         print(f"full message length: {msglen}")

#         full_msg += msg

#         print(len(full_msg))

#         if len(full_msg)-HEADERSIZE == msglen:
#             print("full msg recvd")
#             print(full_msg[HEADERSIZE:])
#             print(pickle.loads(full_msg[HEADERSIZE:]))
#             new_msg = True
#             full_msg = b""




# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.bind(listeningAddress)
# s.listen(1)
# conn, addr = s.accept()
# b = b''
# while 1:
#     tmp = conn.recv(1024)
#     b += tmp
# d = json.loads(b.decode('utf-8'))
# print(d)
