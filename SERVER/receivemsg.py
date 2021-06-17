import socket

# Define the IP address and the Port Number
IP    = "172.21.72.126"
PORT  = 8080

listeningAddress = (IP, PORT)

# Create a datagram based server socket that uses IPv4 addressing scheme
datagramSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
datagramSocket.bind(listeningAddress)

while(True):
    localization, sourceAddress = datagramSocket.recvfrom(128)
    print(localization.decode())