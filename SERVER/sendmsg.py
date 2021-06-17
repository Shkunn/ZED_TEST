import socket
import time
import argparse

IP   = "172.21.72.116"
PORT = 8080

# msg = 0
# cc = bytes([msg])
parser = argparse.ArgumentParser()
parser.add_argument("msg")
args = parser.parse_args()
msg = str(args.msg)
cc = msg.encode()

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
    opened_socket.setblocking(0)
    opened_socket.sendto(cc, (IP, PORT))