import json
import socket
import pickle
import numpy as np

IP   = "172.21.72.133"
# IP   = "192.168.1.71"

PORT = 8080

HEADERSIZE = 10


# s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
# s.bind((IP, PORT))
# s.listen(5)


#region TEST
# extracted_points = """
# {
#     "Points": [
        
#     ],
#     "Human": [
#         {
#             "number": "2"
#         },
#         {
#             "A": "[5,10]",
#             "D": "[10,10]"
#         },
#         {
#             "A": "[5,10]",
#             "D": "[10,10]"
#         }
#     ]
# }
# """

# data = json.loads(extracted_points)
# print(type(data))

# print(data['Points'][0]['X'])

# new_string = json.dumps(data, indent=2)
# print(new_string)
#endregion

# Creating an empty dictionary
# myDict = {}
  
# # Adding list as value
# myDict["Points"] = []
# myDict["Human"] = []

# creating a list
A = ['5', '10', '15']
# B = ['5', '10', '15']

# human_1 = [['5', '10'], ['15', '20']]
# human_2 = [['15', '20'], ['35', '23']]
  
# # Adding this list as sublist in myDict
# myDict["Points"].append(A)
# myDict["Points"].append(B)

# myDict["Human"].append(human_1)
# myDict["Human"].append(human_2)

  
# print(myDict)
# print()

# new_string = json.dumps(myDict, indent=2)
# print(new_string)

# print(myDict['Human'])



"""
SEND MESSAGE
"""
# msg = pickle.dumps(myDict)

# with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
#     opened_socket.setblocking(0)
#     opened_socket.sendto(msg, (IP, PORT))


"""
TEST WITH NUMPY ARRAYS
"""
print("** NEW ***")
# Creating an empty dictionary
myDict = {}
  
# # Adding list as value
myDict["Points"] = []
myDict["Human"] = []
print(myDict)


mat = np.array([])
mat = np.append(mat, A)

for a in range(20):
    mat = np.vstack([mat, A])



mat_tolist = mat.tolist()
myDict["Points"].extend(mat_tolist)

print("MY DICT: ", myDict)

# msg = pickle.dumps(myDict)
# msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')+msg
# print("PICKLE DUMPS: ", msg)

b = json.dumps(myDict).encode('utf-8')
# print(b)


# def mysend(self, msg):
#     totalsent = 0
#     while totalsent < len(b):
#         sent = sock.send(msg[totalsent:])
#         if sent == 0:
#             raise RuntimeError("socket connection broken")
#         totalsent = totalsent + sent

with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
    opened_socket.setblocking(0)
    opened_socket.connect((IP, PORT))
    opened_socket.sendall(b)
    # opened_socket.sendto(b, (IP, PORT))

i = 0

# while mat.shape[0] < 10:
    
#     mat = np.vstack([mat, A])
    
#     if mat.shape[0] == 2:

#         mat_tolist = mat.tolist()
#         myDict["Points"].extend(mat_tolist)

#         # print(myDict)
                
#         # new_string = json.dumps(myDict, indent=2)
#         # print(new_string)

#         # while True:
#             # now our endpoint knows about the OTHER endpoint.
#             # clientsocket, address = s.accept()
#             # print(f"Connection from {address} has been established.")

#         msg = pickle.dumps(myDict)
#         msg = bytes(f"{len(msg):<{HEADERSIZE}}", 'utf-8')+msg
#         print(msg)

#         with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as opened_socket:
#             opened_socket.setblocking(0)
#             opened_socket.sendto(msg, (IP, PORT))


#         """
#         RESTART ALL VALUES
#         """
#         mat = np.array([])
#         myDict["Points"] = []
#         myDict["Human"] = []
#         mat = np.append(mat, A)

#         i += 1

#     if i > 0:
#         break




# mat = np.random.rand(5,3)

# print("SHAPE: ", mat.shape[0])
# print("TYPE: ", type(mat))
# print("MAT: ", mat)

# mat_tolist = mat.tolist()


# """
# ADD TO DICT
# """
# myDict["Points"].extend(mat_tolist)


# """
# PRINT
# """
# print()
# print("DICT:\n", myDict)
# print()

# new_string = json.dumps(myDict, indent=2)
# print(new_string)

# """
# RESTART ARRAY
# """
# mat = np.array([])
# mat = np.append(mat, A)
# while mat.shape[0] < 30:
#     mat = np.vstack([mat, A])

# print("MAT:\n", mat)
