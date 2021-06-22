# import asyncio
import time
import socketio
import numpy as np
import json


sio = socketio.Client(logger=True, engineio_logger=True)
start_timer = None


def send_data(data):
    global start_timer
    start_timer = time.time()
    sio.emit('data', data)


if __name__ == '__main__':
    sio.connect('http://localhost:5000')

    myDict = {}
    
    # # Adding list as value
    myDict["Human"] = []
    print(myDict)

    A = ['10', '15', '20', '30']

    mat = np.array([])
    mat = np.append(mat, A)

    for a in range(2):
        mat = np.vstack([mat, A])


    mat_tolist = mat.tolist()
    myDict["Human"].extend(mat_tolist)

    b = json.dumps(myDict)

    send_data(b)