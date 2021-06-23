# set async_mode to 'threading', 'eventlet', 'gevent' or 'gevent_uwsgi' to
# force a mode else, the best mode is selected automatically from what's
# installed
async_mode = None

from flask import Flask, render_template
from jinja2.environment import copy_cache
import socketio
import json
import numpy as np
import threading
import time


sio = socketio.Server(async_mode='threading', cors_allowed_origins='*')
app = Flask(__name__)
app.wsgi_app = socketio.WSGIApp(sio, app.wsgi_app)

# sio = socketio.Server(cors_allowed_origins='*')

myDict = {}
b = {}
copy_b = {}

@app.route('/')
def index():
    return render_template('latency.html')

def thead_create_mydict():
    global myDict, b
    # myDict = {}

    i = 0

    while i < 10000:
    # # Adding list as value
        myDict["Human"] = {}

        A = [str(i), str(i+1), str(i+1), str(i+2)]

        mat = np.array([])
        mat = np.append(mat, A)

        # for a in range(2):
        #     mat = np.vstack([mat, A])


        mat_tolist = mat.tolist()
        myDict["Human"]["0"] = mat_tolist

        b = json.dumps(myDict)

        # print("thead_create_mydict: ", b)
        
        i += 1
        time.sleep(2)  


@sio.event
def ping_from_client(sid):
    global myDict, b

    sio.emit('pong_from_server', b, room=sid)

def thread_server():
    if sio.async_mode == 'threading':
        # deploy with Werkzeug
        # app.run(threaded=True)
        # app.run(host='172.21.72.133', 
        #     port=5000)

        app.run(host='192.168.1.71',
                port=5000)
        
    else:
        print('Unknown async_mode: ' + sio.async_mode)


# def thread_test():
#     i = 0
#     while i < 1000000:
#         print(i)
#         i += 1
#         time.sleep(1)


if __name__ == '__main__':
    # thread_5 = threading.Thread(target=thread_test    , args=())
    # thread_5.start()

    thread_6 = threading.Thread(target=thread_server    , args=())
    thread_6.start()

    thread_7 = threading.Thread(target=thead_create_mydict    , args=())
    thread_7.start()

    # thread_5.join()
    thread_6.join()
    thread_7.join()